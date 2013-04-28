/*
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *		JZ4720/JZ4740 SoC LCD framebuffer driver
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>

#include <linux/clk.h>
#include <linux/delay.h>

#include <linux/console.h>
#include <linux/fb.h>

#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include <asm/mach-jz4740/dma.h>
#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>

#include "jz4740_lcd.h"
#include "jz4740_slcd.h"

#define FB_A320TV_OFF 0
#define FB_A320TV_NTSC 1
#define FB_A320TV_PAL50 2
#define FB_A320TV_PAL60 3
#define FB_A320TV_PAL_M 4
#define FB_A320TV_LAST 4

static const char *jzfb_tv_out_norm[] = {
	"off", "ntsc", "pal", "pal-60", "pal-m",
};

static struct fb_fix_screeninfo jzfb_fix = {
	.id =		"JZ4740 SLCD FB",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	1,
	.ywrapstep =	0,
	.accel =	FB_ACCEL_NONE,
};

const static struct jz_gpio_bulk_request jz_slcd_ctrl_pins[] = {
	JZ_GPIO_BULK_PIN(LCD_PCLK),
	JZ_GPIO_BULK_PIN(SLCD_RS),
	JZ_GPIO_BULK_PIN(SLCD_CS),
};

const static struct jz_gpio_bulk_request jz_slcd_data_pins[] = {
	JZ_GPIO_BULK_PIN(LCD_DATA0),
	JZ_GPIO_BULK_PIN(LCD_DATA1),
	JZ_GPIO_BULK_PIN(LCD_DATA2),
	JZ_GPIO_BULK_PIN(LCD_DATA3),
	JZ_GPIO_BULK_PIN(LCD_DATA4),
	JZ_GPIO_BULK_PIN(LCD_DATA5),
	JZ_GPIO_BULK_PIN(LCD_DATA6),
	JZ_GPIO_BULK_PIN(LCD_DATA7),
	JZ_GPIO_BULK_PIN(LCD_DATA8),
	JZ_GPIO_BULK_PIN(LCD_DATA9),
	JZ_GPIO_BULK_PIN(LCD_DATA10),
	JZ_GPIO_BULK_PIN(LCD_DATA11),
	JZ_GPIO_BULK_PIN(LCD_DATA12),
	JZ_GPIO_BULK_PIN(LCD_DATA13),
	JZ_GPIO_BULK_PIN(LCD_DATA14),
	JZ_GPIO_BULK_PIN(LCD_DATA15),
	JZ_GPIO_BULK_PIN(LCD_DATA16),
	JZ_GPIO_BULK_PIN(LCD_DATA17),
};

static unsigned int jzfb_num_ctrl_pins(struct jzfb *jzfb)
{
	return ARRAY_SIZE(jz_slcd_ctrl_pins);
}

static unsigned int jzfb_num_data_pins(struct jzfb *jzfb)
{
	switch (jzfb->pdata->lcd_type) {
	case JZ_LCD_TYPE_SMART_PARALLEL_8_BIT:
		return 8;
	case JZ_LCD_TYPE_SMART_PARALLEL_16_BIT:
		return 16;
	case JZ_LCD_TYPE_SMART_PARALLEL_18_BIT:
		return 18;
	default:
		return 0;
	}
}

static void jzfb_free_gpio_pins(struct jzfb *jzfb)
{
	jz_gpio_bulk_free(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_free(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_free(jz_slcd_data_pins,
				  jzfb_num_data_pins(jzfb));
	}
}

static int jzfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			unsigned blue, unsigned transp, struct fb_info *fb)
{
	if (regno >= 16)
		return -EINVAL;

	red   = (red   * ((1 << fb->var.red.length  ) - 1)) / ((1 << 16) - 1);
	green = (green * ((1 << fb->var.green.length) - 1)) / ((1 << 16) - 1);
	blue  = (blue  * ((1 << fb->var.blue.length ) - 1)) / ((1 << 16) - 1);

	((uint32_t *)fb->pseudo_palette)[regno] =
		(red   << fb->var.red.offset  ) |
		(green << fb->var.green.offset) |
		(blue  << fb->var.blue.offset );

	return 0;
}

static int jzfb_get_controller_bpp(struct jzfb *jzfb)
{
	switch (jzfb->pdata->bpp) {
	case 18:
	case 24:
		return 32;
	case 15:
		return 16;
	default:
		return jzfb->pdata->bpp;
	}
}

static struct fb_videomode *jzfb_get_mode(struct jzfb *jzfb, struct fb_var_screeninfo *var)
{
	size_t i;
	struct fb_videomode *mode = jzfb->pdata->modes;

	for (i = 0; i < jzfb->pdata->num_modes; ++i, ++mode) {
		if (mode->xres == var->xres && mode->yres == var->yres)
			return mode;
	}

	return NULL;
}

static int jzfb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	struct fb_videomode *mode;

	if (var->bits_per_pixel != jzfb_get_controller_bpp(jzfb) &&
		var->bits_per_pixel != jzfb->pdata->bpp)
		return -EINVAL;

	mode = jzfb_get_mode(jzfb, var);
	if (mode == NULL)
		return -EINVAL;

	fb_videomode_to_var(var, mode);

	/* Reserve space for double buffering. */
	var->yres_virtual = var->yres * 2;

	switch (jzfb->pdata->bpp) {
	case 8:
		break;
	case 15:
		var->red.offset = 10;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 5;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;
	case 16:
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		break;
	case 18:
		var->red.offset = 16;
		var->red.length = 6;
		var->green.offset = 8;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 6;
		var->bits_per_pixel = 32;
		break;
	case 32:
	case 24:
		var->transp.offset = 24;
		var->transp.length = 8;
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->bits_per_pixel = 32;
		break;
	default:
		break;
	}

	return 0;
}

static void jzfb_refresh_work_complete(void *param)
{
	struct jzfb *jzfb = param;
	complete_all(&jzfb->dma_completion);
}

static void jzfb_disable_dma(struct jzfb *jzfb)
{
	dmaengine_terminate_all(jzfb->dma);
	while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
	writeb(readb(jzfb->base + JZ_REG_SLCD_CTRL) & ~SLCD_CTRL_DMA_EN,
		jzfb->base + JZ_REG_SLCD_CTRL);
}

static void jzfb_upload_frame_dma(struct jzfb *jzfb)
{
	struct fb_info *fb = jzfb->fb;
	struct fb_videomode *mode = fb->mode;
	__u32 offset = fb->fix.line_length * fb->var.yoffset;
	__u32 size = fb->fix.line_length * mode->yres;
	struct dma_async_tx_descriptor *desc;

	/* Ensure that the data to be uploaded is in memory. */
	dma_cache_sync(fb->device, jzfb->vidmem + offset, size,
		       DMA_TO_DEVICE);

	desc = dmaengine_prep_slave_single(jzfb->dma, DMA_MEM_TO_DEV,
		jzfb->vidmem_phys + offset, size, DMA_PREP_INTERRUPT);
	if (!desc)
		return;

	desc->callback = jzfb_refresh_work_complete;
	desc->callback_param = jzfb;
	dmaengine_submit(desc);

	while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
	writeb(readb(jzfb->base + JZ_REG_SLCD_CTRL) | SLCD_CTRL_DMA_EN,
		jzfb->base + JZ_REG_SLCD_CTRL);
	dma_async_issue_pending(jzfb->dma);
}

static void jzfb_upload_frame_cpu(struct jzfb *jzfb)
{
	const int num_pixels = jzfb->fb->mode->xres * jzfb->fb->mode->yres;
	uint16_t *p = jzfb->vidmem;
	int i;

	jzfb_disable_dma(jzfb);
	for (i = 0; i < num_pixels; i++) {
		uint16_t rgb = *p++;
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | rgb, jzfb->base + JZ_REG_SLCD_DATA);
	}
}

static void jzfb_refresh_work(struct work_struct *work)
{
	struct jzfb *jzfb = container_of(work, struct jzfb, refresh_work.work);

	mutex_lock(&jzfb->lock);
	if (jzfb->is_enabled) {
		if (1) {
			int interval;

			if (jzfb->dma_completion.done) {
				if (jzfb->refresh_on_pan)
					interval = HZ / 5;
				else
					interval = HZ / 60;
				jzfb->refresh_on_pan = 0;

				INIT_COMPLETION(jzfb->dma_completion);
				jzfb_upload_frame_dma(jzfb);
			} else
				interval = HZ / 250;

			schedule_delayed_work(&jzfb->refresh_work, interval);
		} else {
			jzfb_upload_frame_cpu(jzfb);
			schedule_delayed_work(&jzfb->refresh_work, HZ / 10);
		}
	}
	mutex_unlock(&jzfb->lock);
}

static int jzfb_set_par(struct fb_info *info)
{
	struct jzfb *jzfb = info->par;
	struct fb_var_screeninfo *var = &info->var;
	struct fb_videomode *mode;
	uint16_t slcd_cfg;

	mode = jzfb_get_mode(jzfb, var);
	if (mode == NULL)
		return -EINVAL;

	info->mode = mode;

	slcd_cfg = SLCD_CFG_BURST_8_WORD;
	/* command size */
	slcd_cfg |= (jzfb->pdata->lcd_type & 3) << SLCD_CFG_CWIDTH_BIT;
	/* data size */
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		unsigned int num_bits;
		switch (jzfb->pdata->lcd_type) {
		case JZ_LCD_TYPE_SMART_SERIAL_8_BIT:
			slcd_cfg |= SLCD_CFG_DWIDTH_8_x1;
			num_bits = 8;
			break;
		case JZ_LCD_TYPE_SMART_SERIAL_16_BIT:
			slcd_cfg |= SLCD_CFG_DWIDTH_16;
			num_bits = 16;
			break;
		case JZ_LCD_TYPE_SMART_SERIAL_18_BIT:
			slcd_cfg |= SLCD_CFG_DWIDTH_18;
			num_bits = 18;
			break;
		default:
			num_bits = 0;
			break;
		}
		if (num_bits != jzfb->pdata->bpp) {
			dev_err(&jzfb->pdev->dev,
				"Data size (%d) does not match bpp (%d)\n",
				num_bits, jzfb->pdata->bpp);
		}
		slcd_cfg |= SLCD_CFG_TYPE_SERIAL;
	} else {
		/* parallel */
		switch (jzfb->pdata->bpp) {
		case 8:
			slcd_cfg |= SLCD_CFG_DWIDTH_8_x1;
			break;
		case 15:
		case 16:
			switch (jzfb->pdata->lcd_type) {
			case JZ_LCD_TYPE_SMART_PARALLEL_8_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_8_x2;
				break;
			default:
				slcd_cfg |= SLCD_CFG_DWIDTH_16;
				break;
			}
			break;
		case 18:
			switch (jzfb->pdata->lcd_type) {
			case JZ_LCD_TYPE_SMART_PARALLEL_8_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_8_x3;
				break;
			case JZ_LCD_TYPE_SMART_PARALLEL_16_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_9_x2;
				break;
			case JZ_LCD_TYPE_SMART_PARALLEL_18_BIT:
				slcd_cfg |= SLCD_CFG_DWIDTH_18;
				break;
			default:
				break;
			}
			break;
		case 24:
			slcd_cfg |= SLCD_CFG_DWIDTH_8_x3;
			break;
		default:
			dev_err(&jzfb->pdev->dev,
				"Unsupported value for bpp: %d\n",
				jzfb->pdata->bpp);
		}
		slcd_cfg |= SLCD_CFG_TYPE_PARALLEL;
	}
	if (!jzfb->pdata->chip_select_active_low)
		slcd_cfg |= SLCD_CFG_CS_ACTIVE_HIGH;
	if (!jzfb->pdata->register_select_active_low)
		slcd_cfg |= SLCD_CFG_RS_CMD_HIGH;
	if (!jzfb->pdata->pixclk_falling_edge)
		slcd_cfg |= SLCD_CFG_CLK_ACTIVE_RISING;

#if 0
	// TODO(MtH): Compute rate from refresh or vice versa.
	if (mode->pixclock) {
		rate = PICOS2KHZ(mode->pixclock) * 1000;
		mode->refresh = rate / vt / ht;
	} else {
		if (jzfb->pdata->lcd_type == JZ_LCD_TYPE_8BIT_SERIAL)
			rate = mode->refresh * (vt + 2 * mode->xres) * ht;
		else
			rate = mode->refresh * vt * ht;

		mode->pixclock = KHZ2PICOS(rate / 1000);
	}
#endif

	mutex_lock(&jzfb->lock);
	if (!jzfb->is_enabled)
		clk_prepare_enable(jzfb->ldclk);

	// TODO(MtH): We should not change config while DMA might be running.
	writew(slcd_cfg, jzfb->base + JZ_REG_SLCD_CFG);

	if (!jzfb->is_enabled)
		clk_disable_unprepare(jzfb->ldclk);
	mutex_unlock(&jzfb->lock);

	// TODO(MtH): Use maximum transfer speed that panel can handle.
	//            ILI9325 can do 10 MHz.
	clk_set_rate(jzfb->lpclk, 12000000);
	clk_set_rate(jzfb->ldclk, 42000000);

	return 0;
}

static void jzfb_enable(struct jzfb *jzfb)
{
	uint32_t ctrl;

	clk_prepare_enable(jzfb->ldclk);

	jz_gpio_bulk_resume(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_resume(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_resume(jz_slcd_data_pins,
				    jzfb_num_data_pins(jzfb));
	}
	jzfb_disable_dma(jzfb);
	complete_all(&jzfb->dma_completion);
	jzfb->panel->enable(jzfb);

	ctrl = readl(jzfb->base + JZ_REG_LCD_CTRL);
	ctrl |= JZ_LCD_CTRL_ENABLE;
	ctrl &= ~JZ_LCD_CTRL_DISABLE;
	writel(ctrl, jzfb->base + JZ_REG_LCD_CTRL);

	schedule_delayed_work(&jzfb->refresh_work, 0);
}

static void jzfb_disable(struct jzfb *jzfb)
{
	/* It is safe but wasteful to call refresh_work() while disabled. */
	cancel_delayed_work_sync(&jzfb->refresh_work);

	/* Abort any DMA transfer that might be in progress and allow direct
	   writes to the panel. */
	jzfb_disable_dma(jzfb);
	complete_all(&jzfb->dma_completion);

	jzfb->panel->disable(jzfb);
	jz_gpio_bulk_suspend(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_suspend(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_suspend(jz_slcd_data_pins,
				     jzfb_num_data_pins(jzfb));
	}

	clk_disable_unprepare(jzfb->ldclk);
}

static int jzfb_blank(int blank_mode, struct fb_info *info)
{
	struct jzfb *jzfb = info->par;
	int ret = 0;
	int new_enabled = (blank_mode == FB_BLANK_UNBLANK);

	mutex_lock(&jzfb->lock);
	if (new_enabled) {
		if (!jzfb->is_enabled)
			jzfb_enable(jzfb);
	} else {
		if (jzfb->is_enabled) {
			/* No sleep in TV-out mode. */
			if (jzfb->tv_out == FB_A320TV_OFF)
				jzfb_disable(jzfb);
			else
				ret = -EBUSY;
		}
	}
	if (!ret)
		jzfb->is_enabled = new_enabled;
	mutex_unlock(&jzfb->lock);

	return ret;
}

static int jzfb_wait_for_vsync(struct fb_info *info)
{
	struct jzfb *jzfb = info->par;

	if (jzfb->tv_out != FB_A320TV_OFF &&
				!jzfb->tv_out_vsync)
		return 0;
	return wait_for_completion_interruptible(&jzfb->dma_completion);
}

static int jzfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct jzfb *jzfb = info->par;
	info->var.yoffset = var->yoffset;

	/* Ensure that the data to be uploaded is in memory. */
	dma_cache_sync(&jzfb->pdev->dev, jzfb->vidmem
				+ info->fix.line_length * var->yoffset,
				info->fix.line_length * var->yres,
				DMA_TO_DEVICE);

	/* update frame start address for TV-out mode */
	(*jzfb->framedesc)[1].addr = jzfb->vidmem_phys
	                      + info->fix.line_length * var->yoffset;

	jzfb_wait_for_vsync(info);

	jzfb->refresh_on_pan = 1;
	flush_delayed_work(&jzfb->refresh_work);
	return 0;
}

static int jzfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	const unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	const unsigned long size = vma->vm_end - vma->vm_start;

	if (offset + size > info->fix.smem_len)
		return -EINVAL;

	if (remap_pfn_range(vma, vma->vm_start,
			    (info->fix.smem_start + offset) >> PAGE_SHIFT,
			    size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int jzfb_alloc_devmem(struct jzfb *jzfb)
{
	int max_linesize = 0, max_framesize = 0;
	int bytes_per_pixel;
	struct fb_videomode *mode = jzfb->pdata->modes;
	void *page;
	int i;

	for (i = 0; i < jzfb->pdata->num_modes; ++mode, ++i) {
		if (max_linesize < mode->xres)
			max_linesize = mode->xres;
		if (max_framesize < mode->xres * mode->yres)
			max_framesize = mode->xres * mode->yres;
	}

	bytes_per_pixel = jzfb_get_controller_bpp(jzfb) >> 3;
	max_linesize *= bytes_per_pixel;
	max_framesize *= bytes_per_pixel;

	jzfb->framedesc = dma_alloc_coherent(&jzfb->pdev->dev,
				    sizeof(*jzfb->framedesc),
				    &jzfb->framedesc_phys, GFP_KERNEL);
	if (!jzfb->framedesc)
		return -ENOMEM;

	jzfb->blackline_size = max_linesize;
	jzfb->blackline = dma_alloc_coherent(&jzfb->pdev->dev,
					     jzfb->blackline_size,
					     &jzfb->blackline_phys, GFP_KERNEL);
	if (!jzfb->blackline)
		goto err_free_framedesc;

	/* Set the black line to black... */
	memset(jzfb->blackline, 0, jzfb->blackline_size);

	/* reserve memory for two frames to allow double buffering */
	jzfb->vidmem_size = PAGE_ALIGN(max_framesize * 2);
	jzfb->vidmem = dma_alloc_coherent(&jzfb->pdev->dev,
						jzfb->vidmem_size,
						&jzfb->vidmem_phys, GFP_KERNEL);

	if (!jzfb->vidmem)
		goto err_free_blackline;

	for (page = jzfb->vidmem;
		 page < jzfb->vidmem + PAGE_ALIGN(jzfb->vidmem_size);
		 page += PAGE_SIZE) {
		SetPageReserved(virt_to_page(page));
	}

	for (i = 0; i < 3; i++)
		(*jzfb->framedesc)[i].next = jzfb->framedesc_phys
				+ ((i + 1) % 3) * sizeof(struct jzfb_framedesc);
	(*jzfb->framedesc)[0].addr = (*jzfb->framedesc)[2].addr =
			jzfb->blackline_phys;
	(*jzfb->framedesc)[0].id = 0xdadabeeb;
	(*jzfb->framedesc)[2].id = 0xfadefeed;
	(*jzfb->framedesc)[0].cmd = (*jzfb->framedesc)[2].cmd =
			jzfb->blackline_size / 4;
	(*jzfb->framedesc)[1].addr = jzfb->vidmem_phys;
	(*jzfb->framedesc)[1].id = 0xdeafbead;
	(*jzfb->framedesc)[1].cmd = (max_framesize / 4)
			| JZ_LCD_CMD_EOF_IRQ | JZ_LCD_CMD_SOF_IRQ;

	return 0;

err_free_blackline:
	dma_free_coherent(&jzfb->pdev->dev, jzfb->blackline_size,
				jzfb->blackline, jzfb->blackline_phys);
err_free_framedesc:
	dma_free_coherent(&jzfb->pdev->dev, sizeof(*jzfb->framedesc),
				jzfb->framedesc, jzfb->framedesc_phys);
	return -ENOMEM;
}

static void jzfb_free_devmem(struct jzfb *jzfb)
{
	dma_free_coherent(&jzfb->pdev->dev, jzfb->vidmem_size,
				jzfb->vidmem, jzfb->vidmem_phys);
	dma_free_coherent(&jzfb->pdev->dev, jzfb->blackline_size,
				jzfb->blackline, jzfb->blackline_phys);
	dma_free_coherent(&jzfb->pdev->dev, sizeof(*jzfb->framedesc),
				jzfb->framedesc, jzfb->framedesc_phys);
}

static int jzfb_tv_out(struct jzfb *jzfb, unsigned int mode)
{
	int blank = jzfb->is_enabled ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
	struct fb_event event = {
		.info = jzfb->fb,
		.data = &blank,
	};

	printk("A320 TV out: %d\n", mode);
	if (mode > FB_A320TV_LAST)
		return -EINVAL;
	if (mode == jzfb->tv_out)
		return 0;

	if (mode != FB_A320TV_OFF) {
		cancel_delayed_work_sync(&jzfb->refresh_work);
		/* Abort any DMA transfer that might be in progress and
		   allow direct writes to the panel.  */
		jzfb_disable_dma(jzfb);
		jzfb->panel->disable(jzfb);
		complete_all(&jzfb->dma_completion);

		/* set up LCD controller for TV output */

		writel(JZ_LCD_CFG_HSYNC_ACTIVE_LOW |
		       JZ_LCD_CFG_VSYNC_ACTIVE_LOW,
		       jzfb->base + JZ_REG_LCD_CFG);

		/* V-Sync pulse end position */
		writel(10, jzfb->base + JZ_REG_LCD_VSYNC);

		if (mode == FB_A320TV_PAL50) {
			/* PAL 50 Hz */
			/* H-Sync pulse start position */
			writel(0x0000007d, jzfb->base + JZ_REG_LCD_HSYNC);
			/* virtual area size */
			writel(0x036c0112, jzfb->base + JZ_REG_LCD_VAT);
			/* horizontal start/end point */
			writel(0x02240364, jzfb->base + JZ_REG_LCD_DAH);
			/* vertical start/end point */
			writel(0x001a010c, jzfb->base + JZ_REG_LCD_DAV);
		} else {
			/* NTSC and PAL 60 Hz */
			writel(0x0000003c, jzfb->base + JZ_REG_LCD_HSYNC);
			writel(0x02e00110, jzfb->base + JZ_REG_LCD_VAT);
			writel(0x019902d9, jzfb->base + JZ_REG_LCD_DAH);
			writel(0x001c010e, jzfb->base + JZ_REG_LCD_DAV);
		}
		writel(0, jzfb->base + JZ_REG_LCD_PS);
		writel(0, jzfb->base + JZ_REG_LCD_CLS);
		writel(0, jzfb->base + JZ_REG_LCD_SPL);
		writel(0, jzfb->base + JZ_REG_LCD_REV);
		/* reset status register */
		writel(0, jzfb->base + JZ_REG_LCD_STATE);

		/* tell LCDC about the frame descriptor address */
		writel(jzfb->framedesc_phys, jzfb->base + JZ_REG_LCD_DA0);

		writel(JZ_LCD_CTRL_BURST_16 | JZ_LCD_CTRL_ENABLE |
					JZ_LCD_CTRL_BPP_15_16 |
					JZ_LCD_CTRL_EOF_IRQ | JZ_LCD_CTRL_SOF_IRQ,
					jzfb->base + JZ_REG_LCD_CTRL);
	} else {
		/* disable EOF/SOF interrupts */
		unsigned long ctrl = readl(jzfb->base + JZ_REG_LCD_CTRL);
		ctrl &= ~(JZ_LCD_CTRL_EOF_IRQ | JZ_LCD_CTRL_SOF_IRQ);
		writel(ctrl, jzfb->base + JZ_REG_LCD_CTRL);

		/* disable LCD controller and re-enable SLCD */
		writel(JZ_LCD_CFG_SLCD, jzfb->base + JZ_REG_LCD_CFG);
		jzfb->panel->enable(jzfb);

		jzfb->refresh_on_pan = 0;
		complete_all(&jzfb->dma_completion);
		schedule_delayed_work(&jzfb->refresh_work, 0);
	}

	/* reaffirm the current blanking state, to trigger a backlight update */
	console_lock();
	fb_notifier_call_chain(FB_EVENT_BLANK, &event);
	console_unlock();
	jzfb->tv_out = mode;
	return 0;
}

static ssize_t jzfb_tv_out_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	if (jzfb->tv_out > FB_A320TV_LAST) {
		dev_err(dev, "Unknown norm for TV-out\n");
		return -1;
	}

	return sprintf(buf, "%s\n", jzfb_tv_out_norm[jzfb->tv_out]);
}

static ssize_t jzfb_tv_out_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	size_t i;
	struct jzfb *jzfb = dev_get_drvdata(dev);

	for (i = 0; i <= FB_A320TV_LAST; i++) {
		if (sysfs_streq(jzfb_tv_out_norm[i], buf)) {
			jzfb_tv_out(jzfb, i);
			return n;
		}
	}
	return -EINVAL;
}

static DEVICE_ATTR(tv_out, 0644, jzfb_tv_out_show, jzfb_tv_out_store);

static ssize_t jzfb_vsync_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", jzfb->tv_out_vsync);
}

static ssize_t jzfb_vsync_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	unsigned int vsync;

	if (sscanf(buf, "%u", &vsync) < 1)
		return -EINVAL;

	jzfb->tv_out_vsync = vsync;
	return n;
}

static DEVICE_ATTR(tv_out_vsync, 0644, jzfb_vsync_show, jzfb_vsync_store);

static ssize_t jzfb_panel_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", jzfb->panel->name);
}

static ssize_t jzfb_panel_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	const struct jz_slcd_panel *panel = jz_slcd_panel_from_name(buf);

	if (!panel) {
		dev_err(dev, "Unknown SLCD panel: %s\n", buf);
		return -EINVAL;
	}

	if (panel != jzfb->panel) {
		jzfb->panel->disable(jzfb);
		jzfb->panel->exit(jzfb);
		jzfb->panel = panel;
		panel->init(jzfb);
		panel->enable(jzfb);
	}
	return n;
}

static DEVICE_ATTR(panel, 0644, jzfb_panel_show, jzfb_panel_store);

static struct fb_ops jzfb_ops = {
	.owner			= THIS_MODULE,
	.fb_check_var 		= jzfb_check_var,
	.fb_set_par 		= jzfb_set_par,
	.fb_setcolreg		= jzfb_setcolreg,
	.fb_blank		= jzfb_blank,
	.fb_pan_display		= jzfb_pan_display,
	.fb_fillrect		= sys_fillrect,
	.fb_copyarea		= sys_copyarea,
	.fb_imageblit		= sys_imageblit,
	.fb_mmap		= jzfb_mmap,
};

static irqreturn_t jz4740_lcd_irq(int irq, void *dev_id)
{
	struct jzfb *jzfb = dev_id;
	unsigned long state = readl(jzfb->base + JZ_REG_LCD_STATE);

	if (state & JZ_LCD_STATE_SOF) {
		INIT_COMPLETION(jzfb->dma_completion);
		state &= ~JZ_LCD_STATE_SOF;
	} else {
		complete_all(&jzfb->dma_completion);
		state &= ~JZ_LCD_STATE_EOF;
	}

	/* Acknowledge the interrupt */
	writel(state, jzfb->base + JZ_REG_LCD_STATE);
	return IRQ_HANDLED;
}

static int jzfb_probe(struct platform_device *pdev)
{
	int ret;
	struct jzfb *jzfb;
	struct fb_info *fb;
	struct jz4740_fb_platform_data *pdata = pdev->dev.platform_data;
	struct dma_slave_config config;
	struct resource *mem;
	dma_cap_mask_t dma_mask;

	if (!pdata) {
		dev_err(&pdev->dev, "Missing platform data\n");
		return -ENOENT;
	}

	fb = framebuffer_alloc(sizeof(struct jzfb), &pdev->dev);

	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		return -ENOMEM;
	}

	fb->fbops = &jzfb_ops;
	fb->flags = FBINFO_DEFAULT;

	jzfb = fb->par;
	jzfb->pdata = pdata;
	jzfb->pdev = pdev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	jzfb->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(jzfb->base)) {
		ret = PTR_ERR(jzfb->base);
		goto err_framebuffer_release;
	}

	jzfb->tv_out = FB_A320TV_OFF;
	jzfb->tv_out_vsync = 1;
	jzfb->refresh_on_pan = 0;
	init_completion(&jzfb->dma_completion);
	complete_all(&jzfb->dma_completion);

	dma_cap_zero(dma_mask);
	dma_cap_set(DMA_SLAVE, dma_mask);

	jzfb->dma = dma_request_channel(dma_mask, NULL, NULL);
	if (!jzfb->dma) {
		dev_err(&pdev->dev, "Failed to get DMA channel\n");
		ret = -EBUSY;
		goto err_framebuffer_release;
	}

	config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	config.dst_maxburst = 16;
	config.slave_id = JZ4740_DMA_TYPE_SLCD;
	config.dst_addr = mem->start + JZ_REG_SLCD_FIFO;

	dmaengine_slave_config(jzfb->dma, &config);

	jzfb->ldclk = clk_get(&pdev->dev, "lcd");
	if (IS_ERR(jzfb->ldclk)) {
		ret = PTR_ERR(jzfb->ldclk);
		dev_err(&pdev->dev, "Failed to get lcd clock: %d\n", ret);
		goto err_free_dma;
	}

	jzfb->lpclk = clk_get(&pdev->dev, "lcd_pclk");
	if (IS_ERR(jzfb->lpclk)) {
		ret = PTR_ERR(jzfb->lpclk);
		dev_err(&pdev->dev, "Failed to get lcd pixel clock: %d\n", ret);
		goto err_put_ldclk;
	}

	platform_set_drvdata(pdev, jzfb);

	fb_videomode_to_modelist(pdata->modes, pdata->num_modes,
				 &fb->modelist);
	fb->mode = pdata->modes;

	fb_videomode_to_var(&fb->var, fb->mode);
	fb->var.bits_per_pixel = pdata->bpp;
	jzfb_check_var(&fb->var, fb);

	ret = jzfb_alloc_devmem(jzfb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to allocate video memory\n");
		goto err_put_lpclk;
	}

	fb->fix = jzfb_fix;
	fb->fix.line_length = fb->var.bits_per_pixel * fb->var.xres / 8;
	fb->fix.mmio_start = mem->start;
	fb->fix.mmio_len = resource_size(mem);
	fb->fix.smem_start = jzfb->vidmem_phys;
	fb->fix.smem_len =  fb->fix.line_length * fb->var.yres_virtual;
	fb->screen_base = jzfb->vidmem;
	fb->pseudo_palette = jzfb->pseudo_palette;

	fb_alloc_cmap(&fb->cmap, 256, 0);

	mutex_init(&jzfb->lock);

	clk_prepare_enable(jzfb->ldclk);
	jzfb->is_enabled = 1;

	writel(JZ_LCD_CFG_SLCD, jzfb->base + JZ_REG_LCD_CFG);
	writeb(0, jzfb->base + JZ_REG_SLCD_CTRL);

	jzfb_set_par(fb);

	jz_gpio_bulk_request(jz_slcd_ctrl_pins, jzfb_num_ctrl_pins(jzfb));
	if (jzfb->pdata->lcd_type & (1 << 6)) {
		/* serial */
		jz_gpio_bulk_request(&jz_slcd_data_pins[15], 1);
	} else {
		/* parallel */
		jz_gpio_bulk_request(jz_slcd_data_pins,
				     jzfb_num_data_pins(jzfb));
	}

	ret = request_irq(JZ4740_IRQ_LCD, jz4740_lcd_irq, 0, "LCD", jzfb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request IRQ\n");
		goto err_free_devmem;
	}

	jzfb->panel = jz_slcd_panels_probe(jzfb);
	if (!jzfb->panel) {
		dev_err(&pdev->dev, "Failed to find panel driver\n");
		ret = -ENOENT;
		goto err_free_irq;
	}
	jzfb_disable_dma(jzfb);
	jzfb->panel->init(jzfb);

	jzfb->panel->enable(jzfb);

	ret = register_framebuffer(fb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register framebuffer: %d\n", ret);
		goto err_free_panel;
	}

	jzfb->fb = fb;
	fb_prepare_logo(jzfb->fb, 0);
	fb_show_logo(jzfb->fb, 0);

	INIT_DELAYED_WORK(&jzfb->refresh_work, jzfb_refresh_work);
	schedule_delayed_work(&jzfb->refresh_work, 0);

	ret = device_create_file(&pdev->dev, &dev_attr_panel);
	if (ret)
		goto err_cancel_work;

	ret = device_create_file(&pdev->dev, &dev_attr_tv_out);
	if (ret)
		goto err_remove_file_panel;

	ret = device_create_file(&pdev->dev, &dev_attr_tv_out_vsync);
	if (!ret)
		return 0;

	device_remove_file(&pdev->dev, &dev_attr_tv_out);
err_remove_file_panel:
	device_remove_file(&pdev->dev, &dev_attr_panel);
err_cancel_work:
	cancel_delayed_work_sync(&jzfb->refresh_work);
err_free_panel:
	jzfb->panel->exit(jzfb);
err_free_irq:
	free_irq(JZ4740_IRQ_LCD, jzfb);
err_free_devmem:
	jzfb_free_gpio_pins(jzfb);

	fb_dealloc_cmap(&fb->cmap);
	jzfb_free_devmem(jzfb);
err_put_lpclk:
	clk_put(jzfb->lpclk);
err_put_ldclk:
	clk_put(jzfb->ldclk);
err_free_dma:
	dma_release_channel(jzfb->dma);
err_framebuffer_release:
	framebuffer_release(fb);
	return ret;
}

static int jzfb_remove(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_tv_out_vsync);
	device_remove_file(&pdev->dev, &dev_attr_tv_out);
	device_remove_file(&pdev->dev, &dev_attr_panel);
	jzfb_blank(FB_BLANK_POWERDOWN, jzfb->fb);

	free_irq(JZ4740_IRQ_LCD, jzfb);

	/* Blanking will prevent future refreshes from behind scheduled.
	   Now wait for a possible refresh in progress to finish. */
	cancel_delayed_work_sync(&jzfb->refresh_work);

	jzfb->panel->exit(jzfb);

	jzfb_free_gpio_pins(jzfb);

	dma_release_channel(jzfb->dma);

	fb_dealloc_cmap(&jzfb->fb->cmap);
	jzfb_free_devmem(jzfb);

	platform_set_drvdata(pdev, NULL);

	clk_put(jzfb->lpclk);
	clk_put(jzfb->ldclk);

	framebuffer_release(jzfb->fb);

	return 0;
}

#ifdef CONFIG_PM

static int jzfb_suspend(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	console_lock();
	fb_set_suspend(jzfb->fb, 1);
	console_unlock();

	mutex_lock(&jzfb->lock);
	if (jzfb->is_enabled)
		jzfb_disable(jzfb);
	mutex_unlock(&jzfb->lock);

	return 0;
}

static int jzfb_resume(struct device *dev)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	mutex_lock(&jzfb->lock);
	if (jzfb->is_enabled)
		jzfb_enable(jzfb);
	mutex_unlock(&jzfb->lock);

	console_lock();
	fb_set_suspend(jzfb->fb, 0);
	console_unlock();

	return 0;
}

static SIMPLE_DEV_PM_OPS(jzfb_pm_ops, jzfb_suspend, jzfb_resume);

#define JZFB_PM_OPS (&jzfb_pm_ops)

#else
#define JZFB_PM_OPS NULL
#endif

static struct platform_driver jzfb_driver = {
	.probe		= jzfb_probe,
	.remove		= jzfb_remove,
	.driver = {
		.name	= "jz4740-fb",
		.pm	= JZFB_PM_OPS,
	},
};
module_platform_driver(jzfb_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>, Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("JZ4740 SoC SLCD framebuffer driver");
MODULE_ALIAS("platform:jz4740-fb");

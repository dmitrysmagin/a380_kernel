/*
 * jz4750_lcd.c -- Ingenic Jz4750 LCD frame buffer device
 *
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2014, Dmitry Smagin <dmitry.s.smagin@gmail.com>
 *
 * Based on the JZ4750 frame buffer driver:
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 * Author: Wolfgang Wang, <lgwang@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/clk.h>
#include <linux/kthread.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/processor.h>

#include <asm/mach-jz4750d/jz4750d_lcdc.h>
#include <asm/mach-jz4750d/jz4750d_tve.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>
#include <asm/mach-jz4750d/irq.h>

#include <video/jzpanel.h>

struct jzfb_platform_data {
	struct panel_ops *panel_ops;
	void *panel_pdata;
};

/* TVE panel */
#include "jz4750_tve.h"

/* choose LCD panel */
#include "jz_rzx50_panel.h"
#include "jz_a380_panel.h"
#include "jz_a320e_panel.h"

/* use new descriptor(8 words) */
struct jz4750_lcd_dma_desc {
	unsigned int next_desc; 	/* LCDDAx */
	unsigned int databuf;   	/* LCDSAx */
	unsigned int frame_id;  	/* LCDFIDx */
	unsigned int cmd; 		/* LCDCMDx */
	unsigned int offsize;       	/* Stride Offsize(in word) */
	unsigned int page_width; 	/* Stride Pagewidth(in word) */
	unsigned int cmd_num; 		/* Command Number(for SLCD) */
	unsigned int desc_size; 	/* Foreground Size */
};

struct jz4750lcd_panel_t {
	unsigned int cfg;	/* panel mode and pin usage etc. */
	unsigned int slcd_cfg;	/* Smart lcd configurations */
	unsigned int ctrl;	/* lcd controll register */
	unsigned int w;		/* Panel Width(in pixel) */
	unsigned int h;		/* Panel Height(in line) */
	unsigned int fclk;	/* frame clk */
	unsigned int hsw;	/* hsync width, in pclk */
	unsigned int vsw;	/* vsync width, in line count */
	unsigned int elw;	/* end of line, in pclk */
	unsigned int blw;	/* begin of line, in pclk */
	unsigned int efw;	/* end of frame, in line count */
	unsigned int bfw;	/* begin of frame, in line count */
};

#define FG_NOCHANGE 		0x0000
#define FG0_CHANGE_SIZE 	0x0001
#define FG0_CHANGE_POSITION 	0x0002
#define FG1_CHANGE_SIZE 	0x0010
#define FG1_CHANGE_POSITION 	0x0020
#define FG_CHANGE_ALL 		( FG0_CHANGE_SIZE | FG0_CHANGE_POSITION | \
				  FG1_CHANGE_SIZE | FG1_CHANGE_POSITION )

//#define JZ_FB_DEBUG

#ifdef JZ_FB_DEBUG
  #define D(fmt, args ...) \
	printk(KERN_ERR "%s(): "fmt"\n", __func__, ##args)
#else
  #define D(fmt, args ...)
#endif

/*
 * TODO: take information on w, h and bpp from panel part, not osd.
 */

struct jz4750lcd_panel_t jz4750_lcd_panel = {
#if defined(CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL)
	.cfg =	LCD_CFG_LCDPIN_LCD |
		LCD_CFG_RECOVER | /* Underrun recover */
		LCD_CFG_NEWDES | /* 8words descriptor */
		LCD_CFG_MODE_GENERIC_TFT | /* Generic TFT panel */
		LCD_CFG_MODE_TFT_24BIT | /* output 24bpp */
		LCD_CFG_HSP | /* Hsync polarity: active low */
		LCD_CFG_VSP,
		/* Vsync polarity: leading edge is falling edge */
	.slcd_cfg = 0,
	.ctrl =		LCD_CTRL_OFUM |
			LCD_CTRL_BST_16,
		/* 16words burst, enable out FIFO underrun irq */
	480, 272, 40, 1, 1, 40, 215, 0, 45,
#elif defined(CONFIG_JZ4750_SLCD_A380_ILI9331)
	.cfg =		LCD_CFG_LCDPIN_SLCD | /* Underrun recover*/
		      LCD_CFG_NEWDES | /* 8words descriptor */
		      LCD_CFG_MODE_SLCD, /* TFT Smart LCD panel */
	.slcd_cfg =	SLCD_CFG_DWIDTH_8BIT_x2 |
		      SLCD_CFG_CWIDTH_8BIT |
		      SLCD_CFG_CS_ACTIVE_LOW |
		      SLCD_CFG_RS_CMD_LOW |
		      SLCD_CFG_CLK_ACTIVE_FALLING |
		      SLCD_CFG_TYPE_PARALLEL,
	.ctrl =	LCD_CTRL_BST_16 |
		      LCD_CTRL_BPP_16 |
		      LCD_CTRL_OFUM,
	      /* 16words burst, enable out FIFO underrun irq */
	400, 240, 200, 0, 0, 0, 0, 0, 0,
#elif defined(CONFIG_JZ4750_SLCD_400X240_8352B)
	.cfg =		LCD_CFG_LCDPIN_SLCD |
			LCD_CFG_NEWDES |
			LCD_CFG_MODE_SLCD, /* TFT Smart LCD panel */
	.slcd_cfg =	SLCD_CFG_DWIDTH_16BIT |
			SLCD_CFG_CWIDTH_16BIT |
			SLCD_CFG_CS_ACTIVE_LOW |
			SLCD_CFG_RS_CMD_LOW |
			SLCD_CFG_CLK_ACTIVE_FALLING |
			SLCD_CFG_TYPE_PARALLEL,
	.ctrl =		LCD_CTRL_BST_16 |
			LCD_CTRL_BPP_18_24,
		/* 16words burst, enable out FIFO underrun irq */
	400, 240, 60, 0, 0, 0, 0, 0, 0,
#else
#error "Select LCD panel first!!!"
#endif
};

struct jz4750lcd_panel_t jz4750_tve_panel = {
	.cfg =		LCD_CFG_TVEN | /* output to tve */
			LCD_CFG_NEWDES | /* 8words descriptor */
			LCD_CFG_TVEPEH |
			LCD_CFG_MODE_INTER_CCIR656,
			/* Interlace CCIR656 mode */
	.ctrl =		LCD_CTRL_BST_16, /* 16words burst */
	TVE_WIDTH_NTSC, TVE_HEIGHT_NTSC, TVE_FREQ_NTSC, 0,0,0,0,0,0,
};

static struct jz4750_lcd_dma_desc *dma_desc_base;
static struct jz4750_lcd_dma_desc *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;

struct jzfb {
	struct fb_info *fb;
	struct platform_device *pdev;
	void *opaque;		/* holds panel context ptr */

	struct jz4750lcd_panel_t *panel;

	uint32_t pseudo_palette[16];
	unsigned int bpp;

	struct clk *lpclk; /* both jz4750 and jz4755 */
	struct clk *ldclk; /* only on jz4750 */

	struct mutex lock;
	bool is_enabled;
	int tv_out; /* 0 - off, 1 - ntsc, 2 - pal */

	/*
	* Number of frames to wait until doing a forced foreground flush.
	* If it looks like we are double buffering, we can flush on vertical
	* panning instead.
	*/
	unsigned int delay_flush;
};

#define DMA_DESC_NUM		6

static unsigned char *lcd_frame0;
static unsigned char *lcd_frame1;

static struct jz4750_lcd_dma_desc *dma0_desc_cmd0, *dma0_desc_cmd;

#ifdef CONFIG_FB_JZ4750_SLCD
static unsigned char *lcd_cmdbuf;
#endif

static void ctrl_enable(struct jzfb *jzfb)
{
	jzfb->delay_flush = 0;

	REG_LCD_STATE = 0; /* clear lcdc status */

	__lcd_clr_dis();
	__lcd_set_ena(); /* enable lcdc */
#ifdef CONFIG_FB_JZ4750_SLCD
	if (!(jzfb->panel->cfg & LCD_CFG_TVEN))
		jzpanel_ops->enable(jzfb->opaque);
#endif
}

static void ctrl_disable(struct jzfb *jzfb)
{
	if (jzfb->panel->cfg & LCD_CFG_LCDPIN_SLCD ||
			jzfb->panel->cfg & LCD_CFG_TVEN) {
		/* Smart lcd and TVE mode only support quick disable */
		__lcd_clr_ena();
	} else {
		int cnt;

		/* Use regular disable: finishes current frame, then stops. */
		__lcd_set_dis();

		/* Wait 20 ms for frame to end (at 60Hz, one frame is 17ms). */
		for (cnt = 20; cnt > 0 && !__lcd_disable_done(); cnt -= 4)
			msleep(4);
		if (cnt <= 0)
			dev_err(&jzfb->pdev->dev,
				"LCD disable timeout! REG_LCD_STATE=0x%08xx\n",
				REG_LCD_STATE);

		REG_LCD_STATE &= ~LCD_STATE_LDD;
	}
}

static void jzfb_power_up(struct jzfb *jzfb)
{
	// TODO: Configure GPIO pins via pinctrl.

	ctrl_enable(jzfb);

	/* Enable panel AFTER enabling lcdc otherwise slcd may hang up */
	if (!(jzfb->panel->cfg & LCD_CFG_TVEN)) /* temp workaround */
		jzpanel_ops->enable(jzfb->opaque);
}

static void jzfb_power_down(struct jzfb *jzfb)
{
	ctrl_disable(jzfb);

	jzpanel_ops->disable(jzfb->opaque);

	// TODO: Configure GPIO pins via pinctrl.
}

static int jz4750fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	if (regno >= ARRAY_SIZE(jzfb->pseudo_palette))
		return 1;

	if (fb->var.bits_per_pixel == 16)
		((u32 *)fb->pseudo_palette)[regno] =
			(red & 0xf800) | ((green & 0xfc00) >> 5) | (blue >> 11);
	else
		((u32 *)fb->pseudo_palette)[regno] =
			((red & 0xff00) << 8) | (green & 0xff00) | (blue >> 8);

	return 0;
}

static int jz4750fb_ioctl(struct fb_info *fb, unsigned int cmd,
			unsigned long arg)
{
	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		/* Implement later, just avoid log message below */
		break;
	default:
		printk("%s, unknown command(0x%x)", __FILE__, cmd);

		break;
	}

	return 0;
}

static int jz4750fb_mmap(struct fb_info *fb, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;

	/* frame buffer memory */
	start = fb->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + fb->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;

	/* Set cacheability to cacheable, write through, no write-allocate. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

static int jz4750fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	D("Requesting mode %i x %i x %i\n",
		var->xres,
		var->yres,
		var->bits_per_pixel);

	if (var->xres > jzfb->panel->w || var->xres > 640)
		return -EINVAL;

	if (var->yres > jzfb->panel->h || var->yres > 480)
		return -EINVAL;

	/* Make sure w/h are divisible by 2 */
	var->xres = (var->xres + 1) & ~1;
	var->yres = (var->yres + 1) & ~1;
	var->xres_virtual = var->xres;
	/* Reserve space for double buffering. */
	var->yres_virtual = var->yres * 2;
	var->xoffset = 0;
	var->yoffset = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	if (var->bits_per_pixel != 32 && var->bits_per_pixel != 16)
		var->bits_per_pixel = 32;

	if (var->bits_per_pixel == 16) {
		var->transp.length = 0;
		var->blue.length = var->red.length = 5;
		var->green.length = 6;
		var->transp.offset = 0;
		var->red.offset = 11;
		var->green.offset = 5;
		var->blue.offset = 0;
	} else {
		var->transp.offset = 24;
		var->red.offset = 16;
		var->green.offset = 8;
		var->blue.offset = 0;
		var->transp.length = var->red.length =
		var->green.length = var->blue.length = 8;
	}

	D("Found working mode: %dx%d, %d bpp\n", var->xres, var->yres,
						 var->bits_per_pixel);

	return 0;
}

static void jz4750fb_set_panel_mode(struct jzfb *jzfb,
			const struct jz4750lcd_panel_t *panel);
static void jz4750fb_foreground_resize(struct jzfb *jzfb,
			  const struct jz4750lcd_panel_t *panel, int fg_change);
static void jz4750fb_change_clock(struct jzfb *jzfb);

static int jz4750fb_set_par(struct fb_info *fb)
{
	struct fb_var_screeninfo *var = &fb->var;
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct jzfb *jzfb = fb->par;

	ctrl_disable(jzfb);

	jzfb->bpp = var->bits_per_pixel;
	jz4750fb_set_panel_mode(jzfb, jzfb->panel);
	jz4750fb_foreground_resize(jzfb, jzfb->panel,
				   FG0_CHANGE_SIZE | FG0_CHANGE_POSITION);

	jz4750fb_change_clock(jzfb);

	if (jzfb->is_enabled)
		ctrl_enable(jzfb);

	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->line_length = var->xres_virtual * (var->bits_per_pixel >> 3);

	return 0;
}

static int jz4750fb_blank(int blank_mode, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	mutex_lock(&jzfb->lock);

	if (blank_mode == FB_BLANK_UNBLANK) {
		if (!jzfb->is_enabled) {
			jzfb_power_up(jzfb);
			jzfb->is_enabled = true;
		}
	} else {
		if (jzfb->is_enabled) {
			jzfb_power_down(jzfb);
			jzfb->is_enabled = false;
		}
	}

	mutex_unlock(&jzfb->lock);

	return 0;
}

static int jz4750fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	if (var->xoffset != fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	D("var.yoffset: %d\n", var->yoffset);

	jzfb->delay_flush = 8;

	dma_cache_wback_inv((unsigned long)(lcd_frame0 +
			    fb->fix.line_length * var->yoffset),
			    fb->fix.line_length * var->yres);

	dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0
			+ (fb->fix.line_length * var->yoffset));
	dma_cache_wback((unsigned int)(dma0_desc0),
			sizeof(struct jz4750_lcd_dma_desc));

	return 0;
}

static struct fb_ops jz4750fb_ops = {
	.owner			= THIS_MODULE,
	.fb_setcolreg		= jz4750fb_setcolreg,
	.fb_check_var		= jz4750fb_check_var,
	.fb_set_par		= jz4750fb_set_par,
	.fb_blank		= jz4750fb_blank,
	.fb_pan_display		= jz4750fb_pan_display,
	.fb_fillrect		= sys_fillrect,
	.fb_copyarea		= sys_copyarea,
	.fb_imageblit		= sys_imageblit,
	.fb_mmap		= jz4750fb_mmap,
	.fb_ioctl		= jz4750fb_ioctl,
};

static int jz4750fb_set_var(struct fb_var_screeninfo *var, int con,
			    struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	var->height                 = jzfb->panel->h;
	var->width                  = jzfb->panel->w;
	var->bits_per_pixel         = jzfb->bpp;

	var->vmode                  = FB_VMODE_NONINTERLACED;
	var->activate               = fb->var.activate;
	var->xres                   = var->width;
	var->yres                   = var->height;
	var->xres_virtual           = var->width;
	var->yres_virtual           = var->height;
	var->xoffset                = 0;
	var->yoffset                = 0;
	var->pixclock               = 0;
	var->left_margin            = 0;
	var->right_margin           = 0;
	var->upper_margin           = 0;
	var->lower_margin           = 0;
	var->hsync_len              = 0;
	var->vsync_len              = 0;
	var->sync                   = 0;
	var->activate              &= ~FB_ACTIVATE_TEST;

	/*
	 * CONUPDATE and SMOOTH_XPAN are equal.  However,
	 * SMOOTH_XPAN is only used internally by fbcon.
	 */
	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = fb->var.xoffset;
		var->yoffset = fb->var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	if (var->bits_per_pixel == 16) {
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		fb->fix.visual		= FB_VISUAL_TRUECOLOR;
		fb->fix.line_length	= var->xres_virtual * 2;
	} else {
		if (var->bits_per_pixel != 32) {
			dev_warn(&jzfb->pdev->dev, "%s: don't support for %dbpp\n",
				fb->fix.id, var->bits_per_pixel);
			var->bits_per_pixel = 32;
		}

		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		var->transp.offset	= 24;
		var->transp.length	= 8;

		fb->fix.visual		= FB_VISUAL_TRUECOLOR;
		fb->fix.line_length	= var->xres_virtual * 4;
	}

	fb->var = *var;
	fb->var.activate &= ~FB_ACTIVATE_ALL;

	return 0;
}

static int jz4750fb_map_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	unsigned int size;
	void *page_virt;

	/* Compute space for max res at 32bpp, double buffered. */
	size = PAGE_ALIGN(640 * 480 * 4 * 2);

	printk("FrameBuffer bpp = %d\n", jzfb->bpp);

	lcd_frame0 = alloc_pages_exact(size, GFP_KERNEL);
	if (!lcd_frame0) {
		printk("jz4750fb, %s: unable to map screen memory\n", fb->fix.id);
		return -ENOMEM;
	}

	dma_desc_base = alloc_pages_exact(PAGE_SIZE, GFP_KERNEL);
	if (!dma_desc_base)
		return -ENOMEM;

#if defined(CONFIG_FB_JZ4750_SLCD)
	lcd_cmdbuf = alloc_pages_exact(PAGE_SIZE, GFP_KERNEL);
	clear_page(lcd_cmdbuf);

	{	int data, i, *ptr;
		ptr = (unsigned int *)lcd_cmdbuf;
		data = WR_GRAM_CMD;
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		for(i = 0; i < 3; i++){
			ptr[i] = data;
		}
	}

	SetPageReserved(virt_to_page(lcd_cmdbuf));
#endif

	SetPageReserved(virt_to_page(dma_desc_base));
	clear_page(dma_desc_base);

	for (page_virt = lcd_frame0;
	     page_virt < lcd_frame0 + size; page_virt += PAGE_SIZE) {
		SetPageReserved(virt_to_page(page_virt));
		clear_page(page_virt);
	}

	fb->fix.smem_start = virt_to_phys(lcd_frame0);
	fb->fix.smem_len = size;
	fb->screen_base = (void *)KSEG1ADDR(lcd_frame0);

	if (!fb->screen_base) {
		return -ENOMEM;
	}

	return 0;
}

static void jz4750fb_unmap_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

	if (lcd_frame0) {
		void *end = lcd_frame0 + fb->fix.smem_len;
		void *page_virt;

		for (page_virt = lcd_frame0; page_virt < end;
					      page_virt += PAGE_SIZE) {
			ClearPageReserved(virt_to_page(page_virt));
		}

		free_pages_exact(lcd_frame0, fb->fix.smem_len);
	}

#if defined(CONFIG_FB_JZ4750_SLCD)
	ClearPageReserved(virt_to_page(lcd_cmdbuf));
	free_pages_exact(lcd_cmdbuf, PAGE_SIZE);
#endif

	if (dma_desc_base) {
		ClearPageReserved(virt_to_page(dma_desc_base));
		free_pages_exact(dma_desc_base, PAGE_SIZE);
	}
}

static void jz4750fb_descriptor_init(struct jzfb *jzfb)
{
	dma0_desc0		= dma_desc_base + 1;
	dma0_desc1		= dma_desc_base + 2;
	dma0_desc_cmd0		= dma_desc_base + 3; /* use only once */
	dma0_desc_cmd		= dma_desc_base + 4;
	dma1_desc0		= dma_desc_base + 5;
	dma1_desc1		= dma_desc_base + 6;

#if defined(CONFIG_FB_JZ4750_SLCD)
	/* First CMD descriptors, use only once, cmd_num isn't 0 */
	dma0_desc_cmd0->next_desc	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd0->databuf		= (unsigned int)virt_to_phys((void *)lcd_cmdbuf);
	dma0_desc_cmd0->frame_id	= (unsigned int)0x0da0cad0; /* dma0's cmd0 */
	dma0_desc_cmd0->cmd		= LCD_CMD_CMD | 3; /* command */
	dma0_desc_cmd0->offsize		= 0;
	dma0_desc_cmd0->page_width	= 0;
	dma0_desc_cmd0->cmd_num		= 3;

	/* Dummy Command Descriptor, cmd_num is 0 */
	dma0_desc_cmd->next_desc	= (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd->databuf		= 0;
	dma0_desc_cmd->frame_id		= (unsigned int)0x0da000cd; /* dma0's cmd0 */
	dma0_desc_cmd->cmd		= LCD_CMD_CMD | 0; /* dummy command */
	dma0_desc_cmd->cmd_num		= 0;
	dma0_desc_cmd->offsize		= 0;
	dma0_desc_cmd->page_width	= 0;
#endif

	/* DMA0 Descriptor0 */
	if (jzfb->panel->cfg & LCD_CFG_TVEN) /* TVE mode */
		dma0_desc0->next_desc	= (unsigned int)virt_to_phys(dma0_desc1);
	else {			/* Normal TFT LCD */
#if defined(CONFIG_FB_JZ4750_SLCD)
		dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
#else
		dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
	}

	dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
	dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */
	dma0_desc0->cmd = LCD_CMD_SOFINT | LCD_CMD_EOFINT;

	/* DMA0 Descriptor1 */
	if (jzfb->panel->cfg & LCD_CFG_TVEN) {
		printk("TV Enable Mode...\n");
#if defined(CONFIG_FB_JZ4750_SLCD)
		dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
#else
		dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
		dma0_desc1->frame_id = (unsigned int)0x0000da01; /* DMA0'1 */
	}

#if defined(CONFIG_FB_JZ4750_SLCD)
	REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd0);
#else
	REG_LCD_DA0 = virt_to_phys(dma0_desc0);
#endif

	/* DMA1 Descriptor0 */
	if (jzfb->panel->cfg & LCD_CFG_TVEN)
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */
	dma1_desc0->cmd = LCD_CMD_SOFINT | LCD_CMD_EOFINT;

	/* DMA1 Descriptor1 */
	if (jzfb->panel->cfg & LCD_CFG_TVEN) { /* TVE mode */
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc1->frame_id = (unsigned int)0x0000da11; /* DMA1'1 */
	}

	REG_LCD_DA1 = virt_to_phys(dma1_desc0);
	dma_cache_wback_inv((unsigned int)(dma_desc_base),
			    (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
}

static void jz4750fb_set_panel_mode(struct jzfb *jzfb,
			const struct jz4750lcd_panel_t *panel)
{
	unsigned int ctrl = panel->ctrl;
	unsigned int osdctrl = 0;
	unsigned int cfg = panel->cfg;
	unsigned int slcd_cfg = panel->slcd_cfg;

	if (jzfb->bpp == 16) {
		ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
		osdctrl |= LCD_OSDCTRL_OSDBPP_15_16;
	} else {
		if (WARN_ON(jzfb->bpp != 32))
			jzfb->bpp = 32;
		ctrl |= LCD_CTRL_BPP_18_24;
		osdctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	REG_LCD_CTRL = ctrl; /* LCDC Controll Register */
	REG_LCD_CFG = cfg; /* LCDC Configure Register */
	REG_SLCD_CFG = slcd_cfg; /* Smart LCD Configure Register */

	if (cfg & LCD_CFG_LCDPIN_SLCD) /* enable Smart LCD DMA */
		REG_SLCD_CTRL = SLCD_CTRL_DMA_EN;

	/* only support TFT16 TFT32, not support STN and Special TFT by now(10-06-2008)*/
	__lcd_vat_set_ht(panel->blw + panel->w + panel->elw);
	__lcd_vat_set_vt(panel->bfw + panel->h + panel->efw);
	__lcd_dah_set_hds(panel->blw);
	__lcd_dah_set_hde(panel->blw + panel->w);
	__lcd_dav_set_vds(panel->bfw);
	__lcd_dav_set_vde(panel->bfw + panel->h);
	__lcd_hsync_set_hps(0);
	__lcd_hsync_set_hpe(panel->hsw);
	__lcd_vsync_set_vpe(panel->vsw);

	REG_LCD_OSDC = LCD_OSDC_F0EN | LCD_OSDC_OSDEN |
		       LCD_OSDC_SOFM0 | LCD_OSDC_EOFM0;
	REG_LCD_OSDCTRL = osdctrl;

	if (panel->cfg & LCD_CFG_TVEN) {
		REG_LCD_RGBC |= LCD_RGBC_YCC; /* enable RGB => YUV */
	} else {
		REG_LCD_RGBC &= ~LCD_RGBC_YCC;
	}

	/* yellow background helps debugging */
	REG_LCD_BGC = 0x00FFFF00;
}

static void jz4750fb_foreground_resize(struct jzfb *jzfb,
			  const struct jz4750lcd_panel_t *panel, int fg_change)
{
	struct fb_var_screeninfo *var = &jzfb->fb->var;
	int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	int x, y, w, h;

	if (!fg_change)
		return;

	w = (var->xres < panel->w ? var->xres : panel->w);
	h = (var->yres < panel->h ? var->yres : panel->h);
	x = (panel->w - w) / 2;
	y = (panel->h - h) / 2;

	printk("OSD: %d x %d, panel: %d x %d\n",
		w, h, panel->w, panel->h);

	fg0_line_size = w * ((jzfb->bpp + 7) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * h;

	printk("fg0_frm_size = 0x%x\n", fg0_frm_size);

	fg1_line_size = w * ((jzfb->bpp + 7) / 8);
	fg1_line_size = ((fg1_line_size + 3) >> 2) << 2; /* word aligned */
	fg1_frm_size = fg1_line_size * h;

	if (fg_change & FG0_CHANGE_POSITION) { /* F0 change position */
		REG_LCD_XYP0 = (y << 16) | x;
	}

	if (fg_change & FG1_CHANGE_POSITION) { /* F1 change position */
		REG_LCD_XYP1 = (y << 16) | x;
	}

	/* set change */
	if (/*!(panel->osd_ctrl & LCD_OSDCTRL_IPU) &&*/
	      (fg_change != FG_CHANGE_ALL))
		REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

	/* wait change ready??? */
	while (REG_LCD_OSDS & LCD_OSDS_READY);

	if (fg_change & FG0_CHANGE_SIZE) { /* change FG0 size */
		if (panel->cfg & LCD_CFG_TVEN) { /* output to TV */
			dma0_desc0->cmd =
			dma0_desc1->cmd = (fg0_frm_size / 4) / 2;
			dma0_desc0->offsize =
			dma0_desc1->offsize = fg0_line_size / 4;
			dma0_desc0->page_width =
			dma0_desc1->page_width =  fg0_line_size / 4;
			dma0_desc1->databuf = virt_to_phys((void *)
					  (lcd_frame0 + fg0_line_size));
			REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
		} else {
			dma0_desc0->cmd =
			dma0_desc1->cmd = fg0_frm_size / 4;
			dma0_desc0->offsize = 0;
			dma0_desc1->offsize = 0;
			dma0_desc0->page_width = 0;
			dma0_desc1->page_width = 0;
		}

		dma0_desc0->desc_size =
		dma0_desc1->desc_size = (h << 16) | w;
		REG_LCD_SIZE0 = (h << 16) | w;

	}

	if (fg_change & FG1_CHANGE_SIZE) { /* change FG1 size*/
		if (panel->cfg & LCD_CFG_TVEN) { /* output to TV */
			dma1_desc0->cmd =
			dma1_desc1->cmd = (fg1_frm_size / 4) / 2;
			dma1_desc0->offsize =
			dma1_desc1->offsize = fg1_line_size / 4;
			dma1_desc0->page_width =
			dma1_desc1->page_width = fg1_line_size / 4;
			dma1_desc1->databuf = virt_to_phys((void *)
					  (lcd_frame1 + fg1_line_size));
			REG_LCD_DA1 = virt_to_phys(dma0_desc1); //tft
		} else {
			dma1_desc0->cmd =
			dma1_desc1->cmd = fg1_frm_size / 4;
			dma1_desc0->offsize = 0;
			dma1_desc1->offsize = 0;
			dma1_desc0->page_width = 0;
			dma1_desc1->page_width = 0;
		}

		dma1_desc0->desc_size =
		dma1_desc1->desc_size = (h << 16) | w;
		REG_LCD_SIZE1 = (h << 16) | w;
	}

	dma_cache_wback((unsigned int)(dma_desc_base),
			(DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
}

static void jz4750fb_change_clock(struct jzfb *jzfb)
{
	unsigned int pclk;

	clk_disable(jzfb->lpclk);

	if ((jzfb->panel->cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = jzfb->panel->fclk *
		      (jzfb->panel->w + jzfb->panel->elw + jzfb->panel->blw) *
		      (jzfb->panel->h + jzfb->panel->efw + jzfb->panel->bfw);
	} else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = jzfb->panel->fclk *
		      (jzfb->panel->w * 3 + jzfb->panel->elw + jzfb->panel->blw) *
		      (jzfb->panel->h + jzfb->panel->efw + jzfb->panel->bfw);
	}

	if (jzfb->panel->cfg & LCD_CFG_TVEN) {
		__cpm_select_tveclk_pll();
		__cpm_select_pixclk_tve();

		/* In TVE mode PCLK = 27MHz */
		clk_set_rate(jzfb->lpclk, 27000000);

#if defined(CONFIG_SOC_JZ4750)
		/* LCDClock > 2.5*Pixclock */
		clk_set_rate(jzfb->ldclk, 27000000 * 3);
#endif
	} else {
		__cpm_select_pixclk_lcd();

		clk_set_rate(jzfb->lpclk, pclk);

#if defined(CONFIG_SOC_JZ4750)
		/* LCDClock > 2.5*Pixclock */
		clk_set_rate(jzfb->ldclk, pclk * 3);
#endif
	}

	jz_clocks.pixclk = __cpm_get_pixclk();
	printk("LCDC: PixClock:%d\n", jz_clocks.pixclk);

#if defined(CONFIG_SOC_JZ4750)
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	printk("LCDC: LcdClock:%d\n", jz_clocks.lcdclk);
#endif

	clk_enable(jzfb->lpclk);
	udelay(1000);
}

static void jz4750fb_deep_set_mode(struct jzfb *jzfb)
{
	struct fb_info *fb = jzfb->fb;

	printk("In jz4750fb_deep_set_mode  \n");

	ctrl_disable(jzfb);

	jz4750fb_descriptor_init(jzfb);
	jz4750fb_set_par(fb);

	printk("Out jz4750fb_deep_set_mode  \n");
}

static irqreturn_t jz4750fb_interrupt_handler(int irq, void *dev_id)
{
	struct jzfb *jzfb = dev_id;
	unsigned int state;
	static int irqcnt = 0;

	state = REG_LCD_STATE;
	D("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & LCD_STATE_SOF) /* Start of frame */
		state &= ~LCD_STATE_SOF;

	if (state & LCD_STATE_EOF) /* End of frame */
		state &= ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		printk("%s, InFiFo0 underrun\n", __FUNCTION__);
		state &= ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		printk("%s, InFiFo1 underrun\n", __FUNCTION__);
		state &= ~LCD_STATE_IFU1;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		state &= ~LCD_STATE_OFU;
		if (irqcnt++ > 100) {
			__lcd_disable_ofu_intr();
			printk("disable Out FiFo underrun irq.\n");
		}
		printk("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	REG_LCD_STATE = state;

	state = REG_LCD_OSDS;

	if (state & LCD_OSDS_SOF0) {
		if (jzfb->delay_flush == 0) {
			struct fb_info *fb = jzfb->fb;

			dma_cache_wback_inv((unsigned long)(lcd_frame0 +
					    fb->fix.line_length * fb->var.yoffset),
					    fb->fix.line_length * fb->var.yres);
		} else {
			jzfb->delay_flush--;
		}

		REG_LCD_OSDS &= ~LCD_OSDS_SOF0;
	}

	if (state & LCD_OSDS_EOF0) {
		/* Add vsync later */
		REG_LCD_OSDS &= ~LCD_OSDS_EOF0;
	}

	return IRQ_HANDLED;
}

#define FB_TVOUT_OFF 0
#define FB_TVOUT_NTSC 1
#define FB_TVOUT_PAL 2
#define FB_TVOUT_LAST 2

static const char *jzfb_tv_out_norm[] = {
	"off", "ntsc", "pal",
};

static void jzfb_tv_out(struct jzfb *jzfb, int mode)
{
	if (mode > FB_TVOUT_LAST)
		return;

	if (jzfb->tv_out == mode)
		return;

	jzfb->tv_out = mode;

	switch (mode) {
	case FB_TVOUT_OFF:
		jz4750tve_disable_tve();

		jzfb->panel = &jz4750_lcd_panel;
		jz4750fb_deep_set_mode(jzfb);
		break;
	case FB_TVOUT_NTSC:
		jz4750tve_disable_tve();

		jzfb->panel = &jz4750_tve_panel;
		jzfb->panel->cfg &= ~LCD_CFG_TVEPEH;
		jzfb->panel->w = TVE_WIDTH_NTSC;
		jzfb->panel->h = TVE_HEIGHT_NTSC;
		jzfb->panel->fclk = TVE_FREQ_NTSC;

		jz4750tve_init(PANEL_MODE_TVE_NTSC);
		udelay(100);
		jz4750tve_enable_tve();
		jz4750fb_deep_set_mode(jzfb);
		break;
	case FB_TVOUT_PAL:
		jz4750tve_disable_tve();

		jzfb->panel = &jz4750_tve_panel;
		jzfb->panel->cfg |= LCD_CFG_TVEPEH;
		jzfb->panel->w = TVE_WIDTH_PAL;
		jzfb->panel->h = TVE_HEIGHT_PAL;
		jzfb->panel->fclk = TVE_FREQ_PAL;

		jz4750tve_init(PANEL_MODE_TVE_PAL);
		udelay(100);
		jz4750tve_enable_tve();
		jz4750fb_deep_set_mode(jzfb);
		break;
	}
}

static ssize_t jzfb_tv_out_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);

	if (jzfb->tv_out > FB_TVOUT_LAST) {
		dev_err(dev, "Unknown norm for TV-out\n");
		return -1;
	}

	return sprintf(buf, "%s\n", jzfb_tv_out_norm[jzfb->tv_out]);
}

static ssize_t jzfb_tv_out_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t n)
{
	size_t i;
	struct jzfb *jzfb = dev_get_drvdata(dev);

	for (i = 0; i <= FB_TVOUT_LAST; i++) {
		if (sysfs_streq(jzfb_tv_out_norm[i], buf)) {
			jzfb_tv_out(jzfb, i);
			return n;
		}
	}

	return -EINVAL;
}

static DEVICE_ATTR(tv_out, 0644, jzfb_tv_out_show, jzfb_tv_out_store);

static void gpio_init(void)
{
#ifdef CONFIG_FB_JZ4750_SLCD
	__gpio_as_slcd_8bit();
#else
	/* gpio init __gpio_as_lcd */
	if (jzfb->panel->cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
	else if (jzfb->panel->cfg & LCD_CFG_MODE_TFT_24BIT)
		__gpio_as_lcd_24bit();
	else
		__gpio_as_lcd_18bit();
#ifdef CONFIG_JZ4750D_RZX50
	/* HACK:
	 * Set D19 (LCD_HSYNC) to proper values
	 * because it's used for d-pad on RZX50.
	 * __gpio_as_lcd_24bit() macro spoils it.
	 */

        __gpio_as_func0(32*3+19);
        __gpio_as_input(32*3+19);
        __gpio_enable_pull(32*3+19);
#endif
#endif
}

static int jz4750_fb_probe(struct platform_device *pdev)
{
	struct jzfb *jzfb;
	struct fb_info *fb;
	int err = 0;

	fb = framebuffer_alloc(sizeof(struct jzfb), &pdev->dev);
	if (!fb) {
		dev_err(&pdev->dev, "Failed to allocate framebuffer device\n");
		err = -ENOMEM;
		goto fb_alloc_failed;
	}

	jzfb = fb->par;
	jzfb->fb = fb;
	jzfb->pdev = pdev;
	//jzfb->pdata = pdata;
	jzfb->bpp = 16;

	/* Later take this from platform data */
	jzfb->panel = &jz4750_lcd_panel;

	strcpy(fb->fix.id, "jz-lcd");
	fb->fix.type		= FB_TYPE_PACKED_PIXELS;
	fb->fix.type_aux	= 0;
	fb->fix.xpanstep	= 1;
	fb->fix.ypanstep	= 1;
	fb->fix.ywrapstep	= 0;
	fb->fix.accel		= FB_ACCEL_NONE;

	fb->var.nonstd		= 0;
	fb->var.activate	= FB_ACTIVATE_NOW;
	fb->var.height		= -1;
	fb->var.width		= -1;
	fb->var.accel_flags	= FB_ACCELF_TEXT;
	fb->var.bits_per_pixel	= jzfb->bpp;

	fb->fbops		= &jz4750fb_ops;
	fb->flags		= FBINFO_FLAG_DEFAULT;

	fb->pseudo_palette	= jzfb->pseudo_palette;

	ctrl_disable(jzfb);

	gpio_init();

	jzpanel_ops->init(&jzfb->opaque, &pdev->dev,
			  0/*pdata->panel_pdata*/);

	err = jz4750fb_map_smem(fb);
	if (err)
		goto map_smem_failed;

	/* Init pixel clock. */
	jzfb->lpclk = devm_clk_get(&pdev->dev, "lpclk");
	if (IS_ERR(jzfb->lpclk)) {
		err = PTR_ERR(jzfb->lpclk);
		dev_err(&pdev->dev, "Failed to get pixel clock: %d\n", err);
		goto failed;
	}

	/* If no ldclk, we are on jz4755, go on without it */
	jzfb->ldclk = devm_clk_get(&pdev->dev, "ldclk");

	jz4750fb_set_var(&fb->var, -1, fb);
	jz4750fb_check_var(&fb->var, fb);

	REG_LCD_STATE = 0; /* clear lcdc status */
	jz4750fb_deep_set_mode(jzfb);

	mutex_init(&jzfb->lock);

	if (request_irq(JZ4750D_IRQ_LCD, jz4750fb_interrupt_handler,
			IRQF_DISABLED, "lcd", jzfb)) {
		dev_err(&pdev->dev, "Failed to request LCD IRQ\n");
		err = -EBUSY;
		goto failed;
	}

	platform_set_drvdata(pdev, jzfb);

	jzfb_power_up(jzfb);
	jzfb->is_enabled = true;

	err = register_framebuffer(fb);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer device\n");
		goto failed;
	}

	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
		fb->node, fb->fix.id, fb->fix.smem_len>>10);

	err = device_create_file(&pdev->dev, &dev_attr_tv_out);
	if (err)
		goto failed;

	return 0;

failed:
	jz4750fb_unmap_smem(fb);
map_smem_failed:
	framebuffer_release(fb);
fb_alloc_failed:

	return err;
}

static int jz4750_fb_remove(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_tv_out);

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	clk_disable(jzfb->lpclk);

	jzpanel_ops->exit(jzfb->opaque);

	jz4750fb_unmap_smem(jzfb->fb);
	framebuffer_release(jzfb->fb);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int jz4750_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Suspending\n");

	if (jzfb->is_enabled)
		jzfb_power_down(jzfb);

	clk_disable(jzfb->lpclk);

	return 0;
}

static int jz4750_fb_resume(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Resuming\n");

	clk_enable(jzfb->lpclk);

	if (jzfb->is_enabled)
		jzfb_power_up(jzfb);

	return 0;
}

#else

#define jz4750_fb_suspend	NULL
#define jz4750_fb_resume	NULL

#endif /* CONFIG_PM */

static struct platform_driver jz4750_fb_driver = {
	.probe		= jz4750_fb_probe,
	.remove		= jz4750_fb_remove,
	.suspend	= jz4750_fb_suspend,
	.resume		= jz4750_fb_resume,
	.driver		= {
		.name = "jz-lcd",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(jz4750_fb_driver);

MODULE_DESCRIPTION("Jz4750 LCD Controller driver");
MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_LICENSE("GPL");

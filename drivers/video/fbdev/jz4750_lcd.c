/*
 * linux/drivers/video/jz4750_lcd.c -- Ingenic Jz4750 LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * --------------------------------
 * NOTE:
 * This LCD driver support TFT16 TFT32 LCD, not support STN and Special TFT LCD
 * now.
 *	It seems not necessory to support STN and Special TFT.
 *	If it's necessary, update this driver in the future.
 *	<Wolfgang Wang, Jun 10 2008>
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
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
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

/* later move to include/video/jzpanel.h */
#ifndef __JZPANEL_H
#define __JZPANEL_H

struct panel_ops {
	int (*init)(void **out_panel, struct device *dev, void *panel_pdata);
	void (*exit)(void *panel);
	void (*enable)(void *panel);
	void (*disable)(void *panel);
};

#endif /* __JZPANEL_H */

struct jzfb_platform_data {
	struct panel_ops *panel_ops;
	void *panel_pdata;
};

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

#if defined(CONFIG_JZ4750D_A380) || defined(CONFIG_JZ4750D_A320E)
 #define SCREEN_WIDTH 400
 #define SCREEN_HEIGHT 240
#elif defined(CONFIG_JZ4750D_RZX50)
 #define SCREEN_WIDTH 480
 #define SCREEN_HEIGHT 272
#endif

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

struct jz4750lcd_panel_t jz4750_info_tve = {
	.cfg =		LCD_CFG_TVEN | /* output to tve */
			LCD_CFG_NEWDES | /* 8words descriptor */
			LCD_CFG_TVEPEH |
			LCD_CFG_MODE_INTER_CCIR656,
			/* Interlace CCIR656 mode */
	.ctrl =		LCD_CTRL_BST_16, /* 16words burst */
	TVE_WIDTH_NTSC, TVE_HEIGHT_NTSC, TVE_FREQ_NTSC, 0,0,0,0,0,0,
};

/* default output to lcd panel */
static struct jz4750lcd_panel_t *jz_panel = &jz4750_lcd_panel;

static struct jzfb *jz4750fb_info;
static struct jz4750_lcd_dma_desc *dma_desc_base;
static struct jz4750_lcd_dma_desc *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;

// 0 - lcd, 1 - tvout
static unsigned long tvout_flag  = 0;

struct jzfb {
	struct fb_info *fb;
	struct platform_device *pdev;
	void *panel;

	uint32_t pseudo_palette[16];
	unsigned int bpp;

	struct mutex lock;
	bool is_enabled;
};

#define DMA_DESC_NUM		6

static unsigned char *lcd_frame0;
static unsigned char *lcd_frame1;

static struct jz4750_lcd_dma_desc *dma0_desc_cmd0, *dma0_desc_cmd;

#ifdef CONFIG_FB_JZ4750_SLCD
static unsigned char *lcd_cmdbuf;
#endif

static void ctrl_enable(void)
{
	__lcd_clr_dis();
	__lcd_set_ena(); /* enable lcdc */
#ifdef CONFIG_FB_JZ4750_SLCD
	jzpanel_ops->enable(jz4750fb_info->panel);
#endif
}

static void ctrl_disable(void)
{
	if (jz_panel->cfg & LCD_CFG_LCDPIN_SLCD ||
			jz_panel->cfg & LCD_CFG_TVEN ) {
		/* Smart lcd and TVE mode only support quick disable */
		__lcd_clr_ena();
	} else {
		int cnt;

		/* Use regular disable: finishes current frame, then stops. */
		__lcd_set_dis();

		/* Wait 20 ms for frame to end (at 60 Hz, one frame is 17 ms). */
		for (cnt = 20; cnt > 0 && !__lcd_disable_done(); cnt -= 4)
			msleep(4);
		if (cnt <= 0)
			printk("LCD disable timeout! REG_LCD_STATE=0x%08xx\n",
				REG_LCD_STATE);

		REG_LCD_STATE &= ~LCD_STATE_LDD;
	}
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

/*
 * switch to tve mode from lcd mode
 * mode:
 *	PANEL_MODE_TVE_PAL: switch to TVE_PAL mode
 *	PANEL_MODE_TVE_NTSC: switch to TVE_NTSC mode
 */
static void jz4750lcd_info_switch_to_TVE(int mode)
{
	jz_panel = &jz4750_info_tve;

	switch (mode) {
	case PANEL_MODE_TVE_PAL:
		/* TVE PAL enable extra halfline signal */
		jz_panel->cfg |= LCD_CFG_TVEPEH;
		jz_panel->w = TVE_WIDTH_PAL;
		jz_panel->h = TVE_HEIGHT_PAL;
		jz_panel->fclk = TVE_FREQ_PAL;
		break;
	case PANEL_MODE_TVE_NTSC:
		/* TVE NTSC disable extra halfline signal */
		jz_panel->cfg &= ~LCD_CFG_TVEPEH;
		jz_panel->w = TVE_WIDTH_NTSC;
		jz_panel->h = TVE_HEIGHT_NTSC;
		jz_panel->fclk = TVE_FREQ_NTSC;
		break;
	default:
		printk("%s, %s: Unknown tve mode\n", __FILE__, __FUNCTION__);
	}
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

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jz4750fb_mmap(struct fb_info *fb, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = fb->fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + fb->fix.smem_len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

#if 1
	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
//	pgprot_val(vma->vm_page_prot) |= _CACHE_CACHABLE_NONCOHERENT;	/* Write-Back */
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}

#define MAKENAME(X,Y) #X "x" #Y

static struct fb_videomode video_modes[] = {
	{
		.name = MAKENAME(SCREEN_WIDTH, SCREEN_HEIGHT),
		.xres = SCREEN_WIDTH,
		.yres = SCREEN_HEIGHT,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

/* checks var and eventually tweaks it to something supported,
 * DO NOT MODIFY PAR */
static int jz4750fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fb)
{
	struct fb_videomode *mode = &video_modes[0];

	if (var->xres != mode->xres)
		return -EINVAL;

	if (var->yres != mode->yres)
		return -EINVAL;

	D("Found working mode: %dx%d\n", mode->xres, mode->yres);

	fb_videomode_to_var(var, mode);

	/* Reserve space for double buffering. */
	/* FIXME: strange behavior when yres_virtual == yres */
	var->yres_virtual = var->yres * 2;

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

	return 0;
}

static void jz4750fb_set_panel_mode(struct jzfb *jzfb,
			const struct jz4750lcd_panel_t *panel);
static void jz4750fb_foreground_resize(struct jzfb *jzfb,
			  const struct jz4750lcd_panel_t *panel, int fg_change);
static void jz4750fb_change_clock(struct jzfb *jzfb);

/*
 * set the video mode according to info->var
 */
static int jz4750fb_set_par(struct fb_info *fb)
{
	struct fb_var_screeninfo *var = &fb->var;
	struct fb_fix_screeninfo *fix = &fb->fix;
	struct jzfb *jzfb = fb->par;

	ctrl_disable();

	jzfb->bpp = var->bits_per_pixel;
	jz4750fb_set_panel_mode(jzfb, jz_panel);
	jz4750fb_foreground_resize(jzfb, jz_panel,
				   FG0_CHANGE_SIZE | FG0_CHANGE_POSITION);
	jz4750fb_change_clock(jzfb);

	ctrl_enable();

	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->line_length = var->xres_virtual * (var->bits_per_pixel >> 3);

	return 0;
}

static void jzfb_enable(struct jzfb *jzfb)
{
	__cpm_start_lcd();
	ctrl_enable();
}

static void jzfb_disable(struct jzfb *jzfb)
{
	ctrl_disable();
	__cpm_stop_lcd();
}

/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jz4750fb_blank(int blank_mode, struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;

/* NOTE: doesn't work good for smart lcd: after blank the whole image shifts.
 * Check dma later? Disable for smart lcd for now
 */
#ifndef CONFIG_FB_JZ4750_SLCD
	mutex_lock(&jzfb->lock);

	if (blank_mode == FB_BLANK_UNBLANK) {
		if (!jzfb->is_enabled) {
			jzfb_enable(jzfb);
			jzfb->is_enabled = true;
		}
	} else {
		if (jzfb->is_enabled) {
			jzfb_disable(jzfb);
			jzfb->is_enabled = false;
		}
	}

	mutex_unlock(&jzfb->lock);
#endif

	return 0;
}

/*
 * pan display
 */
static int jz4750fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *fb)
{
	if (var->xoffset != fb->var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	D("var.yoffset: %d\n", var->yoffset);
	dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0
			+ (fb->fix.line_length * var->yoffset));
	dma_cache_wback((unsigned int)(dma0_desc0),
			sizeof(struct jz4750_lcd_dma_desc));

	return 0;
}

/* use default function cfb_fillrect, cfb_copyarea, cfb_imageblit */
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
	int chgvar = 0;

	var->height                 = jz_panel->h;
	var->width                  = jz_panel->w;
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

	if (fb->var.xres != var->xres)
		chgvar = 1;
	if (fb->var.yres != var->yres)
		chgvar = 1;
	if (fb->var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (fb->var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (fb->var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

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

/*
 * Map screen memory
 */
static int jz4750fb_map_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	unsigned long page;
	unsigned int page_shift, needroom;

	/*
	 * Problem: there could be two panels (lcd/tv) with different sizes.
	 * This code allocates maximum memory, fix it later.
	 */
	needroom = 640 * 480 * 4 * 2;

	printk("FrameBuffer bpp = %d\n", jzfb->bpp);

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	lcd_frame0 = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);

	if (!lcd_frame0)
		return -ENOMEM;

	dma_desc_base =
		(struct jz4750_lcd_dma_desc *)__get_free_pages(GFP_KERNEL, 0);

	if (!dma_desc_base)
		return -ENOMEM;

	memset((void *)lcd_frame0, 0, PAGE_SIZE << page_shift);
	memset((void *)dma_desc_base, 0, PAGE_SIZE);

#if defined(CONFIG_FB_JZ4750_SLCD)
	lcd_cmdbuf = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	memset((void *)lcd_cmdbuf, 0, PAGE_SIZE);

	{	int data, i, *ptr;
		ptr = (unsigned int *)lcd_cmdbuf;
		data = WR_GRAM_CMD;
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		for(i = 0; i < 3; i++){
			ptr[i] = data;
		}
	}

	SetPageReserved(virt_to_page((void*)lcd_cmdbuf));
#endif

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	SetPageReserved(virt_to_page((void*)dma_desc_base));

	for (page = (unsigned long)lcd_frame0;
	     page < PAGE_ALIGN((unsigned long)lcd_frame0 + (PAGE_SIZE<<page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	fb->fix.smem_start = virt_to_phys((void *)lcd_frame0);
	fb->fix.smem_len = (PAGE_SIZE << page_shift); /* page_shift/2 ??? */
	fb->screen_base =
		(unsigned char *)(((unsigned int)lcd_frame0&0x1fffffff) | 0xa0000000);

	if (!fb->screen_base) {
		printk("jz4750fb, %s: unable to map screen memory\n", fb->fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void jz4750fb_unmap_smem(struct fb_info *fb)
{
	struct jzfb *jzfb = fb->par;
	struct page *map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom;

	needroom = 640 * 480 * 4 * 2;

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (fb && fb->screen_base) {
		iounmap(fb->screen_base);
		fb->screen_base = NULL;
		release_mem_region(fb->fix.smem_start,
				   fb->fix.smem_len);
	}

	if (lcd_frame0) {
		for (tmp=(unsigned char *)lcd_frame0;
		     tmp < lcd_frame0 + (PAGE_SIZE << page_shift);
		     tmp += PAGE_SIZE) {
			map = virt_to_page(tmp);
			clear_bit(PG_reserved, &map->flags);
		}
		free_pages((int)lcd_frame0, page_shift);
	}
}

/* initial dma descriptors */
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
	if (jz_panel->cfg & LCD_CFG_TVEN) /* TVE mode */
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

	/* DMA0 Descriptor1 */
	if (jz_panel->cfg & LCD_CFG_TVEN) {
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
	if (jz_panel->cfg & LCD_CFG_TVEN)
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	/* DMA1 Descriptor1 */
	if ( jz_panel->cfg & LCD_CFG_TVEN ) { /* TVE mode */
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

	REG_LCD_OSDC = LCD_OSDC_F0EN | LCD_OSDC_OSDEN;
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
	int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	int x = 0, y = 0;

	fg0_line_size = panel->w * ((jzfb->bpp + 7) / 8);
	fg0_line_size = ((fg0_line_size + 3) >> 2) << 2; /* word aligned */
	fg0_frm_size = fg0_line_size * panel->h;

	printk("fg0_frm_size = 0x%x\n", fg0_frm_size);

	fg1_line_size = panel->w * ((jzfb->bpp + 7) / 8);
	fg1_line_size = ((fg1_line_size + 3) >> 2) << 2; /* word aligned */
	fg1_frm_size = fg1_line_size * panel->h;

	if (fg_change) {
		if (fg_change & FG0_CHANGE_POSITION) { /* F0 change position */
			REG_LCD_XYP0 = (y << 16) | x;
		}

		if (fg_change & FG1_CHANGE_POSITION) { /* F1 change position */
			REG_LCD_XYP1 = (y << 16) | x;
		}

		/* set change */
		if (/*!(panel->osd_ctrl & LCD_OSDCTRL_IPU) &&*/
		     (fg_change != FG_CHANGE_ALL) )
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

		/* wait change ready??? */
		while (REG_LCD_OSDS & LCD_OSDS_READY);

		if (fg_change & FG0_CHANGE_SIZE) { /* change FG0 size */
			if (panel->cfg & LCD_CFG_TVEN) { /* output to TV */
				dma0_desc0->cmd = dma0_desc1->cmd =
						  (fg0_frm_size / 4) / 2;
				dma0_desc0->offsize = dma0_desc1->offsize =
						  fg0_line_size / 4;
				dma0_desc0->page_width = dma0_desc1->page_width
						  =  fg0_line_size/4;
				dma0_desc1->databuf = virt_to_phys((void *)
						  (lcd_frame0 + fg0_line_size));
				REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
			} else {
				dma0_desc0->cmd = dma0_desc1->cmd =
						  fg0_frm_size / 4;
				dma0_desc0->offsize = 0;
				dma0_desc1->offsize = 0;
				dma0_desc0->page_width = 0;
				dma0_desc1->page_width = 0;
			}

			dma0_desc0->desc_size = dma0_desc1->desc_size
				= (panel->h << 16) | panel->w;
			REG_LCD_SIZE0 = (panel->h << 16) | panel->w;

		}

		if (fg_change & FG1_CHANGE_SIZE) { /* change FG1 size*/
			if (panel->cfg & LCD_CFG_TVEN) { /* output to TV */
				dma1_desc0->cmd = dma1_desc1->cmd =
						  (fg1_frm_size / 4) / 2;
				dma1_desc0->offsize = dma1_desc1->offsize =
						      fg1_line_size / 4;
				dma1_desc0->page_width = dma1_desc1->page_width
						      = fg1_line_size / 4;
				dma1_desc1->databuf = virt_to_phys((void *)
						  (lcd_frame1 + fg1_line_size));
				REG_LCD_DA1 = virt_to_phys(dma0_desc1); //tft
			} else {
				dma1_desc0->cmd = dma1_desc1->cmd =
						  fg1_frm_size / 4;
				dma1_desc0->offsize = 0;
				dma1_desc1->offsize = 0;
				dma1_desc0->page_width = 0;
				dma1_desc1->page_width = 0;
			}

			dma1_desc0->desc_size = dma1_desc1->desc_size
				= (panel->h << 16) | panel->w;
			REG_LCD_SIZE1 = (panel->h << 16) | panel->w;;
		}

		dma_cache_wback((unsigned int)(dma_desc_base),
				(DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
	}
}

static void jz4750fb_change_clock(struct jzfb *jzfb)
{
	unsigned int val = 0;
	unsigned int pclk;
	/* Timing setting */
	__cpm_stop_lcd();

	val = jz_panel->fclk; /* frame clk */

	/* Pixclk */
	if ((jz_panel->cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = val *
		      (jz_panel->w + jz_panel->elw + jz_panel->blw) *
		      (jz_panel->h + jz_panel->efw + jz_panel->bfw);
	} else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val *
		      (jz_panel->w * 3 + jz_panel->elw + jz_panel->blw) *
		      (jz_panel->h + jz_panel->efw + jz_panel->bfw);
	}

	/********* In TVE mode PCLK = 27MHz ***********/
	if (jz_panel->cfg & LCD_CFG_TVEN) {		/* LCDC output to TVE */
		REG_CPM_LPCDR  &= (~CPM_LPCDR_LSCS);   //maddrone add
		REG_CPM_LPCDR  |= CPM_LPCDR_LTCS;
		pclk = 27000000;
		//val = __cpm_get_pllout2() / pclk; /* pclk */
		val = 432000000 / pclk; /* pclk */
		printk("maddrone tve: pllout2 = 0x%x\n", __cpm_get_pllout2());
		val--;
		__cpm_set_pixdiv(val);

		D("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
		val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */

		val = __cpm_get_pllout() / val;
		if (val > 0x1f) {
			printk("lcd clock divide is too large, set it to 0x1f\n");
			val = 0x1f;
		}
		__cpm_set_ldiv( val );
#endif
		__cpm_select_pixclk_tve();

		REG_CPM_LPCDR |= CPM_LPCDR_LTCS;  //maddrone add
		REG_CPM_CPCCR |= CPM_CPCCR_CE ; /* update divide */
	} else {		/* LCDC output to  LCD panel */
		val = __cpm_get_pllout2() / pclk; /* pclk */
		val--;
		D("ratio: val = %d\n", val);
		if (val > 0x7ff) {
			printk("pixel clock divid is too large, set it to 0x7ff\n");
			val = 0x7ff;
		}

		__cpm_set_pixdiv(val);

		D("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
		val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
		val = __cpm_get_pllout2() / val;
		if (val > 0x1f) {
			printk("lcd clock divide is too large, set it to 0x1f\n");
			val = 0x1f;
		}
		__cpm_set_ldiv(val);
#endif
		REG_CPM_CPCCR |= CPM_CPCCR_CE ; /* update divide */
	}

	D("REG_CPM_LPCDR=0x%08x\n", REG_CPM_LPCDR);
	D("REG_CPM_CPCCR=0x%08x\n", REG_CPM_CPCCR);

	jz_clocks.pixclk = __cpm_get_pixclk();
	printk("LCDC: PixClock:%d\n", jz_clocks.pixclk);

#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
	jz_clocks.lcdclk = __cpm_get_lcdclk();
	printk("LCDC: LcdClock:%d\n", jz_clocks.lcdclk);
#endif
	__cpm_start_lcd();
	udelay(1000);
}

/*
 * jz4750fb_deep_set_mode,
 *
 */
static void jz4750fb_deep_set_mode(struct jzfb *jzfb)
{
	struct fb_info *fb = jzfb->fb;

	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */

	printk("In jz4750fb_deep_set_mode  \n");

	ctrl_disable();

	jz4750fb_descriptor_init(jzfb);
	jz4750fb_set_par(fb);

	printk("Out jz4750fb_deep_set_mode  \n");
}

static irqreturn_t jz4750fb_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	static int irqcnt = 0;

	state = REG_LCD_STATE;
	D("In the lcd interrupt handler, state=0x%x\n", state);

	if (state & LCD_STATE_EOF) /* End of frame */
		REG_LCD_STATE = state & ~LCD_STATE_EOF;

	if (state & LCD_STATE_IFU0) {
		printk("%s, InFiFo0 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU0;
	}

	if (state & LCD_STATE_IFU1) {
		printk("%s, InFiFo1 underrun\n", __FUNCTION__);
		REG_LCD_STATE = state & ~LCD_STATE_IFU1;
	}

	if (state & LCD_STATE_OFU) { /* Out fifo underrun */
		REG_LCD_STATE = state & ~LCD_STATE_OFU;
		if ( irqcnt++ > 100 ) {
			__lcd_disable_ofu_intr();
			printk("disable Out FiFo underrun irq.\n");
		}
		printk("%s, Out FiFo underrun.\n", __FUNCTION__);
	}

	return IRQ_HANDLED;
}

static int tvout_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%lu\n", tvout_flag);

	return 0;
}

static ssize_t tvout_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *pos)
{
	struct jzfb *jzfb = jz4750fb_info;

	tvout_flag = simple_strtoul(buffer, 0, 10);

	if(tvout_flag == 1) { // lcd to tvout
		jz4750lcd_info_switch_to_TVE(PANEL_MODE_TVE_NTSC);
		jz4750tve_init(PANEL_MODE_TVE_NTSC); /* tve controller init */
		udelay(100);
		jz4750tve_enable_tve();
		jz4750fb_deep_set_mode(jzfb);
	} else if(tvout_flag == 0) { // tvout to lcd
		jz4750tve_disable_tve();
		udelay(100);
		jz_panel = &jz4750_lcd_panel;
		jz4750fb_deep_set_mode(jzfb);
	}

	return count;
}

static int tvout_open(struct inode *inode, struct file *file)
{
	return single_open(file, tvout_show, NULL);
}

static const struct file_operations tvout_fops = {
	.open		= tvout_open,
	.read		= seq_read,
	.write		= tvout_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static void gpio_init(void)
{
#ifdef CONFIG_FB_JZ4750_SLCD
	__gpio_as_slcd_8bit();
#else
	/* gpio init __gpio_as_lcd */
	if (jz_panel->cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
	else if (jz_panel->cfg & LCD_CFG_MODE_TFT_24BIT)
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

	/* In special mode, we only need init special pin,
	 * as general lcd pin has init in uboot */
#if defined(CONFIG_MACH_JZ4750D)
	switch (jz_panel->cfg & LCD_CFG_MODE_MASK) {
	case LCD_CFG_MODE_SPECIAL_TFT_1:
	case LCD_CFG_MODE_SPECIAL_TFT_2:
	case LCD_CFG_MODE_SPECIAL_TFT_3:
		__gpio_as_lcd_special();
		break;
	default:
		;
	}
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

	jz4750fb_info = jzfb = fb->par;
	jzfb->fb = fb;
	jzfb->pdev = pdev;
	//jzfb->pdata = pdata;
	jzfb->bpp = 16;

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

	ctrl_disable();

	gpio_init();

	jzpanel_ops->init(&jzfb->panel, &pdev->dev,
			  0/*pdata->panel_pdata*/);

	err = jz4750fb_map_smem(fb);
	if (err)
		goto map_smem_failed;

	jz4750fb_set_var(&fb->var, -1, fb);
	jz4750fb_check_var(&fb->var, fb);

	REG_LCD_STATE = 0; /* clear lcdc status */
	jz4750fb_deep_set_mode(jzfb);

	err = register_framebuffer(fb);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer device\n");
		goto failed;
	}

	mutex_init(&jzfb->lock);

	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
		fb->node, fb->fix.id, fb->fix.smem_len>>10);

	if (request_irq(JZ4750D_IRQ_LCD, jz4750fb_interrupt_handler,
			IRQF_DISABLED, "lcd", 0)) {
		dev_err(&pdev->dev, "Failed to request LCD IRQ\n");
		err = -EBUSY;
		goto failed;
	}

	platform_set_drvdata(pdev, jzfb);

	jzpanel_ops->enable(jzfb->panel);
	jzfb->is_enabled = true;

	proc_create("jz/tvout", 0644, 0, &tvout_fops);

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

	if (jzfb->is_enabled) {
		jzfb_disable(jzfb);
		jzpanel_ops->disable(jzfb->panel);
	}

	jzpanel_ops->exit(jzfb->panel);

	jz4750fb_unmap_smem(jzfb->fb);
	framebuffer_release(jzfb->fb);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
/*
 * Suspend the LCDC.
 */
static int jz4750_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Suspending\n");

	if (jzfb->is_enabled) {
		jzfb_disable(jzfb);
		jzpanel_ops->disable(jzfb->panel);
	}

	return 0;
}

/*
 * Resume the LCDC.
 */
static int jz4750_fb_resume(struct platform_device *pdev)
{
	struct jzfb *jzfb = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "Resuming\n");

	if (jzfb->is_enabled) {
		jzpanel_ops->enable(jzfb->panel);
		jzfb_enable(jzfb);
	}

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
MODULE_AUTHOR("Wolfgang Wang, <lgwang@ingenic.cn>");
MODULE_LICENSE("GPL");

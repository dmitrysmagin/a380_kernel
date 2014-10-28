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

/*
 * Mod: <maddrone@gmail.com>
 * */

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
#include <linux/pm.h>
#include <linux/proc_fs.h>
#include <linux/kthread.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include "console/fbcon.h"

#include <asm/mach-jz4750d/jz4750d_lcdc.h>
#include <asm/mach-jz4750d/jz4750d_tve.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>

#include "jz4750_lcd.h"
#include "jz4750_tve.h"

/* choose LCD panel */
#if defined(CONFIG_JZ4750_SLCD_A380_ILI9331)
#include "a380_slcd_ili9331.h"
#elif defined(CONFIG_JZ4750_SLCD_400X240_8352B)
#include "a320e_8352b.h"
#endif

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

struct jz4750lcd_info jz4750_lcd_panel = {
#if defined(CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL)
	.panel = {
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
	},
	.osd = {
		 .osd_cfg =	LCD_OSDC_OSDEN | /* Use OSD mode */
				LCD_OSDC_F0EN, /* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x000000, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0xA0,	/* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
		 .fg1 = {16, 0, 0, 480, 272}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_SLCD_A380_ILI9331)
	.panel = {
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
	 },
	.osd = {
		 .osd_cfg =	LCD_OSDC_OSDEN | /* Use OSD mode */
		 		LCD_OSDC_F0EN, /* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = 0,
		 .bgcolor = 0x0000FF, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0x00, /* alpha value */
		 .ipu_restart = 0x80001000, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg1 = {16, 0, 0, 400, 240}, /* bpp, x, y, w, h */
		 .fg0 = {16, 0, 0, 400, 240}, /* bpp, x, y, w, h */
	 },
#elif defined(CONFIG_JZ4750_SLCD_400X240_8352B)
	.panel = {
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
	},
	.osd = {
		.osd_cfg =	LCD_OSDC_OSDEN | /* Use OSD mode */
				LCD_OSDC_F0EN | /* enable Foreground0 */
				LCD_OSDC_F1EN | /* enable Foreground0 */
				LCD_OSDC_ALPHAEN, /* enable Foreground0 */
		.osd_ctrl = 0,		/* disable ipu,  */
		.rgb_ctrl = 0,
		.bgcolor = 0x000000, /* set background color Black */
		.colorkey0 = 0, /* disable colorkey */
		.colorkey1 = 0, /* disable colorkey */
		.alpha = 0x88,	/* alpha value */
		.ipu_restart = 0x80001000, /* ipu restart */
		.fg_change = FG_CHANGE_ALL, /* change all initially */
		.fg0 = {16, 0, 0, 400, 240}, /* bpp, x, y, w, h */
		.fg1 = {16, 0, 0, 400, 240}, /* bpp, x, y, w, h */
	},
#else
#error "Select LCD panel first!!!"
#endif
};

struct jz4750lcd_info jz4750_info_tve = {
	.panel = {
		.cfg =		LCD_CFG_TVEN | /* output to tve */
				LCD_CFG_NEWDES | /* 8words descriptor */
				LCD_CFG_TVEPEH |
				LCD_CFG_MODE_INTER_CCIR656,
				/* Interlace CCIR656 mode */
		.ctrl =		LCD_CTRL_BST_16, /* 16words burst */
		TVE_WIDTH_NTSC, TVE_HEIGHT_NTSC, TVE_FREQ_NTSC, 0,0,0,0,0,0,
	},
	.osd = {
		 .osd_cfg =	LCD_OSDC_OSDEN | /* Use OSD mode */
				 LCD_OSDC_F0EN, /* enable Foreground0 */
		 .osd_ctrl = 0,		/* disable ipu,  */
		 .rgb_ctrl = LCD_RGBC_YCC, /* enable RGB => YUV */
		 .bgcolor = 0x0, /* set background color Black */
		 .colorkey0 = 0, /* disable colorkey */
		 .colorkey1 = 0, /* disable colorkey */
		 .alpha = 0x30,	/* alpha value */
		 .ipu_restart = 0x80000100, /* ipu restart */
		 .fg_change = FG_CHANGE_ALL, /* change all initially */
		 .fg0 = {16,0,0,TVE_WIDTH_NTSC,TVE_HEIGHT_NTSC},
		 .fg1 = {16,0,0,0,0},
	},
};

/* default output to lcd panel */
struct jz4750lcd_info *jz4750_lcd_info = &jz4750_lcd_panel;
static struct lcd_cfb_info *jz4750fb_info;
static struct jz4750_lcd_dma_desc *dma_desc_base;
static struct jz4750_lcd_dma_desc *dma0_desc_palette, *dma0_desc0, *dma0_desc1, *dma1_desc0, *dma1_desc1;

// 0 - lcd, 1 - tvout
static unsigned long tvout_flag  = 0;

struct lcd_cfb_info {
	struct fb_info	fb;
	struct {
		u16 red, green, blue;
	}		palette[NR_PALETTE];
};

#define DMA_DESC_NUM		6

static unsigned char *lcd_palette;
static unsigned char *lcd_frame0;
static unsigned char *lcd_frame1;

static struct jz4750_lcd_dma_desc *dma0_desc_cmd0, *dma0_desc_cmd;

#ifdef CONFIG_FB_JZ4750_SLCD
static unsigned char *lcd_cmdbuf;
#endif

static void jz4750fb_set_mode( struct jz4750lcd_info * lcd_info );
static void jz4750fb_deep_set_mode( struct jz4750lcd_info * lcd_info );

static void ctrl_enable(void)
{
	REG_LCD_STATE = 0; /* clear lcdc status */
#ifdef CONFIG_FB_JZ4750_SLCD
	__lcd_slcd_special_on();
#else
	__lcd_special_on();
#endif
	__lcd_clr_dis();
	__lcd_set_ena(); /* enable lcdc */
}

static void ctrl_disable(void)
{
	if (jz4750_lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ||
			jz4750_lcd_info->panel.cfg & LCD_CFG_TVEN ) {
		__lcd_clr_ena(); /* Smart lcd and TVE mode only support quick disable */
	} else {
		int cnt;
		/* when CPU main freq is 336MHz,wait for 16ms */
		cnt = 336000 * 16;
		__lcd_set_dis(); /* regular disable */
		while(!__lcd_disable_done() && cnt) {
			cnt--;
		}
		if (cnt == 0)
			printk("LCD disable timeout! REG_LCD_STATE=0x%08xx\n",REG_LCD_STATE);
		REG_LCD_STATE &= ~LCD_STATE_LDD;
	}
}

static inline u_int chan_to_field(u_int chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int jz4750fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			  u_int transp, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned short *ptr, ctmp;

	if (regno >= NR_PALETTE)
		return 1;

	cfb->palette[regno].red		= red ;
	cfb->palette[regno].green	= green;
	cfb->palette[regno].blue	= blue;
	if (cfb->fb.var.bits_per_pixel <= 16) {
		red	>>= 8;
		green	>>= 8;
		blue	>>= 8;

		red	&= 0xff;
		green	&= 0xff;
		blue	&= 0xff;
	}
	switch (cfb->fb.var.bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		if (((jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_SINGLE_MSTN ) ||
		    ((jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) == LCD_CFG_MODE_DUAL_MSTN )) {
			ctmp = (77L * red + 150L * green + 29L * blue) >> 8;
			ctmp = ((ctmp >> 3) << 11) | ((ctmp >> 2) << 5) |
				(ctmp >> 3);
		} else {
			/* RGB 565 */
			if (((red >> 3) == 0) && ((red >> 2) != 0))
			red = 1 << 3;
			if (((blue >> 3) == 0) && ((blue >> 2) != 0))
				blue = 1 << 3;
			ctmp = ((red >> 3) << 11)
				| ((green >> 2) << 5) | (blue >> 3);
		}

		ptr = (unsigned short *)lcd_palette;
		ptr = (unsigned short *)(((u32)ptr)|0xa0000000);
		ptr[regno] = ctmp;

		break;

	case 15:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 10) |
				((green >> 3) << 5) |
				(blue >> 3);
		break;
	case 16:
		if (regno < 16) {
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				((red >> 3) << 11) |
				((green >> 2) << 5) |
				(blue >> 3);
		}
		break;
	case 17 ... 32:
		if (regno < 16)
			((u32 *)cfb->fb.pseudo_palette)[regno] =
				(red << 16) |
				(green << 8) |
				(blue << 0);
		break;
	}
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
	struct jz4750lcd_info *info;
	struct jz4750lcd_osd_t *osd_lcd;
	int x, y, w, h;

	info = jz4750_lcd_info = &jz4750_info_tve;
	osd_lcd = &jz4750_lcd_panel.osd;

	switch (mode) {
	case PANEL_MODE_TVE_PAL:
		info->panel.cfg |= LCD_CFG_TVEPEH; /* TVE PAL enable extra halfline signal */
		info->panel.w = TVE_WIDTH_PAL;
		info->panel.h = TVE_HEIGHT_PAL;
		info->panel.fclk = TVE_FREQ_PAL;
		w = ( osd_lcd->fg0.w < TVE_WIDTH_PAL )? osd_lcd->fg0.w:TVE_WIDTH_PAL;
		h = ( osd_lcd->fg0.h < TVE_HEIGHT_PAL )?osd_lcd->fg0.h:TVE_HEIGHT_PAL;
		x = ((TVE_WIDTH_PAL - w) >> 2) << 1;
		y = ((TVE_HEIGHT_PAL - h) >> 2) << 1;
		info->osd.fg0.bpp = osd_lcd->fg0.bpp;
		info->osd.fg0.x = x;
		info->osd.fg0.y = y;
		info->osd.fg0.w = w;
		info->osd.fg0.h = h;
		w = ( osd_lcd->fg1.w < TVE_WIDTH_PAL )? osd_lcd->fg1.w:TVE_WIDTH_PAL;
		h = ( osd_lcd->fg1.h < TVE_HEIGHT_PAL )?osd_lcd->fg1.h:TVE_HEIGHT_PAL;
		x = ((TVE_WIDTH_PAL - w) >> 2) << 1;
		y = ((TVE_HEIGHT_PAL - h) >> 2) << 1;
		info->osd.fg1.bpp = 16;	/* use RGB565 in TVE mode*/
		info->osd.fg1.x = x;
		info->osd.fg1.y = y;
		info->osd.fg1.w = w;
		info->osd.fg1.h = h;
		break;
	case PANEL_MODE_TVE_NTSC:
		info->panel.cfg &= ~LCD_CFG_TVEPEH; /* TVE NTSC disable extra halfline signal */
		info->panel.w = TVE_WIDTH_NTSC;
		info->panel.h = TVE_HEIGHT_NTSC;
		info->panel.fclk = TVE_FREQ_NTSC;
		w = ( osd_lcd->fg0.w < TVE_WIDTH_NTSC )? osd_lcd->fg0.w:TVE_WIDTH_NTSC;
		h = ( osd_lcd->fg0.h < TVE_HEIGHT_NTSC)?osd_lcd->fg0.h:TVE_HEIGHT_NTSC;
		x = ((TVE_WIDTH_NTSC - w) >> 2) << 1;
		y = ((TVE_HEIGHT_NTSC - h) >> 2) << 1;
		info->osd.fg0.bpp = osd_lcd->fg0.bpp;
		info->osd.fg0.x = x;
		info->osd.fg0.y = y;
		info->osd.fg0.w = w;
		info->osd.fg0.h = h;
		w = ( osd_lcd->fg1.w < TVE_WIDTH_NTSC )? osd_lcd->fg1.w:TVE_WIDTH_NTSC;
		h = ( osd_lcd->fg1.h < TVE_HEIGHT_NTSC)?osd_lcd->fg1.h:TVE_HEIGHT_NTSC;
		x = ((TVE_WIDTH_NTSC - w) >> 2) << 1;
		y = ((TVE_HEIGHT_NTSC - h) >> 2) << 1;
		info->osd.fg1.bpp = 16;	/* use RGB565 in TVE mode*/
		info->osd.fg1.x = x;
		info->osd.fg1.y = y;
		info->osd.fg1.w = w;
		info->osd.fg1.h = h;
		break;
	default:
		printk("%s, %s: Unknown tve mode\n", __FILE__, __FUNCTION__);
	}
}

static int jz4750fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case FBIO_GET_MODE:
		D("fbio get mode\n");

		if (copy_to_user(argp, jz4750_lcd_info, sizeof(struct jz4750lcd_info)))
			return -EFAULT;

		break;

	case FBIO_SET_MODE:
		D("fbio set mode\n");

		if (copy_from_user(jz4750_lcd_info, argp, sizeof(struct jz4750lcd_info)))
			return -EFAULT;

		/* set mode */
		jz4750fb_set_mode(jz4750_lcd_info);

		break;

	case FBIO_DEEP_SET_MODE:
		D("fbio deep set mode\n");

		if (copy_from_user(jz4750_lcd_info, argp, sizeof(struct jz4750lcd_info)))
			return -EFAULT;

		jz4750fb_deep_set_mode(jz4750_lcd_info);

		break;

#ifdef CONFIG_FB_JZ4750_TVE
	case FBIO_MODE_SWITCH:
		D("FBIO_MODE_SWITCH");
		switch (arg) {
		case PANEL_MODE_TVE_PAL:	/* switch to TVE_PAL mode */
		case PANEL_MODE_TVE_NTSC:	/* switch to TVE_NTSC mode */
			jz4750lcd_info_switch_to_TVE(arg);
			jz4750tve_init(arg); /* tve controller init */
			udelay(100);
			jz4750tve_enable_tve();
			break;
		case PANEL_MODE_LCD_PANEL:	/* switch to LCD mode */
		default :
			/* turn off TVE, turn off DACn... */
			jz4750tve_disable_tve();
			jz4750_lcd_info = &jz4750_lcd_panel;
			break;
		}

		jz4750fb_deep_set_mode(jz4750_lcd_info);

		break;

	case FBIO_GET_TVE_MODE:
		D("fbio get TVE mode\n");
		if (copy_to_user(argp, jz4750_tve_info, sizeof(struct jz4750tve_info)))
			return -EFAULT;

		break;

	case FBIO_SET_TVE_MODE:
		D("fbio set TVE mode\n");
		if (copy_from_user(jz4750_tve_info, argp, sizeof(struct jz4750tve_info)))
			return -EFAULT;

		/* set tve mode */
		jz4750tve_set_tve_mode(jz4750_tve_info);

		break;
#endif
	default:
		printk("%s, unknown command(0x%x)", __FILE__, cmd);

		break;
	}

	return ret;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int jz4750fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	unsigned long start;
	unsigned long off;
	u32 len;

	off = vma->vm_pgoff << PAGE_SHIFT;
	//fb->fb_get_fix(&fix, PROC_CONSOLE(info), info);

	/* frame buffer memory */
	start = cfb->fb.fix.smem_start;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + cfb->fb.fix.smem_len);
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
static int jz4750fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
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


/*
 * set the video mode according to info->var
 */
static int jz4750fb_set_par(struct fb_info *info)
{
	//printk("jz4750fb_set_par, not implemented\n");
	return 0;
}


/*
 * (Un)Blank the display.
 * Fix me: should we use VESA value?
 */
static int jz4750fb_blank(int blank_mode, struct fb_info *info)
{
	D("jz4750 fb_blank %d %p", blank_mode, info);
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
	//case FB_BLANK_NORMAL:
		/* Turn on panel */
		__lcd_set_ena();

		break;

	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		break;
	default:
		break;

	}
	return 0;
}

/*
 * pan display
 */
static int jz4750fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	/* HACK: var->yoffset should contain correct osd.fg0.h */
	unsigned int dy = var->yoffset ? var->height : 0;

	if (!var || !cfb) {
		return -EINVAL;
	}

	if (var->xoffset != cfb->fb.var.xoffset) {
		/* No support for X panning for now! */
		return -EINVAL;
	}

	/*
	 * TODO: yoffset is the height of requested mode which could be
	 * smaller than panel height. Example: on 480x272 panel if a smaller
	 * res is used (400x240), yoffset is either 0 or 240 (should be 272)
	 * Possible fix: use osd.fg0.h instead ??
	 */
	D("var.yoffset: %d\n", var->yoffset);
	dma0_desc0->databuf = (unsigned int)virt_to_phys((void *)lcd_frame0
			+ (cfb->fb.fix.line_length * dy));
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
			struct fb_info *info)
{
	struct lcd_cfb_info *cfb = (struct lcd_cfb_info *)info;
	struct jz4750lcd_info *lcd_info = jz4750_lcd_info;
	int chgvar = 0;

	var->height                 = lcd_info->osd.fg0.h;	/* tve mode */
	var->width                  = lcd_info->osd.fg0.w;
	var->bits_per_pixel         = lcd_info->osd.fg0.bpp;

	var->vmode                  = FB_VMODE_NONINTERLACED;
	var->activate               = cfb->fb.var.activate;
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
		var->xoffset = cfb->fb.var.xoffset;
		var->yoffset = cfb->fb.var.yoffset;
	}

	if (var->activate & FB_ACTIVATE_TEST)
		return 0;

	if ((var->activate & FB_ACTIVATE_MASK) != FB_ACTIVATE_NOW)
		return -EINVAL;

	if (cfb->fb.var.xres != var->xres)
		chgvar = 1;
	if (cfb->fb.var.yres != var->yres)
		chgvar = 1;
	if (cfb->fb.var.xres_virtual != var->xres_virtual)
		chgvar = 1;
	if (cfb->fb.var.yres_virtual != var->yres_virtual)
		chgvar = 1;
	if (cfb->fb.var.bits_per_pixel != var->bits_per_pixel)
		chgvar = 1;

	//display = fb_display + con;

	var->red.msb_right	= 0;
	var->green.msb_right	= 0;
	var->blue.msb_right	= 0;

	switch(var->bits_per_pixel){
	case 1:	/* Mono */
		cfb->fb.fix.visual	= FB_VISUAL_MONO01;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 2:	/* Mono */
		var->red.offset		= 0;
		var->red.length		= 2;
		var->green.offset	= 0;
		var->green.length	= 2;
		var->blue.offset	= 0;
		var->blue.length	= 2;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= (var->xres * var->bits_per_pixel) / 8;
		break;
	case 4:	/* PSEUDOCOLOUR*/
		var->red.offset		= 0;
		var->red.length		= 4;
		var->green.offset	= 0;
		var->green.length	= 4;
		var->blue.offset	= 0;
		var->blue.length	= 4;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres / 2;
		break;
	case 8:	/* PSEUDOCOLOUR, 256 */
		var->red.offset		= 0;
		var->red.length		= 8;
		var->green.offset	= 0;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_PSEUDOCOLOR;
		cfb->fb.fix.line_length	= var->xres ;
		break;
	case 15: /* DIRECTCOLOUR, 32k */
		var->bits_per_pixel	= 15;
		var->red.offset		= 10;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 5;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_DIRECTCOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 16: /* DIRECTCOLOUR, 64k */
		var->bits_per_pixel	= 16;
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 2;
		break;
	case 17 ... 32:
		/* DIRECTCOLOUR, 256 */
		var->bits_per_pixel	= 32;

		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		var->transp.offset	= 24;
		var->transp.length	= 8;

		cfb->fb.fix.visual	= FB_VISUAL_TRUECOLOR;
		cfb->fb.fix.line_length	= var->xres_virtual * 4;
		break;

	default: /* in theory this should never happen */
		printk(KERN_WARNING "%s: don't support for %dbpp\n",
		       cfb->fb.fix.id, var->bits_per_pixel);
		break;
	}

	cfb->fb.var = *var;
	cfb->fb.var.activate &= ~FB_ACTIVATE_ALL;

	/*
	 * Update the old var.  The fbcon drivers still use this.
	 * Once they are using cfb->fb.var, this can be dropped.
	 *					--rmk
	 */
	//display->var = cfb->fb.var;
	/*
	 * If we are setting all the virtual consoles, also set the
	 * defaults used to create new consoles.
	 */
	fb_set_cmap(&cfb->fb.cmap, &cfb->fb);

	return 0;
}

static struct lcd_cfb_info * jz4750fb_alloc_fb_info(void)
{
	struct lcd_cfb_info *cfb;

	cfb = kmalloc(sizeof(struct lcd_cfb_info) + sizeof(u32) * 16, GFP_KERNEL);

	if (!cfb)
		return NULL;

	jz4750fb_info = cfb;

	memset(cfb, 0, sizeof(struct lcd_cfb_info) );

	strcpy(cfb->fb.fix.id, "jz-lcd");
	cfb->fb.fix.type	= FB_TYPE_PACKED_PIXELS;
	cfb->fb.fix.type_aux	= 0;
	cfb->fb.fix.xpanstep	= 1;
	cfb->fb.fix.ypanstep	= 1;
	cfb->fb.fix.ywrapstep	= 0;
	cfb->fb.fix.accel	= FB_ACCEL_NONE;

	cfb->fb.var.nonstd	= 0;
	cfb->fb.var.activate	= FB_ACTIVATE_NOW;
	cfb->fb.var.height	= -1;
	cfb->fb.var.width	= -1;
	cfb->fb.var.accel_flags	= FB_ACCELF_TEXT;

	cfb->fb.fbops		= &jz4750fb_ops;
	cfb->fb.flags		= FBINFO_FLAG_DEFAULT;

	cfb->fb.pseudo_palette	= (void *)(cfb + 1);

	switch (jz4750_lcd_info->osd.fg0.bpp) {
	case 1:
		fb_alloc_cmap(&cfb->fb.cmap, 4, 0);
		break;
	case 2:
		fb_alloc_cmap(&cfb->fb.cmap, 8, 0);
		break;
	case 4:
		fb_alloc_cmap(&cfb->fb.cmap, 32, 0);
		break;
	case 8:

	default:
		fb_alloc_cmap(&cfb->fb.cmap, 256, 0);
		break;
	}
	D("fb_alloc_cmap,fb.cmap.len:%d....\n", cfb->fb.cmap.len);

	return cfb;
}

static inline int bpp_to_data_bpp(int bpp)
{
	switch (bpp) {
		case 32:
		case 16:
			break;

		case 15:
			bpp = 16;
			break;

		default:
			bpp = -EINVAL;
	}

	return bpp;
}

/*
 * Map screen memory
 */
static int jz4750fb_map_smem(struct lcd_cfb_info *cfb)
{
	unsigned long page;
	unsigned int page_shift, needroom, needroom1, bpp, w, h;

	bpp = bpp_to_data_bpp(jz4750_lcd_info->osd.fg0.bpp);

	/*
	 * Problem: there could be two panels (lcd/tv) with different sizes.
	 * This code allocates maximum memory, fix it later.
	 */
#ifndef CONFIG_FB_JZ4750_TVE
	w = jz4750_lcd_info->osd.fg0.w;
	h = jz4750_lcd_info->osd.fg0.h;
#else
	w = ( jz4750_lcd_info->osd.fg0.w > TVE_WIDTH_PAL )?jz4750_lcd_info->osd.fg0.w:TVE_WIDTH_PAL;
	h = ( jz4750_lcd_info->osd.fg0.h > TVE_HEIGHT_PAL )?jz4750_lcd_info->osd.fg0.h:TVE_HEIGHT_PAL;
#endif
	needroom1 = needroom = ((w * bpp + 7) >> 3) * h;

#if defined(CONFIG_FB_JZ4750_LCD_USE_2LAYER_FRAMEBUFFER)
	bpp = bpp_to_data_bpp(jz4750_lcd_info->osd.fg1.bpp);

#ifndef CONFIG_FB_JZ4750_TVE
	w = jz4750_lcd_info->osd.fg1.w;
	h = jz4750_lcd_info->osd.fg1.h;
#else
	w = ( jz4750_lcd_info->osd.fg1.w > TVE_WIDTH_PAL )?jz4750_lcd_info->osd.fg1.w:TVE_WIDTH_PAL;
	h = ( jz4750_lcd_info->osd.fg1.h > TVE_HEIGHT_PAL )?jz4750_lcd_info->osd.fg1.h:TVE_HEIGHT_PAL;
#endif
	needroom += ((w * bpp + 7) >> 3) * h;
#endif // two layer

	printk("FrameBuffer bpp = %d\n", bpp);

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;
	lcd_palette = (unsigned char *)__get_free_pages(GFP_KERNEL, 0);
	lcd_frame0 = (unsigned char *)__get_free_pages(GFP_KERNEL, page_shift);

	if ((!lcd_palette) || (!lcd_frame0))
		return -ENOMEM;
	memset((void *)lcd_palette, 0, PAGE_SIZE);
	memset((void *)lcd_frame0, 0, PAGE_SIZE << page_shift);

	dma_desc_base = (struct jz4750_lcd_dma_desc *)((void*)lcd_palette + ((PALETTE_SIZE+3)/4)*4);

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
#endif

#if defined(CONFIG_FB_JZ4750_LCD_USE_2LAYER_FRAMEBUFFER)
	lcd_frame1 = lcd_frame0 + needroom1;
#endif

	/*
	 * Set page reserved so that mmap will work. This is necessary
	 * since we'll be remapping normal memory.
	 */
	page = (unsigned long)lcd_palette;
	SetPageReserved(virt_to_page((void*)page));

	for (page = (unsigned long)lcd_frame0;
	     page < PAGE_ALIGN((unsigned long)lcd_frame0 + (PAGE_SIZE<<page_shift));
	     page += PAGE_SIZE) {
		SetPageReserved(virt_to_page((void*)page));
	}

	cfb->fb.fix.smem_start = virt_to_phys((void *)lcd_frame0);
	cfb->fb.fix.smem_len = (PAGE_SIZE << page_shift); /* page_shift/2 ??? */
	cfb->fb.screen_base =
		(unsigned char *)(((unsigned int)lcd_frame0&0x1fffffff) | 0xa0000000);

	if (!cfb->fb.screen_base) {
		printk("jz4750fb, %s: unable to map screen memory\n", cfb->fb.fix.id);
		return -ENOMEM;
	}

	return 0;
}

static void jz4750fb_free_fb_info(struct lcd_cfb_info *cfb)
{
	if (cfb) {
		fb_alloc_cmap(&cfb->fb.cmap, 0, 0);
		kfree(cfb);
	}
}

static void jz4750fb_unmap_smem(struct lcd_cfb_info *cfb)
{
	struct page * map = NULL;
	unsigned char *tmp;
	unsigned int page_shift, needroom, bpp, w, h;

	bpp = jz4750_lcd_info->osd.fg0.bpp;
	if ( bpp == 18 || bpp == 24)
		bpp = 32;
	if ( bpp == 15 )
		bpp = 16;
	w = jz4750_lcd_info->osd.fg0.w;
	h = jz4750_lcd_info->osd.fg0.h;
	needroom = ((w * bpp + 7) >> 3) * h;
#if defined(CONFIG_FB_JZ4750_LCD_USE_2LAYER_FRAMEBUFFER)
	bpp = jz4750_lcd_info->osd.fg1.bpp;
	if ( bpp == 18 || bpp == 24)
		bpp = 32;
	if ( bpp == 15 )
		bpp = 16;
	w = jz4750_lcd_info->osd.fg1.w;
	h = jz4750_lcd_info->osd.fg1.h;
	needroom += ((w * bpp + 7) >> 3) * h;
#endif

	for (page_shift = 0; page_shift < 12; page_shift++)
		if ((PAGE_SIZE << page_shift) >= needroom)
			break;

	if (cfb && cfb->fb.screen_base) {
		iounmap(cfb->fb.screen_base);
		cfb->fb.screen_base = NULL;
		release_mem_region(cfb->fb.fix.smem_start,
				   cfb->fb.fix.smem_len);
	}

	if (lcd_palette) {
		map = virt_to_page(lcd_palette);
		clear_bit(PG_reserved, &map->flags);
		free_pages((int)lcd_palette, 0);
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
static void jz4750fb_descriptor_init( struct jz4750lcd_info * lcd_info )
{
	unsigned int pal_size;

	switch ( lcd_info->osd.fg0.bpp ) {
	case 1:
		pal_size = 4;
		break;
	case 2:
		pal_size = 8;
		break;
	case 4:
		pal_size = 32;
		break;
	case 8:
	default:
		pal_size = 512;
	}

	pal_size /= 4;

	dma0_desc_palette	= dma_desc_base + 0;
	dma0_desc0		= dma_desc_base + 1;
	dma0_desc1		= dma_desc_base + 2;
	dma0_desc_cmd0		= dma_desc_base + 3; /* use only once */
	dma0_desc_cmd		= dma_desc_base + 4;
	dma1_desc0		= dma_desc_base + 5;
	dma1_desc1		= dma_desc_base + 6;

	/*
	 * Normal TFT panel's DMA Chan0:
	 *	TO LCD Panel:
	 *		no palette:	dma0_desc0 <<==>> dma0_desc0
	 *		palette :	dma0_desc_palette <<==>> dma0_desc0
	 *	TO TV Encoder:
	 *		no palette:	dma0_desc0 <<==>> dma0_desc1
	 *		palette:	dma0_desc_palette --> dma0_desc0
	 *				--> dma0_desc1 --> dma0_desc_palette --> ...
	 *
	 * SMART LCD TFT panel(dma0_desc_cmd)'s DMA Chan0:
	 *	TO LCD Panel:
	 *		no palette:	dma0_desc_cmd <<==>> dma0_desc0
	 *		palette :	dma0_desc_palette --> dma0_desc_cmd
	 *				--> dma0_desc0 --> dma0_desc_palette --> ...
	 *	TO TV Encoder:
	 *		no palette:	dma0_desc_cmd --> dma0_desc0
	 *				--> dma0_desc1 --> dma0_desc_cmd --> ...
	 *		palette:	dma0_desc_palette --> dma0_desc_cmd
	 *				--> dma0_desc0 --> dma0_desc1
	 *				--> dma0_desc_palette --> ...
	 * DMA Chan1:
	 *	TO LCD Panel:
	 *		dma1_desc0 <<==>> dma1_desc0
	 *	TO TV Encoder:
	 *		dma1_desc0 <<==>> dma1_desc1
	 */

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

	/* Palette Descriptor */
	dma0_desc_palette->next_desc	= (unsigned int)virt_to_phys(dma0_desc_cmd0);
#else
	/* Palette Descriptor */
	dma0_desc_palette->next_desc	= (unsigned int)virt_to_phys(dma0_desc0);
#endif
	dma0_desc_palette->databuf	= (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id	= (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd		= LCD_CMD_PAL | pal_size; /* Palette Descriptor */

	/* DMA0 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
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
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */

		printk("TV Enable Mode...\n");
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette);
		else
#if defined(CONFIG_FB_JZ4750_SLCD)  /* for smatlcd */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
#else
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
#endif
		dma0_desc1->frame_id = (unsigned int)0x0000da01; /* DMA0'1 */
	}

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else {
#if defined(CONFIG_FB_JZ4750_SLCD)  /* for smartlcd */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd0); //smart lcd
#else
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
#endif
	}

	/* DMA1 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	/* DMA1 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc1->frame_id = (unsigned int)0x0000da11; /* DMA1'1 */
	}

	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	dma_cache_wback_inv((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));

#if 0
	/* Palette Descriptor */
	if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD )
//		dma0_desc_palette->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
		dma0_desc_palette->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd1);
	else
		dma0_desc_palette->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_palette->databuf = (unsigned int)virt_to_phys((void *)lcd_palette);
	dma0_desc_palette->frame_id = (unsigned int)0xaaaaaaaa;
	dma0_desc_palette->cmd	= LCD_CMD_PAL | pal_size; /* Palette Descriptor */

	/* Dummy Command Descriptor, cmd_num is 0 */
	dma0_desc_cmd->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd->databuf	= (unsigned int)virt_to_phys((void *)lcd_cmdbuf);
	dma0_desc_cmd->frame_id = (unsigned int)0x0da0cad0; /* dma0's cmd0 */
	dma0_desc_cmd->cmd	= LCD_CMD_CMD | 3; /* dummy command */
	dma0_desc_cmd->offsize	= 0; /* dummy command */
	dma0_desc_cmd->page_width = 0; /* dummy command */
	dma0_desc_cmd->cmd_num	= 3;

//---------------------------------
	dma0_desc_cmd1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	dma0_desc_cmd1->databuf	= 0;
	dma0_desc_cmd1->frame_id = (unsigned int)0x0da0cad1; /* dma0's cmd0 */
	dma0_desc_cmd1->cmd	= LCD_CMD_CMD | 0; /* dummy command */
	dma0_desc_cmd1->cmd_num	= 0;
	dma0_desc_cmd1->offsize	= 0; /* dummy command */
	dma0_desc_cmd1->page_width = 0; /* dummy command */
//-----------------------------------
	/* DMA0 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc1);
	else{			/* Normal TFT LCD */
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup?? */
//			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette); //tft
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd); // smart lcd
		else if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD )
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd1);
//			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
		else
			dma0_desc0->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
	}

	dma0_desc0->databuf = virt_to_phys((void *)lcd_frame0);
	dma0_desc0->frame_id = (unsigned int)0x0000da00; /* DMA0'0 */

	/* DMA0 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup?? */
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_palette);

		else if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD )
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc_cmd);
		else
			dma0_desc1->next_desc = (unsigned int)virt_to_phys(dma0_desc0);
		dma0_desc1->frame_id = (unsigned int)0x0000da01; /* DMA0'1 */
	}

	/* DMA1 Descriptor0 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) /* TVE mode */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc1);
	else			/* Normal TFT LCD */
		dma1_desc0->next_desc = (unsigned int)virt_to_phys(dma1_desc0);

	dma1_desc0->databuf = virt_to_phys((void *)lcd_frame1);
	dma1_desc0->frame_id = (unsigned int)0x0000da10; /* DMA1'0 */

	/* DMA1 Descriptor1 */
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* TVE mode */
		dma1_desc1->next_desc = (unsigned int)virt_to_phys(dma1_desc0);
		dma1_desc1->frame_id = (unsigned int)0x0000da11; /* DMA1'1 */
	}

	if (lcd_info->osd.fg0.bpp <= 8) /* load palette only once at setup?? */
		REG_LCD_DA0 = virt_to_phys(dma0_desc_palette);
	else
//		REG_LCD_DA0 = virt_to_phys(dma0_desc_cmd); //smart lcd
		REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
	REG_LCD_DA1 = virt_to_phys(dma1_desc0);	/* set Dma-chan1's Descripter Addrress */
	dma_cache_wback_inv((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
#endif
}

static void jz4750fb_set_panel_mode( struct jz4750lcd_info * lcd_info )
{
	struct jz4750lcd_panel_t *panel = &lcd_info->panel;
#ifdef CONFIG_JZ4750D_VGA_DISPLAY
	REG_TVE_CTRL |= TVE_CTRL_DAPD;
	REG_TVE_CTRL &= ~( TVE_CTRL_DAPD1 | TVE_CTRL_DAPD2 | TVE_CTRL_DAPD3);
#endif
	/* set bpp */
	lcd_info->panel.ctrl &= ~LCD_CTRL_BPP_MASK;
	if ( lcd_info->osd.fg0.bpp == 1 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_1;
	else if ( lcd_info->osd.fg0.bpp == 2 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_2;
	else if ( lcd_info->osd.fg0.bpp == 4 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_4;
	else if ( lcd_info->osd.fg0.bpp == 8 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_8;
	else if ( lcd_info->osd.fg0.bpp == 15 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB555;
	else if ( lcd_info->osd.fg0.bpp == 16 )
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_16 | LCD_CTRL_RGB565;
	else if ( lcd_info->osd.fg0.bpp > 16 && lcd_info->osd.fg0.bpp < 32+1 ) {
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	} else {
		printk("The BPP %d is not supported\n", lcd_info->osd.fg0.bpp);
		lcd_info->osd.fg0.bpp = 32;
		lcd_info->panel.ctrl |= LCD_CTRL_BPP_18_24;
	}

	lcd_info->panel.cfg |= LCD_CFG_NEWDES; /* use 8words descriptor always */

	REG_LCD_CTRL = lcd_info->panel.ctrl; /* LCDC Controll Register */
	REG_LCD_CFG = lcd_info->panel.cfg; /* LCDC Configure Register */
	REG_SLCD_CFG = lcd_info->panel.slcd_cfg; /* Smart LCD Configure Register */

	if ( lcd_info->panel.cfg & LCD_CFG_LCDPIN_SLCD ) /* enable Smart LCD DMA */
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
}

static void jz4750fb_set_osd_mode( struct jz4750lcd_info * lcd_info )
{
	D("%s, %d\n", __FILE__, __LINE__ );
	lcd_info->osd.osd_ctrl &= ~(LCD_OSDCTRL_OSDBPP_MASK);
	if ( lcd_info->osd.fg1.bpp == 15 )
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB555;
	else if ( lcd_info->osd.fg1.bpp == 16 )
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_15_16|LCD_OSDCTRL_RGB565;
	else {
		lcd_info->osd.fg1.bpp = 32;
		lcd_info->osd.osd_ctrl |= LCD_OSDCTRL_OSDBPP_18_24;
	}

	REG_LCD_OSDC	= lcd_info->osd.osd_cfg; /* F0, F1, alpha, */

	REG_LCD_OSDCTRL = lcd_info->osd.osd_ctrl; /* IPUEN, bpp */
	REG_LCD_RGBC	= lcd_info->osd.rgb_ctrl;
	REG_LCD_BGC	= lcd_info->osd.bgcolor;
	REG_LCD_KEY0	= lcd_info->osd.colorkey0;
	REG_LCD_KEY1	= lcd_info->osd.colorkey1;
	REG_LCD_ALPHA	= lcd_info->osd.alpha;
	REG_LCD_IPUR	= lcd_info->osd.ipu_restart;
}

static void jz4750fb_foreground_resize( struct jz4750lcd_info * lcd_info )
{
	int fg0_line_size, fg0_frm_size, fg1_line_size, fg1_frm_size;
	/*
	 * NOTE:
	 * Foreground change sequence:
	 *	1. Change Position Registers -> LCD_OSDCTL.Change;
	 *	2. LCD_OSDCTRL.Change -> descripter->Size
	 * Foreground, only one of the following can be change at one time:
	 *	1. F0 size;
	 *	2. F0 position
	 *	3. F1 size
	 *	4. F1 position
	 */

	/*
	 * The rules of f0, f1's position:
	 *	f0.x + f0.w <= panel.w;
	 *	f0.y + f0.h <= panel.h;
	 *
	 * When output is LCD panel, fg.y and fg.h can be odd number or even number.
	 * When output is TVE, as the TVE has odd frame and even frame,
	 * to simplified operation, fg.y and fg.h should be even number always.
	 *
	 */

	/* Foreground 0  */
	if ( lcd_info->osd.fg0.x >= lcd_info->panel.w )
		lcd_info->osd.fg0.x = lcd_info->panel.w;
	if ( lcd_info->osd.fg0.y >= lcd_info->panel.h )
		lcd_info->osd.fg0.y = lcd_info->panel.h;
	if ( lcd_info->osd.fg0.x + lcd_info->osd.fg0.w > lcd_info->panel.w )
		lcd_info->osd.fg0.w = lcd_info->panel.w - lcd_info->osd.fg0.x;
	if ( lcd_info->osd.fg0.y + lcd_info->osd.fg0.h > lcd_info->panel.h )
		lcd_info->osd.fg0.h = lcd_info->panel.h - lcd_info->osd.fg0.y;

	/* Foreground 1 */
	/* Case TVE ??? TVE 720x573 or 720x480*/
	if ( lcd_info->osd.fg1.x >= lcd_info->panel.w )
		lcd_info->osd.fg1.x = lcd_info->panel.w;
	if ( lcd_info->osd.fg1.y >= lcd_info->panel.h )
		lcd_info->osd.fg1.y = lcd_info->panel.h;
	if ( lcd_info->osd.fg1.x + lcd_info->osd.fg1.w > lcd_info->panel.w )
		lcd_info->osd.fg1.w = lcd_info->panel.w - lcd_info->osd.fg1.x;
	if ( lcd_info->osd.fg1.y + lcd_info->osd.fg1.h > lcd_info->panel.h )
		lcd_info->osd.fg1.h = lcd_info->panel.h - lcd_info->osd.fg1.y;

//	fg0_line_size = lcd_info->osd.fg0.w*((lcd_info->osd.fg0.bpp+7)/8);
	fg0_line_size = (lcd_info->osd.fg0.w*(lcd_info->osd.fg0.bpp)/8);
	fg0_line_size = ((fg0_line_size+3)>>2)<<2; /* word aligned */
	fg0_frm_size = fg0_line_size * lcd_info->osd.fg0.h;

	printk("fg0_frm_size = 0x%x\n",fg0_frm_size);

	fg1_line_size = lcd_info->osd.fg1.w*((lcd_info->osd.fg1.bpp+7)/8);
	fg1_line_size = ((fg1_line_size+3)>>2)<<2; /* word aligned */
	fg1_frm_size = fg1_line_size * lcd_info->osd.fg1.h;

	if ( lcd_info->osd.fg_change ) {
		if ( lcd_info->osd.fg_change & FG0_CHANGE_POSITION ) { /* F0 change position */
			REG_LCD_XYP0 = lcd_info->osd.fg0.y << 16 | lcd_info->osd.fg0.x;
		}
		if ( lcd_info->osd.fg_change & FG1_CHANGE_POSITION ) { /* F1 change position */
			REG_LCD_XYP1 = lcd_info->osd.fg1.y << 16 | lcd_info->osd.fg1.x;
		}

		/* set change */
		if ( !(lcd_info->osd.osd_ctrl & LCD_OSDCTRL_IPU) &&
		     (lcd_info->osd.fg_change != FG_CHANGE_ALL) )
			REG_LCD_OSDCTRL |= LCD_OSDCTRL_CHANGES;

		/* wait change ready???  maddrone open*/
		while ( REG_LCD_OSDS & LCD_OSDS_READY )	/* fix in the future, Wolfgang, 06-20-2008 */
		D("wait LCD_OSDS_READY\n");

		if ( lcd_info->osd.fg_change & FG0_CHANGE_SIZE ) { /* change FG0 size */
			if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
				dma0_desc0->cmd = dma0_desc1->cmd = (fg0_frm_size/4)/2;
				dma0_desc0->offsize = dma0_desc1->offsize
					= fg0_line_size/4;
				dma0_desc0->page_width = dma0_desc1->page_width
					= fg0_line_size/4;
				dma0_desc1->databuf = virt_to_phys((void *)(lcd_frame0 + fg0_line_size));
				REG_LCD_DA0 = virt_to_phys(dma0_desc0); //tft
			} else {
				dma0_desc0->cmd = dma0_desc1->cmd = fg0_frm_size/4;
				dma0_desc0->offsize = dma0_desc1->offsize =0;
				dma0_desc0->page_width = dma0_desc1->page_width = 0;
			}

			dma0_desc0->desc_size = dma0_desc1->desc_size
				= lcd_info->osd.fg0.h << 16 | lcd_info->osd.fg0.w;
			REG_LCD_SIZE0 = (lcd_info->osd.fg0.h<<16)|lcd_info->osd.fg0.w;

		}

		if ( lcd_info->osd.fg_change & FG1_CHANGE_SIZE ) { /* change FG1 size*/
			if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) { /* output to TV */
				dma1_desc0->cmd = dma1_desc1->cmd = (fg1_frm_size/4)/2;
				dma1_desc0->offsize = dma1_desc1->offsize = fg1_line_size/4;
				dma1_desc0->page_width = dma1_desc1->page_width = fg1_line_size/4;
				dma1_desc1->databuf = virt_to_phys((void *)(lcd_frame1 + fg1_line_size));
				REG_LCD_DA1 = virt_to_phys(dma0_desc1); //tft
			} else {
				dma1_desc0->cmd = dma1_desc1->cmd = fg1_frm_size/4;
				dma1_desc0->offsize = dma1_desc1->offsize = 0;
				dma1_desc0->page_width = dma1_desc1->page_width = 0;
			}

			dma1_desc0->desc_size = dma1_desc1->desc_size
				= lcd_info->osd.fg1.h << 16 | lcd_info->osd.fg1.w;
			REG_LCD_SIZE1 = lcd_info->osd.fg1.h << 16|lcd_info->osd.fg1.w;
		}

		dma_cache_wback((unsigned int)(dma_desc_base), (DMA_DESC_NUM)*sizeof(struct jz4750_lcd_dma_desc));
		lcd_info->osd.fg_change = FG_NOCHANGE; /* clear change flag */
	}
}

static void jz4750fb_change_clock( struct jz4750lcd_info * lcd_info )
{
	unsigned int val = 0;
	unsigned int pclk;
	/* Timing setting */
	__cpm_stop_lcd();

	val = lcd_info->panel.fclk; /* frame clk */

	/* Pixclk */
	if ( (lcd_info->panel.cfg & LCD_CFG_MODE_MASK) != LCD_CFG_MODE_SERIAL_TFT) {
		pclk = val *
		      (lcd_info->panel.w + lcd_info->panel.elw + lcd_info->panel.blw) *
		      (lcd_info->panel.h + lcd_info->panel.efw + lcd_info->panel.bfw);
	} else {
		/* serial mode: Hsync period = 3*Width_Pixel */
		pclk = val *
		      (lcd_info->panel.w * 3 + lcd_info->panel.elw + lcd_info->panel.blw) *
		      (lcd_info->panel.h + lcd_info->panel.efw + lcd_info->panel.bfw);
	}

	/********* In TVE mode PCLK = 27MHz ***********/
	if ( lcd_info->panel.cfg & LCD_CFG_TVEN ) {		/* LCDC output to TVE */
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

		val =(__cpm_get_pllout()) / val;
		if ( val > 0x1f ) {
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
		if ( val > 0x7ff ) {
			printk("pixel clock divid is too large, set it to 0x7ff\n");
			val = 0x7ff;
		}

		__cpm_set_pixdiv(val);

		D("REG_CPM_LPCDR = 0x%08x\n", REG_CPM_LPCDR);
#if defined(CONFIG_SOC_JZ4750) /* Jz4750D don't use LCLK */
		val = pclk * 3 ;	/* LCDClock > 2.5*Pixclock */
		val =__cpm_get_pllout2() / val;
		if ( val > 0x1f ) {
			printk("lcd clock divide is too large, set it to 0x1f\n");
			val = 0x1f;
		}
		__cpm_set_ldiv( val );
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
	/*
	 * set lcd device clock and lcd pixel clock.
	 * what about TVE mode???
	 *
	 */
}

/*
 * jz4750fb_set_mode(), set osd configure, resize foreground
 *
 */
static void jz4750fb_set_mode( struct jz4750lcd_info * lcd_info )
{
	struct lcd_cfb_info *cfb = jz4750fb_info;

	jz4750fb_set_osd_mode(lcd_info);
	jz4750fb_foreground_resize(lcd_info);
	jz4750fb_set_var(&cfb->fb.var, -1, &cfb->fb);
}

/*
 * jz4750fb_deep_set_mode,
 *
 */
static void jz4750fb_deep_set_mode( struct jz4750lcd_info * lcd_info )
{
	/* configurate sequence:
	 * 1. disable lcdc.
	 * 2. init frame descriptor.
	 * 3. set panel mode
	 * 4. set osd mode
	 * 5. start lcd clock in CPM
	 * 6. enable lcdc.
	 */

	printk("In jz4750fb_deep_set_mode  \n");

#ifdef CONFIG_FB_JZ4750_SLCD
	__lcd_clr_ena();	/* quick disable */
	__slcd_disable_dma();
#else
	__lcd_set_dis();	/* regular disable */
	mdelay(50);
#endif
	lcd_info->osd.fg_change = FG_CHANGE_ALL; /* change FG0, FG1 size, postion??? */
	jz4750fb_descriptor_init(lcd_info);
	jz4750fb_set_panel_mode(lcd_info);
	jz4750fb_set_mode(lcd_info);
	jz4750fb_change_clock(lcd_info);
#ifdef CONFIG_FB_JZ4750_SLCD
	__slcd_enable_dma();
	REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN;
#else
	__lcd_clr_dis();
#endif
	__lcd_set_ena();	/* enable lcdc */
	printk("Out jz4750fb_deep_set_mode  \n");
}

static irqreturn_t jz4750fb_interrupt_handler(int irq, void *dev_id)
{
	unsigned int state;
	static int irqcnt=0;

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

#ifdef CONFIG_PM
/*
 * Suspend the LCDC.
 */
static int jz4750_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	ctrl_disable();

	__cpm_stop_lcd();

	return 0;
}

/*
 * Resume the LCDC.
 */
static int jz4750_fb_resume(struct platform_device *pdev)
{
	__cpm_start_lcd();

	ctrl_enable();

	return 0;
}

#else

#define jz4750_fb_suspend	NULL
#define jz4750_fb_resume	NULL

#endif /* CONFIG_PM */

static int proc_tvout_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", tvout_flag);
}

static int proc_tvout_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	tvout_flag = simple_strtoul(buffer, 0, 10);

	if(tvout_flag == 1) { // lcd to tvout
		jz4750lcd_info_switch_to_TVE(PANEL_MODE_TVE_NTSC);
		jz4750tve_init(PANEL_MODE_TVE_NTSC); /* tve controller init */
		udelay(100);
		jz4750tve_enable_tve();
		jz4750fb_deep_set_mode(jz4750_lcd_info);
	} else if(tvout_flag == 0) { // tvout to lcd
		jz4750tve_disable_tve();
		udelay(100);
		jz4750_lcd_info = &jz4750_lcd_panel;
		jz4750fb_deep_set_mode(jz4750_lcd_info);
	}

	return count;
}

static void gpio_init(void)
{
	__lcd_special_pin_init();

#ifdef CONFIG_FB_JZ4750_SLCD
	__gpio_as_slcd_8bit();
#else
	/* gpio init __gpio_as_lcd */
	if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_16BIT)
		__gpio_as_lcd_16bit();
	else if (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_TFT_24BIT)
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
	switch (jz4750_lcd_info->panel.cfg & LCD_CFG_MODE_MASK) {
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

static void set_bpp_to_ctrl_bpp(void)
{
	switch (jz4750_lcd_info->osd.fg0.bpp) {
		case 15:
		case 16:
			break;

		case 17 ... 32:
			jz4750_lcd_info->osd.fg0.bpp = 32;
			break;

		default:
			D("FG0: BPP (%d) not support, Set BPP 32.\n",
					jz4750_lcd_info->osd.fg0.bpp);

			jz4750_lcd_info->osd.fg0.bpp = 32;
			break;
	}

	switch (jz4750_lcd_info->osd.fg1.bpp) {
		case 15:
		case 16:
			break;

		case 17 ... 32:
			jz4750_lcd_info->osd.fg1.bpp = 32;
			break;

		default:
			D("FG1: BPP (%d) not support, Set BPP 32.\n",
					jz4750_lcd_info->osd.fg1.bpp);

			jz4750_lcd_info->osd.fg1.bpp = 32;
			break;
	}
}

static void slcd_init(void)
{
	/* Configure SLCD module for setting smart lcd control registers */
#if defined(CONFIG_FB_JZ4750_SLCD)
	__lcd_as_smart_lcd();
	__slcd_disable_dma();
	//__init_slcd_bus();	/* Note: modify this depend on you lcd */

#endif
}

static int __devinit jz4750_fb_probe(struct platform_device *dev)
{
	struct lcd_cfb_info *cfb;
	int err = 0;
	struct proc_dir_entry *res;

	cfb = jz4750fb_alloc_fb_info();
	if (!cfb)
		goto failed;

	ctrl_disable();

	gpio_init();
	slcd_init();

	set_bpp_to_ctrl_bpp();

	/* init clk */
	jz4750fb_change_clock(jz4750_lcd_info);

	err = jz4750fb_map_smem(cfb);
	if (err)
		goto failed;

	jz4750fb_deep_set_mode(jz4750_lcd_info);

	err = register_framebuffer(&cfb->fb);
	if (err < 0) {
		D("Failed to register framebuffer device.");
		goto failed;
	}

	printk("fb%d: %s frame buffer device, using %dK of video memory\n",
		cfb->fb.node, cfb->fb.fix.id, cfb->fb.fix.smem_len>>10);

	if (request_irq(IRQ_LCD, jz4750fb_interrupt_handler, IRQF_DISABLED,
			"lcd", 0)) {
		D("Failed to request LCD IRQ.\n");
		err = -EBUSY;
		goto failed;
	}

	ctrl_enable();

	//maddrone add
	res = create_proc_entry("jz/tvout", 0, NULL);
	if(res) {
		res->read_proc = proc_tvout_read_proc;
		res->write_proc = proc_tvout_write_proc;
	}

	return 0;

failed:
	jz4750fb_unmap_smem(cfb);
	jz4750fb_free_fb_info(cfb);

	return err;
}

static int __devexit jz4750_fb_remove(struct platform_device *pdev)
{
	struct lcd_cfb_info *cfb = jz4750fb_info;

	jz4750fb_unmap_smem(cfb);
	jz4750fb_free_fb_info(cfb);

	return 0;
}

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

static int __init jz4750_fb_init(void)
{
	return platform_driver_register(&jz4750_fb_driver);
}

static void __exit jz4750_fb_cleanup(void)
{
	platform_driver_unregister(&jz4750_fb_driver);
}

module_init(jz4750_fb_init);
module_exit(jz4750_fb_cleanup);

MODULE_DESCRIPTION("Jz4750 LCD Controller driver");
MODULE_AUTHOR("Wolfgang Wang, <lgwang@ingenic.cn>");
MODULE_LICENSE("GPL");

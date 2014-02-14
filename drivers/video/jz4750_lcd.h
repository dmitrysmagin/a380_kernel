/*
 * linux/drivers/video/jz4750_lcd.h -- Ingenic Jz4750 On-Chip LCD frame buffer device
 *
 * Copyright (C) 2005-2008, Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ4750_LCD_H__
#define __JZ4750_LCD_H__

//#include <asm/io.h>

#define NR_PALETTE	256
#define PALETTE_SIZE	(NR_PALETTE*2)

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


struct jz4750lcd_fg_t {
	int bpp;	/* foreground bpp */
	int x;		/* foreground start position x */
	int y;		/* foreground start position y */
	int w;		/* foreground width */
	int h;		/* foreground height */
};

struct jz4750lcd_osd_t {
	unsigned int osd_cfg;	/* OSDEN, ALHPAEN, F0EN, F1EN, etc */
	unsigned int osd_ctrl;	/* IPUEN, OSDBPP, etc */
	unsigned int rgb_ctrl;	/* RGB Dummy, RGB sequence, RGB to YUV */
	unsigned int bgcolor;	/* background color(RGB888) */
	unsigned int colorkey0;	/* foreground0's Colorkey enable, Colorkey value */
	unsigned int colorkey1; /* foreground1's Colorkey enable, Colorkey value */
	unsigned int alpha;	/* ALPHAEN, alpha value */
	unsigned int ipu_restart; /* IPU Restart enable, ipu restart interval time */

#define FG_NOCHANGE 		0x0000
#define FG0_CHANGE_SIZE 	0x0001
#define FG0_CHANGE_POSITION 	0x0002
#define FG1_CHANGE_SIZE 	0x0010
#define FG1_CHANGE_POSITION 	0x0020
#define FG_CHANGE_ALL 		( FG0_CHANGE_SIZE | FG0_CHANGE_POSITION | \
				  FG1_CHANGE_SIZE | FG1_CHANGE_POSITION )
	int fg_change;
	struct jz4750lcd_fg_t fg0;	/* foreground 0 */
	struct jz4750lcd_fg_t fg1;	/* foreground 1 */
};

struct jz4750lcd_info {
	struct jz4750lcd_panel_t panel;
	struct jz4750lcd_osd_t osd;
};


/* Jz LCDFB supported I/O controls. */
#define FBIOSETBACKLIGHT	0x4688 /* set back light level */
#define FBIODISPON		0x4689 /* display on */
#define FBIODISPOFF		0x468a /* display off */
#define FBIORESET		0x468b /* lcd reset */
#define FBIOPRINT_REG		0x468c /* print lcd registers(debug) */
#define FBIOROTATE		0x46a0 /* rotated fb */
#define FBIOGETBUFADDRS		0x46a1 /* get buffers addresses */
#define FBIO_GET_MODE		0x46a2 /* get lcd info */
#define FBIO_SET_MODE		0x46a3 /* set osd mode */
#define FBIO_DEEP_SET_MODE	0x46a4 /* set panel and osd mode */
#define FBIO_MODE_SWITCH	0x46a5 /* switch mode between LCD and TVE */
#define FBIO_GET_TVE_MODE	0x46a6 /* get tve info */
#define FBIO_SET_TVE_MODE	0x46a7 /* set tve mode */

/*
 * LCD panel specific definition
 */

/* board RZX50 */
#if defined(CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL)

#define MODE 0xc9		/* 8bit serial RGB */

/* SPI pins */
#define SPEN		(32*3+22)	/*LCD_CS*/
#define SPCK		(32*4+13)	/*LCD_SCL*/
#define SPDA		(32*4+12)	/*LCD_SDA*/
#define LCD_RET 	(32*4+2)	/*LCD_DISP_N use for lcd reset*/


#define __spi_write_reg1(reg, val) \
do { \
	unsigned char no;\
	unsigned short value;\
	unsigned char a=0;\
	unsigned char b=0;\
	a=reg;\
	b=val;\
	__gpio_set_pin(SPEN);\
	__gpio_set_pin(SPCK);\
	__gpio_clear_pin(SPDA);\
	__gpio_clear_pin(SPEN);\
	udelay(25);\
	value=((a<<8)|(b&0xFF));\
	for(no=0;no<16;no++)\
	{\
		__gpio_clear_pin(SPCK);\
		if((value&0x8000)==0x8000)\
		__gpio_set_pin(SPDA);\
		else\
		__gpio_clear_pin(SPDA);\
		udelay(25);\
		__gpio_set_pin(SPCK);\
		value=(value<<1); \
		udelay(25);\
	 }\
	__gpio_set_pin(SPEN);\
	udelay(100);\
} while (0)

#define __spi_write_reg(reg, val) \
do {\
	__spi_write_reg1((reg<<2|2), val); \
	udelay(100); \
}while(0)

#define __lcd_special_pin_init() \
do { \
	__gpio_as_output(SPEN); \
	__gpio_as_output(SPCK); \
	__gpio_as_output(SPDA); \
	__gpio_as_output(LCD_RET);\
	udelay(50);\
	__gpio_clear_pin(LCD_RET);\
	mdelay(150);\
	__gpio_set_pin(LCD_RET);\
} while (0)

#define __lcd_special_on() \
do { \
	udelay(50);\
	__gpio_clear_pin(LCD_RET);\
	mdelay(150);\
	__gpio_set_pin(LCD_RET);\
	mdelay(10);\
	__spi_write_reg(0x00, 0x03); \
	__spi_write_reg(0x01, 0x40); \
	__spi_write_reg(0x02, 0x11); \
	__spi_write_reg(0x03, MODE); \
	__spi_write_reg(0x04, 0x32); \
	__spi_write_reg(0x05, 0x0e); \
	__spi_write_reg(0x07, 0x03); \
	__spi_write_reg(0x08, 0x08); \
	__spi_write_reg(0x09, 0x32); \
	__spi_write_reg(0x0A, 0x88); \
	__spi_write_reg(0x0B, 0xc6); \
	__spi_write_reg(0x0C, 0x20); \
	__spi_write_reg(0x0D, 0x20); \
} while (0)	//reg 0x0a is control the display direction:DB0->horizontal level DB1->vertical level

#define __lcd_special_off() \
do { \
	__spi_write_reg(0x00, 0x03); \
} while (0)

#endif /* CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL */

/*
 * Platform specific definition
 */

/* board A380 or RZX50 */
#if defined(CONFIG_JZ4750D_A380) || defined(CONFIG_JZ4750D_RZX50)

/* stubs */
#ifndef __lcd_special_on
#define __lcd_special_on()
#endif

#ifndef __lcd_special_off
#define __lcd_special_off()
#endif

#define __lcd_display_pin_init() \
do { \
	__gpio_as_output(GPIO_LCD_VCC_EN_N);	 \
	__gpio_as_output(GPIO_LCD_PWM);	 \
	__lcd_special_pin_init();	   \
} while (0)

#define __lcd_display_on() \
do { \
	__lcd_special_on();	\
	__lcd_set_backlight_level(60);		\
	mdelay(300); \
	__gpio_set_pin(GPIO_LCD_VCC_EN_N);	\
} while (0)

#define __lcd_display_off() \
do { \
	__lcd_close_backlight();		\
	__lcd_special_off();	 \
} while (0)

#endif


#endif /* __JZ4750_LCD_H__ */

/*
 * linux/drivers/video/jz4740_slcd.h
 * -- LCD panel definitions for Ingenic On-Chip SLCD frame buffer device
 *
 * Copyright (C) 2005-2007, Ingenic Semiconductor Inc.
 * Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __JZ4740_SLCD_H__
#define __JZ4740_SLCD_H__

#include <asm/mach-jz4740/base.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

/*************************************************************************
 * SLCD (Smart LCD Controller)
 *************************************************************************/

#define JZ_REG_SLCD_CFG		0xA0	/* SLCD Configure Register */
#define JZ_REG_SLCD_CTRL	0xA4	/* SLCD Control Register */
#define JZ_REG_SLCD_STATE	0xA8	/* SLCD Status Register */
#define JZ_REG_SLCD_DATA	0xAC	/* SLCD Data Register */
#define JZ_REG_SLCD_FIFO	0xB0	/* SLCD FIFO Register */

/* SLCD Configure Register */
#define SLCD_CFG_BURST_BIT		14
#define SLCD_CFG_BURST_MASK		(0x3 << SLCD_CFG_BURST_BIT)
  #define SLCD_CFG_BURST_4_WORD		(0 << SLCD_CFG_BURST_BIT)
  #define SLCD_CFG_BURST_8_WORD		(1 << SLCD_CFG_BURST_BIT)
#define SLCD_CFG_DWIDTH_BIT		10
#define SLCD_CFG_DWIDTH_MASK		(0x7 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_18		(0 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_16		(1 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_8_x3		(2 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_8_x2		(3 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_8_x1		(4 << SLCD_CFG_DWIDTH_BIT)
  #define SLCD_CFG_DWIDTH_9_x2		(7 << SLCD_CFG_DWIDTH_BIT)
#define SLCD_CFG_CWIDTH_BIT		8
#define SLCD_CFG_CWIDTH_MASK		(0x3 << SLCD_CFG_CWIDTH_BIT)
  #define SLCD_CFG_CWIDTH_16BIT		(0 << SLCD_CFG_CWIDTH_BIT)
  #define SLCD_CFG_CWIDTH_8BIT		(1 << SLCD_CFG_CWIDTH_BIT)
  #define SLCD_CFG_CWIDTH_18BIT		(2 << SLCD_CFG_CWIDTH_BIT)
#define SLCD_CFG_CS_ACTIVE_LOW		(0 << 4)
#define SLCD_CFG_CS_ACTIVE_HIGH		(1 << 4)
#define SLCD_CFG_RS_CMD_LOW		(0 << 3)
#define SLCD_CFG_RS_CMD_HIGH		(1 << 3)
#define SLCD_CFG_CLK_ACTIVE_FALLING	(0 << 1)
#define SLCD_CFG_CLK_ACTIVE_RISING	(1 << 1)
#define SLCD_CFG_TYPE_PARALLEL		(0 << 0)
#define SLCD_CFG_TYPE_SERIAL		(1 << 0)

/* SLCD Control Register */
#define SLCD_CTRL_DMA_EN		(1 << 0)

/* SLCD Status Register */
#define SLCD_STATE_BUSY			(1 << 0)

/* SLCD Data Register */
#define SLCD_DATA_RS_DATA		(0 << 31)
#define SLCD_DATA_RS_COMMAND		(1 << 31)

/* SLCD FIFO Register */
#define SLCD_FIFO_RS_DATA		(0 << 31)
#define SLCD_FIFO_RS_COMMAND		(1 << 31)

/*************************************************************************/

struct jzfb_framedesc {
	uint32_t next;
	uint32_t addr;
	uint32_t id;
	uint32_t cmd;
} __attribute__((packed));

struct jzfb {
	struct fb_info *fb;
	struct platform_device *pdev;
	void __iomem *base;
	struct resource *mem;
	struct jz4740_fb_platform_data *pdata;
	const struct jz_slcd_panel *panel;

	size_t vidmem_size;
	void *vidmem;
	dma_addr_t vidmem_phys;

	size_t blackline_size;
	void *blackline;
	dma_addr_t blackline_phys;

	struct jzfb_framedesc (*framedesc)[3];
	dma_addr_t framedesc_phys;

	struct jz4740_dma_chan *dma;
	struct completion dma_completion;
	unsigned refresh_on_pan:1;

	struct clk *ldclk;
	struct clk *lpclk;

	unsigned int tv_out;
	unsigned tv_out_vsync:1;

	unsigned is_enabled:1;
	struct mutex lock; /* Protecting against running enable/disable in paralell */

	struct delayed_work refresh_work;

	uint32_t pseudo_palette[16];
#ifdef CONFIG_JZ_SLCD_ILI9338
	unsigned int rgb[3];
#endif
};

struct jz_slcd_panel {
	/* request and configure GPIO pins */
	int (*init)(struct jzfb *jzfb);
	/* free GPIO pins */
	void (*exit)(struct jzfb *jzfb);
	/* activate, reset and initialize */
	void (*enable)(struct jzfb *jzfb);
	/* deactivate */
	void (*disable)(struct jzfb *jzfb);
	/* panel name */
	const char *name;
};

const struct jz_slcd_panel *jz_slcd_panel_from_name(const char *name);
const struct jz_slcd_panel *jz_slcd_panels_probe(struct jzfb *jzfb);

#endif  /*__JZ4740_SLCD_H__*/

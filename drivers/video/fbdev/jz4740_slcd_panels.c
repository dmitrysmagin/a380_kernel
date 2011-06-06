/*
 * linux/drivers/video/jz4740_slcd_panels.c
 * -- LCD panel definitions for Ingenic On-Chip SLCD frame buffer device
 *
 * Copyright (C) 2005-2007, Ingenic Semiconductor Inc.
 * Copyright (C) 2009, Ignacio Garcia Perez <iggarpe@gmail.com>
 * Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2011, ChinaChip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/io.h>
#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include "jz4740_slcd.h"

static char *default_slcd_panel;
#ifdef CONFIG_JZ_SLCD_ILI9338
static unsigned int default_slcd_rgb[3] = { 100, 100, 100, };
#endif

/* Send a command without data. */
static void send_panel_command(struct jzfb *jzfb, u32 cmd) {
	u16 slcd_cfg = readw(jzfb->base + JZ_REG_SLCD_CFG);
	switch (slcd_cfg & SLCD_CFG_CWIDTH_MASK) {
	case SLCD_CFG_CWIDTH_8BIT:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8), jzfb->base + JZ_REG_SLCD_DATA);
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_CWIDTH_16BIT:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | (cmd&0xffff), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_CWIDTH_18BIT:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | ((cmd&0xff00) << 2) | ((cmd&0xff) << 1), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	default:
		break;
	}
}

/* Send data without command. */
static void send_panel_data(struct jzfb *jzfb, u32 data)
{
	u16 slcd_cfg = readw(jzfb->base + JZ_REG_SLCD_CFG);
	switch (slcd_cfg & SLCD_CFG_DWIDTH_MASK) {
	case SLCD_CFG_DWIDTH_18:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		writel(SLCD_DATA_RS_DATA | ((data<<6)&0xfc0000)|((data<<4)&0xfc00) | ((data<<2)&0xfc), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_DWIDTH_16:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | (data&0xffff), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_DWIDTH_9_x2:
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		data = ((data << 6) & 0xfc0000) | ((data << 4) & 0xfc00) | ((data << 2) & 0xfc);
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | data, jzfb->base + JZ_REG_SLCD_DATA);
		break;
	default:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | (data&0xffff), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	}
}

/* Send command and data. */
static void set_panel_reg(struct jzfb *jzfb, u32 cmd, u32 data)
{
	send_panel_command(jzfb, cmd);
	send_panel_data(jzfb, data);
}

#ifdef CONFIG_JZ_SLCD_ILI9325

// TODO(MtH): GPIO assignments belong in the board definition, since two
//            boards using the same panel controller could still use different
//            GPIO assignments.
// TODO(MtH): CS mismatch: B17 (A320) vs C20 (standard).
#define ILI9325_GPIO_CS_N 	JZ_GPIO_PORTB(17)	/* Chip select */
#define ILI9325_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */

static void ili9325_enable(struct jzfb *jzfb)
{
	/* RESET pulse */
	gpio_set_value(ILI9325_GPIO_RESET_N, 0);
	msleep(10);
	gpio_set_value(ILI9325_GPIO_RESET_N, 1);
	msleep(50);

	/* Enable chip select */
	gpio_set_value(ILI9325_GPIO_CS_N, 0);

	/* Black magic */
	set_panel_reg(jzfb, 0xE3, 0x3008);
	set_panel_reg(jzfb, 0xE7, 0x0012);
	set_panel_reg(jzfb, 0xEF, 0x1231);
	set_panel_reg(jzfb, 0x01, 0x0100);
	set_panel_reg(jzfb, 0x02, 0x0700);
	set_panel_reg(jzfb, 0x03, 0x1098);
	set_panel_reg(jzfb, 0x04, 0x0000);
	set_panel_reg(jzfb, 0x08, 0x0207);
	set_panel_reg(jzfb, 0x09, 0x0000);
	set_panel_reg(jzfb, 0x0A, 0x0000);
	set_panel_reg(jzfb, 0x0C, 0x0000);
	set_panel_reg(jzfb, 0x0D, 0x0000);
	set_panel_reg(jzfb, 0x0F, 0x0000);
	set_panel_reg(jzfb, 0x10, 0x0000);
	set_panel_reg(jzfb, 0x11, 0x0007);
	set_panel_reg(jzfb, 0x12, 0x0000);
	set_panel_reg(jzfb, 0x13, 0x0000);
	msleep(200);
	set_panel_reg(jzfb, 0x10, 0x1290);
	set_panel_reg(jzfb, 0x11, 0x0227);
	msleep(50);
	set_panel_reg(jzfb, 0x12, 0x001B);
	msleep(50);
	set_panel_reg(jzfb, 0x13, 0x0500);
	set_panel_reg(jzfb, 0x29, 0x000C);
	set_panel_reg(jzfb, 0x2B, 0x000D);
	msleep(50);
	set_panel_reg(jzfb, 0x20, 0x0000);
	set_panel_reg(jzfb, 0x21, 0x0000);
	set_panel_reg(jzfb, 0x30, 0x0000);
	set_panel_reg(jzfb, 0x31, 0x0204);
	set_panel_reg(jzfb, 0x32, 0x0200);
	set_panel_reg(jzfb, 0x35, 0x0007);
	set_panel_reg(jzfb, 0x36, 0x1404);
	set_panel_reg(jzfb, 0x37, 0x0705);
	set_panel_reg(jzfb, 0x38, 0x0305);
	set_panel_reg(jzfb, 0x39, 0x0707);
	set_panel_reg(jzfb, 0x3C, 0x0701);
	set_panel_reg(jzfb, 0x3D, 0x000E);
	set_panel_reg(jzfb, 0x50, 0x0000);
	set_panel_reg(jzfb, 0x51, 0x00EF);
	set_panel_reg(jzfb, 0x52, 0x0000);
	set_panel_reg(jzfb, 0x53, 0x013F);
	set_panel_reg(jzfb, 0x60, 0xA700);
	set_panel_reg(jzfb, 0x61, 0x0001);
	set_panel_reg(jzfb, 0x6A, 0x0000);
	set_panel_reg(jzfb, 0x80, 0x0000);
	set_panel_reg(jzfb, 0x81, 0x0000);
	set_panel_reg(jzfb, 0x82, 0x0000);
	set_panel_reg(jzfb, 0x83, 0x0000);
	set_panel_reg(jzfb, 0x84, 0x0000);
	set_panel_reg(jzfb, 0x85, 0x0000);
	set_panel_reg(jzfb, 0x90, 0x0010);
	set_panel_reg(jzfb, 0x92, 0x0600);
	msleep(50);
	set_panel_reg(jzfb, 0x07, 0x0133);
	msleep(50);
	send_panel_command(jzfb, 0x22);
}

/* TODO(IGP): make sure LCD power consumption is low in these conditions */
static void ili9325_disable(struct jzfb *jzfb)
{
	/* Keep chip select disabled */
	gpio_set_value(ILI9325_GPIO_CS_N, 1);
	/* Keep RESET active */
	gpio_set_value(ILI9325_GPIO_RESET_N, 0);
}

static int ili9325_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(ILI9325_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(ILI9325_GPIO_CS_N, 1);

	ret = gpio_request(ILI9325_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(ILI9325_GPIO_RESET_N, 0);

	msleep(100);
	return 0;

err_reset:
	gpio_free(ILI9325_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for ILI9325 panel driver\n");
	return ret;
}

static void ili9325_exit(struct jzfb *jzfb)
{
	gpio_free(ILI9325_GPIO_CS_N);
	gpio_free(ILI9325_GPIO_RESET_N);
}

#endif

#ifdef CONFIG_JZ_SLCD_ILI9331

#define ILI9331_GPIO_CS_N 	JZ_GPIO_PORTB(17)	/* Chip select */
#define ILI9331_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */

static void ili9331_enable(struct jzfb *jzfb)
{
	/* RESET pulse */
	gpio_set_value(ILI9331_GPIO_RESET_N, 0);
	msleep(10);
	gpio_set_value(ILI9331_GPIO_RESET_N, 1);
	msleep(50);

	/* Enable chip select */
	gpio_set_value(ILI9331_GPIO_CS_N, 0);

	/* Black magic */
	set_panel_reg(jzfb, 0xE7, 0x1014);
	set_panel_reg(jzfb, 0x01, 0x0000);
	set_panel_reg(jzfb, 0x02, 0x0200);
	set_panel_reg(jzfb, 0x03, 0x1048);
	set_panel_reg(jzfb, 0x08, 0x0202);
	set_panel_reg(jzfb, 0x09, 0x0000);
	set_panel_reg(jzfb, 0x0A, 0x0000);
	set_panel_reg(jzfb, 0x0C, 0x0000);
	set_panel_reg(jzfb, 0x0D, 0x0000);
	set_panel_reg(jzfb, 0x0F, 0x0000);
	set_panel_reg(jzfb, 0x10, 0x0000);
	set_panel_reg(jzfb, 0x11, 0x0007);
	set_panel_reg(jzfb, 0x12, 0x0000);
	set_panel_reg(jzfb, 0x13, 0x0000);
	msleep(100);
	set_panel_reg(jzfb, 0x10, 0x1690);
	set_panel_reg(jzfb, 0x11, 0x0224);
	msleep(50);
	set_panel_reg(jzfb, 0x12, 0x001F);
	msleep(50);
	set_panel_reg(jzfb, 0x13, 0x0500);
	set_panel_reg(jzfb, 0x29, 0x000C);
	set_panel_reg(jzfb, 0x2B, 0x000D);
	msleep(50);
	set_panel_reg(jzfb, 0x30, 0x0000);
	set_panel_reg(jzfb, 0x31, 0x0106);
	set_panel_reg(jzfb, 0x32, 0x0000);
	set_panel_reg(jzfb, 0x35, 0x0204);
	set_panel_reg(jzfb, 0x36, 0x160A);
	set_panel_reg(jzfb, 0x37, 0x0707);
	set_panel_reg(jzfb, 0x38, 0x0106);
	set_panel_reg(jzfb, 0x39, 0x0706);
	set_panel_reg(jzfb, 0x3C, 0x0402);
	set_panel_reg(jzfb, 0x3D, 0x0C0F);
	set_panel_reg(jzfb, 0x50, 0x0000);
	set_panel_reg(jzfb, 0x51, 0x00EF);
	set_panel_reg(jzfb, 0x52, 0x0000);
	set_panel_reg(jzfb, 0x53, 0x013F);
	set_panel_reg(jzfb, 0x20, 0x0000);
	set_panel_reg(jzfb, 0x21, 0x0000);
	set_panel_reg(jzfb, 0x60, 0x2700);
	set_panel_reg(jzfb, 0x61, 0x0001);
	set_panel_reg(jzfb, 0x6A, 0x0000);
	set_panel_reg(jzfb, 0x80, 0x0000);
	set_panel_reg(jzfb, 0x81, 0x0000);
	set_panel_reg(jzfb, 0x82, 0x0000);
	set_panel_reg(jzfb, 0x83, 0x0000);
	set_panel_reg(jzfb, 0x84, 0x0000);
	set_panel_reg(jzfb, 0x85, 0x0000);
	set_panel_reg(jzfb, 0x20, 0x00EF);
	set_panel_reg(jzfb, 0x21, 0x0190);
	set_panel_reg(jzfb, 0x90, 0x0010);
	set_panel_reg(jzfb, 0x92, 0x0600);
	set_panel_reg(jzfb, 0x07, 0x0133);
	send_panel_command(jzfb, 0x22);
}

/* TODO(IGP): make sure LCD power consumption is low in these conditions */
static void ili9331_disable(struct jzfb *jzfb)
{
	/* Keep chip select disabled */
	gpio_set_value(ILI9331_GPIO_CS_N, 1);
	/* Keep RESET active */
	gpio_set_value(ILI9331_GPIO_RESET_N, 0);
}

static int ili9331_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(ILI9331_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(ILI9331_GPIO_CS_N, 1);

	ret = gpio_request(ILI9331_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(ILI9331_GPIO_RESET_N, 0);

	msleep(100);
	return 0;

err_reset:
	gpio_free(ILI9331_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for ILI9331 panel driver\n");
	return ret;
}

static void ili9331_exit(struct jzfb *jzfb)
{
	gpio_free(ILI9331_GPIO_CS_N);
	gpio_free(ILI9331_GPIO_RESET_N);
}

#endif

#ifdef CONFIG_JZ_SLCD_ILI9338

static void ili9338_set_color_table(struct jzfb *jzfb)
{
	unsigned int c;
	struct device *dev = &jzfb->pdev->dev;

	/* Set up a custom color lookup table.
	 * This helps to fix the 'blueish' display on some devices. */
	send_panel_command(jzfb, 0x2d);

	for (c = 0; c < 3; c++) {
		unsigned int i, n, v, s;
		n = c == 1 ? 64 /* 6 bits G */ : 32 /* 5 bits R/B */;
		s = jzfb->rgb[c] * (((63 << 24) - 1) / (100 * (n - 1)));
		v = 0;
		for (i = 0; i < n; i++, v += s)
			send_panel_data(jzfb, (v >> 24) + ((v >> 23) & 1));
	}

	dev_info(dev, "ILI9338 color table initialized with R=%u G=%u B=%u\n",
				jzfb->rgb[0], jzfb->rgb[1], jzfb->rgb[2]);
}

module_param_array_named(rgb, default_slcd_rgb, uint, NULL, 0);
MODULE_PARM_DESC(rgb, "comma-separated list of three values representing the percentage of red, green and blue");

static ssize_t rgb_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	return sprintf(buf, "%u,%u,%u\n", jzfb->rgb[0],
				jzfb->rgb[1], jzfb->rgb[2]);
}

static ssize_t rgb_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t n)
{
	struct jzfb *jzfb = dev_get_drvdata(dev);
	unsigned int rgb[3];

	if (sscanf(buf, "%u,%u,%u", &rgb[0], &rgb[1], &rgb[2]) < 3)
		return -EINVAL;

	if (rgb[0] > 100 || rgb[1] > 100 || rgb[2] > 100)
		return -EINVAL;

	memcpy(jzfb->rgb, rgb, sizeof(rgb));
	ili9338_set_color_table(jzfb);
	return n;
}

static DEVICE_ATTR(rgb, 0644, rgb_show, rgb_store);

#define ILI9338_GPIO_CS_N 	JZ_GPIO_PORTB(17)	/* Chip select */
#define ILI9338_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */

static void ili9338_enable(struct jzfb *jzfb)
{
	/* RESET pulse */
	gpio_set_value(ILI9338_GPIO_RESET_N, 0);
	msleep(10);
	gpio_set_value(ILI9338_GPIO_RESET_N, 1);
	msleep(50);

	/* Enable chip select */
	gpio_set_value(ILI9338_GPIO_CS_N, 0);

	/* Black magic */
	send_panel_command(jzfb, 0x11);
	msleep(100);

	send_panel_command(jzfb, 0xCB);
	send_panel_data(jzfb, 0x01);

	send_panel_command(jzfb, 0xC0);
	send_panel_data(jzfb, 0x26);
	send_panel_data(jzfb, 0x01);
	send_panel_command(jzfb, 0xC1);
	send_panel_data(jzfb, 0x10);
	send_panel_command(jzfb, 0xC5);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x52);

	send_panel_command(jzfb, 0x26);
	send_panel_data(jzfb, 0x01);
	send_panel_command(jzfb, 0xE0);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x08);
	send_panel_data(jzfb, 0x0E);
	send_panel_data(jzfb, 0x06);
	send_panel_data(jzfb, 0x42);
	send_panel_data(jzfb, 0x28);
	send_panel_data(jzfb, 0x36);
	send_panel_data(jzfb, 0x03);
	send_panel_data(jzfb, 0x0E);
	send_panel_data(jzfb, 0x04);
	send_panel_data(jzfb, 0x13);
	send_panel_data(jzfb, 0x0E);
	send_panel_data(jzfb, 0x0C);
	send_panel_command(jzfb, 0XE1);
	send_panel_data(jzfb, 0x0C);
	send_panel_data(jzfb, 0x23);
	send_panel_data(jzfb, 0x26);
	send_panel_data(jzfb, 0x04);
	send_panel_data(jzfb, 0x0C);
	send_panel_data(jzfb, 0x04);
	send_panel_data(jzfb, 0x39);
	send_panel_data(jzfb, 0x24);
	send_panel_data(jzfb, 0x4B);
	send_panel_data(jzfb, 0x03);
	send_panel_data(jzfb, 0x0B);
	send_panel_data(jzfb, 0x0B);
	send_panel_data(jzfb, 0x33);
	send_panel_data(jzfb, 0x37);
	send_panel_data(jzfb, 0x0F);

	send_panel_command(jzfb, 0x2a);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x01);
	send_panel_data(jzfb, 0x3f);

	send_panel_command(jzfb, 0x2b);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0xef);

	send_panel_command(jzfb, 0x36);
	send_panel_data(jzfb, 0xe8);

	send_panel_command(jzfb, 0x3A);
	send_panel_data(jzfb, 0x05);

	ili9338_set_color_table(jzfb);

	send_panel_command(jzfb, 0x29);

	send_panel_command(jzfb, 0x2c);
}

/* TODO(IGP): make sure LCD power consumption is low in these conditions */
static void ili9338_disable(struct jzfb *jzfb)
{
	/* Keep chip select disabled */
	gpio_set_value(ILI9338_GPIO_CS_N, 1);
	/* Keep RESET active */
	gpio_set_value(ILI9338_GPIO_RESET_N, 0);
}

static int ili9338_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(ILI9338_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(ILI9338_GPIO_CS_N, 1);

	ret = gpio_request(ILI9338_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(ILI9338_GPIO_RESET_N, 0);

	memcpy(jzfb->rgb, default_slcd_rgb, sizeof(default_slcd_rgb));
	msleep(100);

	ret = device_create_file(dev, &dev_attr_rgb);
	if (!ret)
		return 0;

err_reset:
	gpio_free(ILI9338_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for ILI9338 panel driver\n");
	return ret;
}

static void ili9338_exit(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	device_remove_file(dev, &dev_attr_rgb);

	gpio_free(ILI9338_GPIO_CS_N);
	gpio_free(ILI9338_GPIO_RESET_N);
}

#endif

static const struct jz_slcd_panel jz_slcd_panels[] = {
#ifdef CONFIG_JZ_SLCD_ILI9325
	{
		ili9325_init, ili9325_exit,
		ili9325_enable, ili9325_disable,
		"ili9325",
	},
#endif
#ifdef CONFIG_JZ_SLCD_ILI9331
	{
		ili9331_init, ili9331_exit,
		ili9331_enable, ili9331_disable,
		"ili9331",
	},
#endif
#ifdef CONFIG_JZ_SLCD_ILI9338
	{
		ili9338_init, ili9338_exit,
		ili9338_enable, ili9338_disable,
		"ili9338",
	},
#endif
};

module_param_named(panel, default_slcd_panel, charp, 0);
MODULE_PARM_DESC(panel, "SLCD panel used on the device");

const struct jz_slcd_panel *jz_slcd_panel_from_name(const char *name)
{
	unsigned int i;
	for (i = 0; i < ARRAY_SIZE(jz_slcd_panels); i++) {
		if (sysfs_streq(name, jz_slcd_panels[i].name))
			return &jz_slcd_panels[i];
	}
	return NULL;
}

const struct jz_slcd_panel *jz_slcd_panels_probe(struct jzfb *jzfb)
{
	const struct jz_slcd_panel *panel;
	if (ARRAY_SIZE(jz_slcd_panels) == 0)
		return NULL;

	panel = &jz_slcd_panels[0];

	if (default_slcd_panel) {
		panel = jz_slcd_panel_from_name(default_slcd_panel);
		if (!panel) {
			struct device *dev = &jzfb->pdev->dev;
			dev_err(dev, "Unknown SLCD panel: %s\n",
						default_slcd_panel);
		}
	}
	return panel;
}

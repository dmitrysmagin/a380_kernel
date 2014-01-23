/*
 * linux/arch/mips/jz4750d/board-rzx50.c
 *
 * Ritmix RZX50 board setup routines.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 * Mod: <maddrone@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/jzsoc.h>
#include <asm/mach-jz4750d/jz4750d_mmc.h>
#include <asm/mach-jz4750d/platform.h>

#include "serial.h"
#include "clock.h"

struct jz4750d_clock_board_data jz4750d_clock_bdata = {
	.ext_rate = 24000000,
	.rtc_rate = 32768,
};

void __init board_msc_init(void);

static void cetus_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_4bit();
}

static void cetus_sd_power_on(struct device *dev)
{
}

static void cetus_sd_power_off(struct device *dev)
{
}

static void cetus_sd_cpm_start(struct device *dev)
{
	__cpm_start_msc(0);
}

static unsigned int cetus_sd_status(struct device *dev)
{
	return 1;
}

static void cetus_sd_plug_change(int state)
{
}

static unsigned int cetus_sd_get_wp(struct device *dev)
{
	return 0;
}

struct jz_mmc_platform_data cetus_sd_data = {
	.support_sdio   = 1,
	.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq     = 0,
	.detect_pin     = 0,
	.nonremovable   = 1,
	.init           = cetus_sd_gpio_init,
	.power_on       = cetus_sd_power_on,
	.power_off      = cetus_sd_power_off,
	.cpm_start      = cetus_sd_cpm_start,
	.status         = cetus_sd_status,
	.plug_change    = cetus_sd_plug_change,
	.write_protect  = cetus_sd_get_wp,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.bus_width      = 4, /* 1, 4 or 8 */
};

static void cetus_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_1bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void cetus_tf_power_on(struct device *dev)
{
	__msc1_enable_power();
}

static void cetus_tf_power_off(struct device *dev)
{
	__msc1_disable_power();
}

static void cetus_tf_cpm_start(struct device *dev)
{
	__cpm_start_msc(1);
}

static unsigned int cetus_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (!status);
}

static void cetus_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_fall_edge(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_rise_edge(MSC1_HOTPLUG_PIN);
}

struct jz_mmc_platform_data cetus_tf_data = {
	.support_sdio   = 1,
	.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq     = MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = cetus_tf_gpio_init,
	.power_on       = cetus_tf_power_on,
	.power_off      = cetus_tf_power_off,
	.cpm_start      = cetus_tf_cpm_start,
	.status         = cetus_tf_status,
	.plug_change    = cetus_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.bus_width      = 1,
};

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_lcd_device,
	&jz_usb_gdt_device,
	//&jz_i2c_device,
	&jz_msc0_device,
	&jz_msc1_device,
};

static int __init rzx50_init_platform_devices(void)
{
	jz_msc0_device.dev.platform_data = &cetus_sd_data;
	jz_msc1_device.dev.platform_data = &cetus_tf_data;

	jz4750d_serial_device_register();

	return platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
}

static void __init board_gpio_setup(void)
{
	__gpio_as_output(GPIO_AMPEN);
	__gpio_clear_pin(GPIO_AMPEN);
#ifdef GPIO_CHARGE
	__gpio_as_output(GPIO_CHARGE);
	__gpio_set_pin(GPIO_CHARGE);
#endif
}

static int __init jz_board_setup(void)
{
	printk("Ritmix RZX50 board setup %s %s\n",__DATE__,__TIME__);

	board_gpio_setup();

	if (rzx50_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}

arch_initcall(jz_board_setup);

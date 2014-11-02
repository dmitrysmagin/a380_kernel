/*
 * linux/arch/mips/jz4750d/board-a380.c
 *
 * Dingoo A380 board setup routines.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 * Mod: <maddrone@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mmc/host.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/pwm_backlight.h>
#include <linux/input/matrix_keypad.h>

#include <asm/mach-jz4750d/board-a380.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>

#include <asm/mach-jz4750d/jz4750d_mmc.h>
#include <asm/mach-jz4750d/gpio.h>
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
	.status         = cetus_sd_status,
	.plug_change    = cetus_sd_plug_change,
	.write_protect  = cetus_sd_get_wp,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
			  MMC_CAP_4_BIT_DATA,
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
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= MSC1_HOTPLUG_IRQ,
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = cetus_tf_gpio_init,
	.power_on       = cetus_tf_power_on,
	.power_off      = cetus_tf_power_off,
	.status		= cetus_tf_status,
	.plug_change	= cetus_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.bus_width      = 1,
};

/* Keys mapped to gpio */
static struct gpio_keys_button a380_buttons[] = {
	/* D-pad up */ {
		.gpio			= JZ_GPIO_PORTE(0),
		.active_low		= 1,
		.code			= KEY_UP,
		.debounce_interval	= 5,
	},
	/* D-pad down */ {
		.gpio			= JZ_GPIO_PORTE(1),
		.active_low		= 1,
		.code			= KEY_DOWN,
		.debounce_interval	= 5,
	},
	/* D-pad left */ {
		.gpio			= JZ_GPIO_PORTE(2),
		.active_low		= 1,
		.code			= KEY_LEFT,
		.debounce_interval	= 5,
	},
	/* D-pad right */ {
		.gpio			= JZ_GPIO_PORTE(3),
		.active_low		= 1,
		.code			= KEY_RIGHT,
		.debounce_interval	= 5,
	},
	/* A button */ {
		.gpio			= JZ_GPIO_PORTC(31),
		.active_low		= 0,
		.code			= KEY_LEFTCTRL,
		.debounce_interval	= 5,
	},
	/* B button */ {
		.gpio			= JZ_GPIO_PORTD(16),
		.active_low		= 1,
		.code			= KEY_LEFTALT,
		.debounce_interval	= 5,
	},
	/* X button */ {
		.gpio			= JZ_GPIO_PORTD(17),
		.active_low		= 1,
		.code			= KEY_SPACE,
		.debounce_interval	= 5,
	},
	/* Y button */ {
		.gpio			= JZ_GPIO_PORTE(11),
		.active_low		= 1,
		.code			= KEY_LEFTSHIFT,
		.debounce_interval	= 5,
	},
	/* Left shoulder button */ {
		.gpio			= JZ_GPIO_PORTE(10),
		.active_low		= 1,
		.code			= KEY_TAB,
		.debounce_interval	= 5,
	},
	/* Right shoulder button */ {
		.gpio			= JZ_GPIO_PORTE(7),
		.active_low		= 1,
		.code			= KEY_BACKSPACE,
		.debounce_interval	= 5,
	},
	/* START button */ {
		.gpio			= JZ_GPIO_PORTD(21),
		.active_low		= 1,
		.code			= KEY_ENTER,
		.debounce_interval	= 5,
	},
	/* SELECT button */ {
		.gpio			= JZ_GPIO_PORTE(8),
		.active_low		= 1,
		.code			= KEY_ESC,
		.debounce_interval	= 5,
	},
	/* POWER slider */ {
		.gpio			= JZ_GPIO_PORTE(30),
		.active_low		= 1,
		.code			= KEY_POWER,
		.wakeup			= 1,
	},
	/* POWER hold */ {
		.gpio			= JZ_GPIO_PORTD(18),
		.active_low		= 1,
		.code			= KEY_PAUSE,
	},
};

static struct gpio_keys_platform_data a380_gpio_keys_pdata = {
	.buttons = a380_buttons,
	.nbuttons = ARRAY_SIZE(a380_buttons),
	.rep = 1,
};

static struct platform_device a380_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &a380_gpio_keys_pdata,
	},
};

/* LCD backlight */
static struct platform_pwm_backlight_data a380_backlight_pdata = {
	.pwm_id = 2,
	.max_brightness = 255,
	.dft_brightness = 145,
	.pwm_period_ns = 40000, /* 25 kHz: outside human hearing range */
};

static struct platform_device a380_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
	.dev = {
		.platform_data = &a380_backlight_pdata,
	},
};

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_lcd_device,
	&jz_udc_device,
	//&jz_i2c_device,
	&jz_msc0_device,
	&jz_msc1_device,
	&jz_pcm_device,
	&jz_i2s_device,
	&jz_codec_device,
	&a380_gpio_keys_device,
	&a380_backlight_device,
};

static int __init a380_init_platform_devices(void)
{
	jz_msc0_device.dev.platform_data = &cetus_sd_data;
	jz_msc1_device.dev.platform_data = &cetus_tf_data;

	jz4750d_serial_device_register();

	return platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
}

static void __init board_gpio_setup(void)
{
	int i;

	__gpio_as_output(GPIO_AMPEN);
	__gpio_clear_pin(GPIO_AMPEN);
#ifdef GPIO_CHARGE
	__gpio_as_output(GPIO_CHARGE);
	__gpio_set_pin(GPIO_CHARGE);
#endif

	/* Set up gpios for keys */
	for (i = 0; i < ARRAY_SIZE(a380_buttons); i++) {
		__gpio_as_input(a380_buttons[i].gpio);
		if (a380_buttons[i].gpio == JZ_GPIO_PORTC(31))
			__gpio_disable_pull(a380_buttons[i].gpio);
		else
			__gpio_enable_pull(a380_buttons[i].gpio);
	}
}

static int __init jz_board_setup(void)
{
	printk("Dingoo A380 board setup %s %s\n",__DATE__,__TIME__);

	board_gpio_setup();

	if (a380_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}

arch_initcall(jz_board_setup);

/*
 * linux/arch/mips/jz4750d/board-a320e.c
 *
 * Dingoo A320E board setup routines.
 *
 * Copyright (c) 2006-2008  Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
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

#include <asm/mach-jz4750d/board-a320e.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>

#include <asm/mach-jz4750d/irq.h>
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

static void a320e_sd_gpio_init(struct device *dev)
{
	__gpio_as_msc0_4bit();
}

static void a320e_sd_power_on(struct device *dev)
{
}

static void a320e_sd_power_off(struct device *dev)
{
}

static unsigned int a320e_sd_status(struct device *dev)
{
	return 1;
}

static void a320e_sd_plug_change(int state)
{
}

static unsigned int a320e_sd_get_wp(struct device *dev)
{
	return 0;
}

struct jz_mmc_platform_data a320e_sd_data = {
	.support_sdio   = 1,
	.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq     = 0,
	.detect_pin     = 0,
	.nonremovable   = 1,
	.init           = a320e_sd_gpio_init,
	.power_on       = a320e_sd_power_on,
	.power_off      = a320e_sd_power_off,
	.status         = a320e_sd_status,
	.plug_change    = a320e_sd_plug_change,
	.write_protect  = a320e_sd_get_wp,
	.max_bus_width  = MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
			  MMC_CAP_4_BIT_DATA,
	.bus_width      = 4, /* 1, 4 or 8 */
};

static void a320e_tf_gpio_init(struct device *dev)
{
	__gpio_as_msc1_1bit();
	__gpio_as_output(GPIO_SD1_VCC_EN_N);
	__gpio_as_input(GPIO_SD1_CD_N);
}

static void a320e_tf_power_on(struct device *dev)
{
	__msc1_enable_power();
}

static void a320e_tf_power_off(struct device *dev)
{
	__msc1_disable_power();
}

static unsigned int a320e_tf_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) __gpio_get_pin(GPIO_SD1_CD_N);
	return (!status);
}

static void a320e_tf_plug_change(int state)
{
	if(state == CARD_INSERTED)
		__gpio_as_irq_fall_edge(MSC1_HOTPLUG_PIN);
	else
		__gpio_as_irq_rise_edge(MSC1_HOTPLUG_PIN);
}

struct jz_mmc_platform_data a320e_tf_data = {
	.support_sdio   = 1,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.status_irq	= JZ4750D_IRQ_GPIO(GPIO_SD1_CD_N),
	.detect_pin     = GPIO_SD1_CD_N,
	.init           = a320e_tf_gpio_init,
	.power_on       = a320e_tf_power_on,
	.power_off      = a320e_tf_power_off,
	.status		= a320e_tf_status,
	.plug_change	= a320e_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.bus_width      = 1,
};

/* Keys mapped to gpio */
static struct gpio_keys_button a320e_buttons[] = {
	/* D-pad up */ {
		.gpio			= JZ_GPIO_PORTD(26),
		.active_low		= 1,
		.code			= KEY_UP,
		.debounce_interval	= 5,
	},
	/* D-pad down */ {
		.gpio			= JZ_GPIO_PORTE(19),
		.active_low		= 1,
		.code			= KEY_DOWN,
		.debounce_interval	= 5,
	},
	/* D-pad left */ {
		.gpio			= JZ_GPIO_PORTD(27),
		.active_low		= 1,
		.code			= KEY_LEFT,
		.debounce_interval	= 5,
	},
	/* D-pad right */ {
		.gpio			= JZ_GPIO_PORTE(18),
		.active_low		= 1,
		.code			= KEY_RIGHT,
		.debounce_interval	= 5,
	},
	/* A button */ {
		.gpio			= JZ_GPIO_PORTD(17),
		.active_low		= 1,
		.code			= KEY_LEFTCTRL,
		.debounce_interval	= 5,
	},
	/* B button */ {
		.gpio			= JZ_GPIO_PORTE(0),
		.active_low		= 1,
		.code			= KEY_LEFTALT,
		.debounce_interval	= 5,
	},
	/* X button */ {
		.gpio			= JZ_GPIO_PORTC(23),
		.active_low		= 1,
		.code			= KEY_SPACE,
		.debounce_interval	= 5,
	},
	/* Y button */ {
		.gpio			= JZ_GPIO_PORTC(24),
		.active_low		= 1,
		.code			= KEY_LEFTSHIFT,
		.debounce_interval	= 5,
	},
	/* Left shoulder button */ {
		.gpio			= JZ_GPIO_PORTD(25),
		.active_low		= 1,
		.code			= KEY_TAB,
		.debounce_interval	= 5,
	},
	/* Right shoulder button */ {
		.gpio			= JZ_GPIO_PORTD(16),
		.active_low		= 1,
		.code			= KEY_BACKSPACE,
		.debounce_interval	= 5,
	},
	/* START button */ {
		.gpio			= JZ_GPIO_PORTE(11),
		.active_low		= 1,
		.code			= KEY_ENTER,
		.debounce_interval	= 5,
	},
	/* SELECT button */ {
		.gpio			= JZ_GPIO_PORTE(10),
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
		.gpio			= JZ_GPIO_PORTD(24),
		.active_low		= 1,
		.code			= KEY_PAUSE,
	},
};

static struct gpio_keys_platform_data a320e_gpio_keys_pdata = {
	.buttons = a320e_buttons,
	.nbuttons = ARRAY_SIZE(a320e_buttons),
	.rep = 1,
};

static struct platform_device a320e_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &a320e_gpio_keys_pdata,
	},
};

/* LCD backlight */
static struct platform_pwm_backlight_data a320e_backlight_pdata = {
	.pwm_id = 2,
	.max_brightness = 255,
	.dft_brightness = 145,
	.pwm_period_ns = 40000, /* 25 kHz: outside human hearing range */
};

static struct platform_device a320e_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
	.dev = {
		.platform_data = &a320e_backlight_pdata,
	},
};

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_lcd_device,
	&jz_udc_device,
	//&jz_msc0_device, /* A320E has no SD0 */
	&jz_msc1_device,
	&a320e_gpio_keys_device,
	&a320e_backlight_device,
};

static int __init a320e_init_platform_devices(void)
{
	jz_msc0_device.dev.platform_data = &a320e_sd_data;
	jz_msc1_device.dev.platform_data = &a320e_tf_data;

	jz4750d_serial_device_register();

	return platform_add_devices(jz_platform_devices,
			ARRAY_SIZE(jz_platform_devices));
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
	for (i = 0; i < ARRAY_SIZE(a320e_buttons); i++) {
		__gpio_as_input(a320e_buttons[i].gpio);
		__gpio_enable_pull(a320e_buttons[i].gpio);
	}
}

static int __init jz_board_setup(void)
{
	printk("Dingoo A320E board setup %s %s\n",__DATE__,__TIME__);

	board_gpio_setup();

	if (a320e_init_platform_devices())
		panic("Failed to initialize platform devices");

	return 0;
}

arch_initcall(jz_board_setup);

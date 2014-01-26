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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mmc/host.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/input/matrix_keypad.h>

#include <asm/mach-jz4750d/board-rzx50.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>

#include <asm/mach-jz4750d/jz4750d_mmc.h>
#include <asm/mach-jz4750d/gpio.h>
#include <asm/mach-jz4750d/platform.h>

#ifdef CONFIG_SOUND_JZ_I2S
#include <../sound/oss/jz_audio.h>
#endif

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
	.status         = cetus_tf_status,
	.plug_change    = cetus_tf_plug_change,
	.max_bus_width  = MMC_CAP_SD_HIGHSPEED | MMC_CAP_4_BIT_DATA,
	.bus_width      = 1,
};

#ifdef CONFIG_SOUND_JZ_I2S
static struct snd_endpoint snd_endpoints_list[] = {
	{
		.name	= "HANDSET",
		.id	= 0
	},
	{
		.name	= "SPEAKER",
		.id	= 1
	},
	{
		.name	= "HEADSET",
		.id	= 2
	},
};

static struct jz_snd_endpoints vogue_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

struct platform_device vogue_snd_device = {
	.name = "mixer",
	.id = -1,
	.dev = {
		.platform_data = &vogue_snd_endpoints,
	},
};
#endif

#ifndef CONFIG_KEYBOARD_RZX50_GPIO_KEYS
/* Keys mapped to gpio */
static struct gpio_keys_button rzx50_buttons[] = {
	/* Left shift */ {
		.gpio			= JZ_GPIO_PORTE(11),
		.active_low		= 1,
		.code			= KEY_TAB,
		.debounce_interval	= 5,
	},
#ifndef CONFIG_JZ_POWEROFF
	/* POWER slider */ {
		.gpio			= JZ_GPIO_PORTE(30),
		.active_low		= 1,
		.code			= KEY_POWER,
		.wakeup			= 1,
	},
#endif
	/* POWER hold */ {
		.gpio			= JZ_GPIO_PORTE(3),
		.active_low		= 1,
		.code			= KEY_PAUSE,
	},
};

static struct gpio_keys_platform_data rzx50_gpio_keys_pdata = {
	.buttons = rzx50_buttons,
	.nbuttons = ARRAY_SIZE(rzx50_buttons),
	.rep = 1,
};

static struct platform_device rzx50_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &rzx50_gpio_keys_pdata,
	},
};

/* Keys grouped in matrix */
static const uint32_t board_matrix_keys[] = {
	KEY(0, 0, KEY_UP),
	KEY(0, 1, KEY_LEFT),
	KEY(0, 2, KEY_RIGHT),
	KEY(0, 3, KEY_DOWN),

	KEY(1, 0, KEY_ENTER),		/* Start */
	KEY(1, 1, KEY_LEFTSHIFT),	/* Y */
	KEY(1, 2, KEY_LEFTALT),		/* B */
	KEY(1, 3, KEY_LEFTCTRL),	/* A */

	KEY(2, 0, KEY_RESERVED),	/* Not used */
	KEY(2, 1, KEY_SPACE),		/* X */
	KEY(2, 2, KEY_BACKSPACE),	/* Right Shift */
	KEY(2, 3, KEY_ESC),		/* Select */
};

static const struct matrix_keymap_data board_keymap_data = {
	.keymap			= board_matrix_keys,
	.keymap_size		= ARRAY_SIZE(board_matrix_keys),
};

static unsigned int board_keypad_row_gpios[] = {
	JZ_GPIO_PORTD(19),
	JZ_GPIO_PORTE(7),
	JZ_GPIO_PORTE(8),
};

static unsigned int board_keypad_col_gpios[] = {
	JZ_GPIO_PORTE(18),
	JZ_GPIO_PORTF(10),
	JZ_GPIO_PORTF(11),
	JZ_GPIO_PORTF(12),
};

static struct matrix_keypad_platform_data board_keypad_platform_data = {
	.keymap_data	= &board_keymap_data,
	.row_gpios	= board_keypad_row_gpios,
	.num_row_gpios	= ARRAY_SIZE(board_keypad_row_gpios),
	.col_gpios	= board_keypad_col_gpios,
	.num_col_gpios	= ARRAY_SIZE(board_keypad_col_gpios),
	.active_low	= 1,

	.debounce_ms		= 5,
	.col_scan_delay_us	= 20,
};

static struct platform_device board_keyboard = {
	.name	= "matrix-keypad",
	.id	= -1,
	.dev	= {
		.platform_data = &board_keypad_platform_data,
	},
};
#endif

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
	&jz_lcd_device,
	&jz_usb_gdt_device,
	//&jz_i2c_device,
#ifdef CONFIG_SOUND_JZ_I2S
	&vogue_snd_device,
#endif
	&jz_msc0_device,
	&jz_msc1_device,
#ifndef CONFIG_KEYBOARD_RZX50_GPIO_KEYS
	&rzx50_gpio_keys_device,
	&board_keyboard,
#endif
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

/*
 *  Copyright (C) 2013-2014, Dmitry Smagin <dmitry.s.smagin@gmail.com>
 *  Copyright (C) 2012, Antony Pavlov <antonynpavlov@gmail.com>
 *  JZ4750D platform devices
 *
 *  based on JZ4740 platform devices
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#include <linux/dma-mapping.h>

#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/mach-jz4750d/jz4750d_intc.h>

#include <asm/mach-jz4750d/platform.h>
#include <asm/mach-jz4750d/base.h>
#include <asm/mach-jz4750d/dma.h>
#include <asm/mach-jz4750d/irq.h>

#include "serial.h"
#include "clock.h"

/* UDC (USB gadget controller) */
static struct resource jz_usb_gdt_resources[] = {
	{
		.start		= JZ4750D_UDC_BASE_ADDR,
		.end		= JZ4750D_UDC_BASE_ADDR + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= JZ4750D_IRQ_UDC,
		.end		= JZ4750D_IRQ_UDC,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device jz_udc_device = {
	.name		= "jz-udc",
	.id		= 0,
	.dev = {
		.dma_mask = &jz_udc_device.dev.coherent_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(jz_usb_gdt_resources),
	.resource	= jz_usb_gdt_resources,
};

/* MMC/SD controller MSC0 */
static struct resource jz_msc0_resources[] = {
	{
		.start          = JZ4750D_MSC0_BASE_ADDR,
		.end            = JZ4750D_MSC0_BASE_ADDR + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = JZ4750D_IRQ_MSC0,
		.end            = JZ4750D_IRQ_MSC0,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC0_RX,
		.end            = DMA_ID_MSC0_TX,
		.flags          = IORESOURCE_DMA,
	},
};

struct platform_device jz_msc0_device = {
	.name = "jz-msc",
	.id = 0,
	.dev = {
		.dma_mask  = &jz_msc0_device.dev.coherent_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources  = ARRAY_SIZE(jz_msc0_resources),
	.resource       = jz_msc0_resources,
};

/* MMC/SD controller MSC1 */
static struct resource jz_msc1_resources[] = {
	{
		.start          = JZ4750D_MSC1_BASE_ADDR,
		.end            = JZ4750D_MSC1_BASE_ADDR + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = JZ4750D_IRQ_MSC1,
		.end            = JZ4750D_IRQ_MSC1,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC1_RX,
		.end            = DMA_ID_MSC1_TX,
		.flags          = IORESOURCE_DMA,
	},

};

struct platform_device jz_msc1_device = {
	.name = "jz-msc",
	.id = 1,
	.dev = {
		.dma_mask = &jz_msc1_device.dev.coherent_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources  = ARRAY_SIZE(jz_msc1_resources),
	.resource       = jz_msc1_resources,
};

/* RTC controller */
static struct resource jz_rtc_resources[] = {
	{
		.start	= JZ4750D_RTC_BASE_ADDR,
		.end	= JZ4750D_RTC_BASE_ADDR + 0x38 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start  = JZ4750D_IRQ_RTC,
		.end	= JZ4750D_IRQ_RTC,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device jz_rtc_device = {
	.name		= "jz4750-rtc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_rtc_resources),
	.resource	= jz_rtc_resources,
};

/* I2C controller */
static struct resource jz_i2c_resources[] = {
	{
		.start          = JZ4750D_I2C_BASE_ADDR,
		.end            = JZ4750D_I2C_BASE_ADDR + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = JZ4750D_IRQ_I2C,
		.end            = JZ4750D_IRQ_I2C,
		.flags          = IORESOURCE_IRQ,
	}
};

struct platform_device jz_i2c_device = {
	.name = "jz47xx-i2c",
	.id = 0,
	.dev = {
		.dma_mask = &jz_i2c_device.dev.coherent_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources  = ARRAY_SIZE(jz_i2c_resources),
	.resource       = jz_i2c_resources,
};

/* LCD controller */
static struct resource jz_lcd_resources[] = {
	{
		.start          = JZ4750D_LCD_BASE_ADDR,
		.end            = JZ4750D_LCD_BASE_ADDR + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = JZ4750D_IRQ_LCD,
		.end            = JZ4750D_IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

struct platform_device jz_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask = &jz_lcd_device.dev.coherent_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* I2S controller */
static struct resource jz_i2s_resources[] = {
	{
		.start	= JZ4750D_AIC_BASE_ADDR,
		.end	= JZ4750D_AIC_BASE_ADDR + 0x38 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz_i2s_device = {
	.name		= "jz4750-i2s",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_i2s_resources),
	.resource	= jz_i2s_resources,
};

/* PCM */
struct platform_device jz_pcm_device = {
	.name		= "jz4750-pcm", /* jz2750-audio */
	.id		= -1,
};

/* Codec */
static struct resource jz_codec_resources[] = {
	{
		.start	= JZ4750D_AIC_BASE_ADDR + 0x80,
		.end	= JZ4750D_AIC_BASE_ADDR + 0x88 - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device jz_codec_device = {
	.name		= "jz4750-codec", /* JZDLV */
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_codec_resources),
	.resource	= jz_codec_resources,
};

/* ADC controller */
static struct resource jz_adc_resources[] = {
	{
		.start	= JZ4750D_SADC_BASE_ADDR,
		.end	= JZ4750D_SADC_BASE_ADDR + 0x30,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= JZ4750D_IRQ_SADC,
		.end	= JZ4750D_IRQ_SADC,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= JZ4750D_IRQ_ADC_BASE,
		.end	= JZ4750D_IRQ_ADC_BASE,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device jz_adc_device = {
	.name		= "jz4750-adc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(jz_adc_resources),
	.resource	= jz_adc_resources,
};

/* Watchdog */
static struct resource jz_wdt_resources[] = {
	{
		.start = JZ4750D_WDT_BASE_ADDR,
		.end   = JZ4750D_WDT_BASE_ADDR + 0x10 - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device jz_wdt_device = {
	.name          = "jz4750-wdt",
	.id            = -1,
	.num_resources = ARRAY_SIZE(jz_wdt_resources),
	.resource      = jz_wdt_resources,
};

/* Serial */
#define JZ4750D_UART_DATA(_id) \
	{ \
		.flags = UPF_SKIP_TEST | UPF_IOREMAP | UPF_FIXED_TYPE, \
		.iotype = UPIO_MEM, \
		.regshift = 2, \
		.serial_out = jz4750d_serial_out, \
		.type = PORT_16550, \
		.mapbase = JZ4750D_UART ## _id ## _BASE_ADDR, \
		.irq = JZ4750D_IRQ_UART ## _id, \
	}

static struct plat_serial8250_port jz4750d_uart_data[] = {
	JZ4750D_UART_DATA(0),
	JZ4750D_UART_DATA(1),
	JZ4750D_UART_DATA(2),
	{},
};

static struct platform_device jz4750d_uart_device = {
	.name = "serial8250",
	.id = 0,
	.dev = {
		.platform_data = jz4750d_uart_data,
	},
};

#define JZ_REG_CLOCK_CTRL	0x00
#define JZ_CLOCK_CTRL_ECS	BIT(30)

void jz4750d_serial_device_register(void)
{
	void __iomem *cpm_base = ioremap(JZ4750D_CPM_BASE_ADDR, 0x100);
	struct plat_serial8250_port *p;
	int uart_rate;

	uart_rate = jz4750d_clock_bdata.ext_rate;

	/*
	 * ECS bit selects the clock source between EXCLK and EXCLK/2 output
	 * This bit is only used to APB device such as UART I2S I2C SSI SADC
	 * UDC_PHY etc.
	 */

	if (readl(cpm_base + JZ_REG_CLOCK_CTRL) & JZ_CLOCK_CTRL_ECS) {
		uart_rate >>= 1;
	}

	for (p = jz4750d_uart_data; p->flags != 0; ++p)
		p->uartclk = uart_rate;

	platform_device_register(&jz4750d_uart_device);
}

/*
 * Platform device support for Jz4740 SoC.
 *
 * Copyright 2007, <yliu@ingenic.cn>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#include <asm/jzsoc.h>
#include <asm/mach-jz4750d/platform.h>

/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(LCD_BASE),
		.end            = CPHYSADDR(LCD_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

struct platform_device jz_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_lcd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* UDC (USB gadget controller) */
static struct resource jz_usb_gdt_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UDC,
		.end		= IRQ_UDC,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64 udc_dmamask = ~(u32)0;

struct platform_device jz_usb_gdt_device = {
	.name		= "jz-udc",
	.id		= 0,
	.dev = {
		.dma_mask		= &udc_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_gdt_resources),
	.resource	= jz_usb_gdt_resources,
};

/** MMC/SD controller MSC0**/
static struct resource jz_msc0_resources[] = {
	{
		.start          = CPHYSADDR(MSC_BASE),
		.end            = CPHYSADDR(MSC_BASE) + 0x1000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC0,
		.end            = IRQ_MSC0,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC0_RX,
		.end            = DMA_ID_MSC0_TX,
		.flags          = IORESOURCE_DMA,
	},
};

static u64 jz_msc0_dmamask =  ~(u32)0;

struct platform_device jz_msc0_device = {
	.name = "jz-msc",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_msc0_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc0_resources),
	.resource       = jz_msc0_resources,
};

/** MMC/SD controller MSC1**/
static struct resource jz_msc1_resources[] = {
	{
		.start          = CPHYSADDR(MSC_BASE) + 0x1000,
		.end            = CPHYSADDR(MSC_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = IRQ_MSC1,
		.end            = IRQ_MSC1,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = DMA_ID_MSC1_RX,
		.end            = DMA_ID_MSC1_TX,
		.flags          = IORESOURCE_DMA,
	},

};

static u64 jz_msc1_dmamask =  ~(u32)0;

struct platform_device jz_msc1_device = {
	.name = "jz-msc",
	.id = 1,
	.dev = {
		.dma_mask               = &jz_msc1_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_msc1_resources),
	.resource       = jz_msc1_resources,
};

/** I2C controller **/
#if 0
static struct resource jz_i2c_resources[] = {
	[0] = {
		.start          = CPHYSADDR(I2C_BASE),
		.end            = CPHYSADDR(I2C_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
#if 0
	[1] = {
		.start          = IRQ_I2C,
		.end            = IRQ_I2C,
		.flags          = IORESOURCE_IRQ,
	}
#endif
};

static u64 jz_i2c_dmamask =  ~(u32)0;

struct platform_device jz_i2c_device = {
	.name = "jz_i2c",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_i2c_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_i2c_resources),
	.resource       = jz_i2c_resources,
};
#endif


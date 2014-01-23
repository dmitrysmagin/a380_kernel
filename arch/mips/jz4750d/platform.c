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
#include <linux/serial_core.h>
#include <linux/serial_8250.h>

#include <asm/jzsoc.h>
#include <asm/mach-jz4750d/platform.h>
#include <asm/mach-jz4750d/base.h>
//#include <asm/mach-jz4750d/irq.h>

#include "serial.h"
#include "clock.h"

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

/*** UDC (USB gadget controller) ***/
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

/*** MMC/SD controller MSC0 ***/
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

/*** MMC/SD controller MSC1 ***/
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

/*** I2C controller ***/
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

/* Serial */
#define JZ4750D_IRQ_UART0 IRQ_UART0
#define JZ4750D_IRQ_UART1 IRQ_UART1
#define JZ4750D_IRQ_UART2 IRQ_UART2

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
	 * This bit is only used to APB device such as UART I2S I2C SSI SADC UDC_PHY etc.
	 */

	if (readl(cpm_base + JZ_REG_CLOCK_CTRL) & JZ_CLOCK_CTRL_ECS) {
		uart_rate >>= 1;
	}

	for (p = jz4750d_uart_data; p->flags != 0; ++p)
		p->uartclk = uart_rate;

	platform_device_register(&jz4750d_uart_device);
}

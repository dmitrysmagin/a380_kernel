/*
 *  Copyright (C) 2012, Antony Pavlov <antonynpavlov@gmail.com>
 *  JZ4750D IRQ definitions
 *
 *  based on JZ4740 IRQ definitions
 *  Copyright (C) 2009-2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or (at your
 *  option) any later version.
 *
 */

#ifndef __ASM_MACH_JZ4750D_IRQ_H__
#define __ASM_MACH_JZ4750D_IRQ_H__

/*
 * Note: First 32 interrupt numbers coincide with respective bits
 * in ICSR register (0x10001000)
 *
 */

#define MIPS_CPU_IRQ_BASE 0
#define JZ4750D_IRQ_BASE 0

#define JZ4750D_IRQ(x)		(JZ4750D_IRQ_BASE + (x))

/* 1st-level interrupts */

#define JZ4750D_IRQ_ETH		JZ4750D_IRQ(0)
#define JZ4750D_IRQ_SFT		JZ4750D_IRQ(4)
#define JZ4750D_IRQ_I2C		JZ4750D_IRQ(5)
#define JZ4750D_IRQ_RTC		JZ4750D_IRQ(6)
#define JZ4750D_IRQ_UART2	JZ4750D_IRQ(7)
#define JZ4750D_IRQ_UART1	JZ4750D_IRQ(8)
#define JZ4750D_IRQ_UART0	JZ4750D_IRQ(9)
#define JZ4750D_IRQ_AIC		JZ4750D_IRQ(10)
#define JZ4750D_IRQ_GPIO5	JZ4750D_IRQ(11)
#define JZ4750D_IRQ_GPIO4	JZ4750D_IRQ(12)
#define JZ4750D_IRQ_GPIO3	JZ4750D_IRQ(13)
#define JZ4750D_IRQ_GPIO2	JZ4750D_IRQ(14)
#define JZ4750D_IRQ_GPIO1	JZ4750D_IRQ(15)
#define JZ4750D_IRQ_GPIO0	JZ4750D_IRQ(16)
#define JZ4750D_IRQ_BCH		JZ4750D_IRQ(17)
#define JZ4750D_IRQ_SADC	JZ4750D_IRQ(18)
#define JZ4750D_IRQ_CIM		JZ4750D_IRQ(19)
#define JZ4750D_IRQ_TSSI	JZ4750D_IRQ(20)
#define JZ4750D_IRQ_TCU2	JZ4750D_IRQ(21)
#define JZ4750D_IRQ_TCU1	JZ4750D_IRQ(22)
#define JZ4750D_IRQ_TCU0	JZ4750D_IRQ(23)
#define JZ4750D_IRQ_MSC1	JZ4750D_IRQ(24)
#define JZ4750D_IRQ_MSC0	JZ4750D_IRQ(25)
#define JZ4750D_IRQ_SSI		JZ4750D_IRQ(26)
#define JZ4750D_IRQ_UDC		JZ4750D_IRQ(27)
#define JZ4750D_IRQ_DMAC1	JZ4750D_IRQ(28)
#define JZ4750D_IRQ_DMAC0	JZ4750D_IRQ(29)
#define JZ4750D_IRQ_IPU		JZ4750D_IRQ(30)
#define JZ4750D_IRQ_LCD		JZ4750D_IRQ(31)

/* 2nd-level interrupts */

/* 32 to 43 for DMAC0's 0-5  and DMAC1's 0-5 */
#define JZ4750D_IRQ_DMA(x)	(JZ4750D_IRQ(32) + (x))

/* 48 to 240 for GPIO pin 0 to 192 */
#define JZ4750D_IRQ_INTC_GPIO(x) (JZ4750D_IRQ_GPIO0 - (x))
#define JZ4750D_IRQ_GPIO(x)	(JZ4750D_IRQ(48) + (x))

#define NR_IRQS (256)

#endif

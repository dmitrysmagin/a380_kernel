/*
 *  linux/include/asm-mips/mach-jz4750d/board-320e.h
 *
 *  JZ4750D-based 320E board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750D_320E_H__
#define __ASM_JZ4750D_320E_H__

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		24000000
#define JZ_EXTAL2		32768     /* RTC extal freq: 32.768 KHz */

/*======================================================================
 * GPIO
 */

#define GPIO_SD0_VCC_EN_N	(0xff) /* CIM_D0 */
#define GPIO_SD0_CD_N		(0xff) /* CIM_D1 */
#define GPIO_SD0_WP		(0xff) /* CIM_D2 */
#define GPIO_SD1_VCC_EN_N	(0xff) /* CIM_D3 */
#define GPIO_SD1_CD_N		(32*4+8) /* CIM_D4 */

#define GPIO_USB_DETE		(32*4+9) /* CIM_D6 */
#define GPIO_DC_DETE_N		(0xff) /* CIM_MCLK */
#define GPIO_CHARG_STAT_N	(32*3+23) /* CIM_VSYNC */

#define GPIO_AMPEN		(32*4+5) /* GPE5 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

/*======================================================================
 * The GPIO interrupt pin is low voltage or fall edge acitve
 */
#define ACTIVE_LOW_MSC0_CD	1 /* work when GPIO_SD0_CD_N = 0 */
#define ACTIVE_LOW_MSC1_CD	1 /* work when GPIO_SD1_CD_N = 0 */

/*======================================================================
 * MMC/SD
 */

#define MSC0_WP_PIN		GPIO_SD0_WP
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_WP_PIN		GPIO_SD1_WP
#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

#define __msc0_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD0_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD0_CD_N);		\
} while (0)

#define __msc0_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD0_VCC_EN_N);	\
} while (0)

#define __msc0_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD0_VCC_EN_N);	\
} while (0)

#if ACTIVE_LOW_MSC0_CD == 1 /* work when cd is low */
#define __msc0_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 0;			\
	detected;				\
})
#else
#define __msc0_card_detected(s)			\
({						\
	int detected = 0;			\
	if (__gpio_get_pin(GPIO_SD0_CD_N))	\
		detected = 1;			\
	detected;				\
})
#endif /*ACTIVE_LOW_MSC0_CD*/

#define __msc1_init_io()			\
do {						\
	__gpio_as_output(GPIO_SD1_VCC_EN_N);	\
	__gpio_as_input(GPIO_SD1_CD_N);	\
} while (0)

#define __msc1_enable_power()			\
do {						\
	__gpio_clear_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#define __msc1_disable_power()			\
do {						\
	__gpio_set_pin(GPIO_SD1_VCC_EN_N);	\
} while (0)

#if ACTIVE_LOW_MSC1_CD == 1 /* work when cd is low */
#define __msc1_card_detected(s)			\
({						\
	int detected = 1;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
		detected = 0;			\
	detected;				\
})
#else
#define __msc1_card_detected(s)			\
({						\
	int detected = 0;			\
	if (__gpio_get_pin(GPIO_SD1_CD_N))	\
		detected = 1;			\
	detected;				\
})
#endif /*ACTIVE_LOW_MSC1_CD*/


/*======================================================================
 * SPI
 */
#define GPIO_SSI0_CE0	(32*1+29)
#define GPIO_SSI0_CE1	(32*1+31)
#define SSI0_GPC_PIN	(32*1+30)
#define GPI1_SSI0_CE0	(32*1+29)/* same as SSI0, for avoiding compilation error and ... */
#define GPI1_SSI0_CE1	(32*1+31)

#define	 SPI_CHIPSELECT_NUM_A		GPIO_SSI0_CE0
#define	 SPI_CHIPSELECT_NUM_B		GPIO_SSI0_CE1
#define	 SPI_CHIPSELECT_NUM_C		GPIO_SSI1_CE0

#define  SPI0_BUS		0
#define  SPI1_BUS		1

#endif /* __ASM_JZ4750d_320E_H__ */

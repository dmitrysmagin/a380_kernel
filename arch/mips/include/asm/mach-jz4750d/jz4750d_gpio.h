/*
 * linux/include/asm-mips/mach-jz4750d/jz4750d_gpio.h
 *
 * JZ4750D GPIO definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750D_GPIO_H__
#define __ASM_JZ4750D_GPIO_H__

#include <asm/mach-jz4750d/jz4750d_regs.h>

#define GPIO_BASE	0xB0010000

//------------------------------------------------------
// GPIO Pins Description
//
// PORT 0:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
//	0		D0		-
//	1		D1		-
//	2		D2		-
//	3		D3		-
//	4		D4		-
//	5		D5		-
//	6		D6		-
//	7		D7		-
//	8		D8		-
//	9		D9		-
//	10		D10		-
//	11		D11		-
//	12		D12		-
//	13		D13		-
//	14		D14		-
//	15		D15		-
//	16		D16		-
//	17		D17		-
//	18		D18		-
//	19		D19		-
//	20		D20		-
//	21		D21		-
//	22		D22		-
//	23		D23		-
//	24		D24		-
//	25		D25		-
//	26		D26		-
//	27		D27		-
//	28		D28		-
//	29		D29		-
//	30		D30		-
//	31		D31		-
//
//------------------------------------------------------
// PORT 1:
//
// PIN/BIT N		FUNC0		FUNC1	       FUNC2         NOTE
//	0		A0		-              -
//	1		A1		-              -
//	2		A2		-              -
//	3		A3		-              -
//	4		A4		-              -
//	5		A5		-              -
//	6		A6		-              -
//	7		A7		-              -
//	8		A8		-              -
//	9		A9		-              -
//	10		A10		-              -
//	11		A11		-              -
//	12		A12		-              -
//	13		A13		-              -
//	14		A14		-              -
//	15		A15/CLE		CL(unshare)    MSC0_CLK
//	16		DCS0#		-              -
//	17		RAS#		-              -
//	18		CAS#		-              -
//	19		SDWE#/BUFD#	-              -
//	20		WE0#		-              -
//	21		WE1#		-              -
//	22		WE2#		-              -
//	23		WE3#		-              -
//	24		CKO		-	       -              Note1
//	25		CKE		-              -
//	26		SSI_CLK	        MSC1_CLK       -
//	27		SSI_DT		MSC1_D1        -
//	28		SSI_DR		MSC1_D0        -
//	29		SSI_CE0#	MSC1_CMD       -
//	30		SSI_GPC	        MSC1_D2        -
//	31		SSI_CE1#	MSC1_D3        -
//
// Note1: BIT24: it is CKO when chip is reset
//
//------------------------------------------------------
// PORT 2:
//
// PIN/BIT N		FUNC0		FUNC1		FUNC2         NOTE
//	0		SD0		A20             -
//	1		SD1		A21             -
//	2		SD2		A22             -
//	3		SD3		A23             -
//	4		SD4		A24             -
//	5		SD5		A25             -
//	6		SD6		-               -
//	7		SD7		-               -
//	8		SD8		TSDI0           -
//	9		SD9		TSDI1           -
//	10		SD10		TSDI2           -
//	11		SD11		TSDI3           -
//	12		SD12		TSDI4           -
//	13		SD13		TSDI5           -
//	14		SD14		TSDI6           -
//	15		SD15		TSDI7           -
//	16		A16/ALE		AL(unshare)     MSC0_CMD
//	17		A17             MSC0_D3         -
//	18		A18             DREQ            -
//	19		A19             DACK            -
//	20		WAIT#		-		-             Note2
//	21		CS1#		-               - 
//	22		CS2#		-               -
//	23		CS3#		-               -
//	24		CS4#		-               -
//	25		RD#		-               -
//	26		WR#		-               -
//	27		FRB#		-		-             Note3
//	28		FRE#		MSC0_D0         -
//	29		FWE#		MSC0_D1         -
//	30		-       	-		-             Note4
//	31		-       	-		-             Note5
//
// Note2: BIT20: it is WIAT# pin when chip is reset
//
// Note3: BIT27: when NAND is used, it should connect to NANF FRB#.
//
// Note4: BIT30: it is BOOT_SEL0 which would be set as input without pulling when chip is reset.
//
// Note5: BIT31: it is BOOT_SEL1 which would be set as input without pulling when chip is reset.
//
//------------------------------------------------------
// PORT 3:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
//	0		LCD_B2		-
//	1		LCD_B3		-
//	2		LCD_B4		-
//	3		LCD_B5		-
//	4		LCD_B6		-
//	5		LCD_B7		-
//	6		LCD_G2		-
//	7		LCD_G3		-
//	8		LCD_G4		-
//	9		LCD_G5		-
//	10		LCD_G6		-
//	11		LCD_G7		-
//	12		LCD_R2		-
//	13		LCD_R3		-
//	14		LCD_R4		-
//	15		LCD_R5		-
//	16		LCD_R6		-
//	17		LCD_R7		-
//	18		LCD_PCLK	-
//	19		LCD_HSYNC	-
//	20		LCD_VSYNC	-
//	21		LCD_DE		-
//	22		LCD_CLS		LCD_R1
//	23		LCD_SPL		LCD_G0
//	24		LCD_PS		LCD_G1
//	25		LCD_REV		LCD_B1
//	26		LCD_B0   	-
//	27		LCD_R0		-
//	28		UART0_RXD	TSCLK
//	29		UART0_TXD	TSSTR
//	30		UART0_CTS	TSFRM
//	31		UART0_RTS	TSFAIL
//
//------------------------------------------------------
// PORT 4:
//
// PIN/BIT N		FUNC0		FUNC1	       FUNC2         NOTE
//	0		CIM_D0		TSDI0          -
//	1		CIM_D1		TSDI1          -
//	2		CIM_D2		TSDI2          -
//	3		CIM_D3		TSDI3          -
//	4		CIM_D4		TSDI4          -
//	5		CIM_D5		TSDI5          -
//	6		CIM_D6		TSDI6          -
//	7		CIM_D7		TSDI7          -
//	8		CIM_MCLK	TSFAIL         -
//	9		CIM_PCLK	TSCLK          -
//	10		CIM_VSYNC	TSSTR          -
//	11		CIM_HSYNC	TSFRM          -
//	12		I2C_SDA		-              -
//	13		I2C_SCK		-              -
//	18		SDATO           -              -
//	19		SDATI           -              -
//	20		PWM0		-              -
//	22		PWM2		SYNC           -
//	23		PWM3		UART1_RxD      BCLK
//	24		PWM4		-              -
//	25		PWM5		UART1_TxD      SCLK_RSTN
//	28		DCS1#		-              -
//	29		-        	-              -              Note6
//	30		WKUP		-	       -              Note7
//	31		-		-	       -              Note8
//
// Note6: BIT29: it is BOOT_SEL2 which would be set as input without pulling when chip is reset.
// Note7: BIT30: it is only used as input and interrupt, and with no pull-up and pull-down
// Note8: BIT31: it is used to select the function of UART or JTAG set by PESEL[31]
//        PESEL[31] = 0, select JTAG function
//        PESEL[31] = 1, select UART function
//
//------------------------------------------------------
// PORT 5:
//
// PIN/BIT N		FUNC0		FUNC1		NOTE
//	10		SSI_CLK		-
//	11		SSI_DT		PWM1
//	12		SSI_DR		-
//	13		SSI_CE0#	-
//	14		SSI_GPC 	-
//	15		SSI_CE2#	-
//
//////////////////////////////////////////////////////////

/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
#define MAX_GPIO_NUM	192
#define GPIO_WAKEUP     (32 * 4 + 30)

//n = 0,1,2,3,4,5 (PORTA, PORTB, PORTC, PORTD, PORTE, PORTF)
#define GPIO_PXPIN(n)	(GPIO_BASE + (0x00 + (n)*0x100)) /* PIN Level Register */
#define GPIO_PXDAT(n)	(GPIO_BASE + (0x10 + (n)*0x100)) /* Port Data Register */
#define GPIO_PXDATS(n)	(GPIO_BASE + (0x14 + (n)*0x100)) /* Port Data Set Register */
#define GPIO_PXDATC(n)	(GPIO_BASE + (0x18 + (n)*0x100)) /* Port Data Clear Register */
#define GPIO_PXIM(n)	(GPIO_BASE + (0x20 + (n)*0x100)) /* Interrupt Mask Register */
#define GPIO_PXIMS(n)	(GPIO_BASE + (0x24 + (n)*0x100)) /* Interrupt Mask Set Reg */
#define GPIO_PXIMC(n)	(GPIO_BASE + (0x28 + (n)*0x100)) /* Interrupt Mask Clear Reg */
#define GPIO_PXPE(n)	(GPIO_BASE + (0x30 + (n)*0x100)) /* Pull Enable Register */
#define GPIO_PXPES(n)	(GPIO_BASE + (0x34 + (n)*0x100)) /* Pull Enable Set Reg. */
#define GPIO_PXPEC(n)	(GPIO_BASE + (0x38 + (n)*0x100)) /* Pull Enable Clear Reg. */
#define GPIO_PXFUN(n)	(GPIO_BASE + (0x40 + (n)*0x100)) /* Function Register */
#define GPIO_PXFUNS(n)	(GPIO_BASE + (0x44 + (n)*0x100)) /* Function Set Register */
#define GPIO_PXFUNC(n)	(GPIO_BASE + (0x48 + (n)*0x100)) /* Function Clear Register */
#define GPIO_PXSEL(n)	(GPIO_BASE + (0x50 + (n)*0x100)) /* Select Register */
#define GPIO_PXSELS(n)	(GPIO_BASE + (0x54 + (n)*0x100)) /* Select Set Register */
#define GPIO_PXSELC(n)	(GPIO_BASE + (0x58 + (n)*0x100)) /* Select Clear Register */
#define GPIO_PXDIR(n)	(GPIO_BASE + (0x60 + (n)*0x100)) /* Direction Register */
#define GPIO_PXDIRS(n)	(GPIO_BASE + (0x64 + (n)*0x100)) /* Direction Set Register */
#define GPIO_PXDIRC(n)	(GPIO_BASE + (0x68 + (n)*0x100)) /* Direction Clear Register */
#define GPIO_PXTRG(n)	(GPIO_BASE + (0x70 + (n)*0x100)) /* Trigger Register */
#define GPIO_PXTRGS(n)	(GPIO_BASE + (0x74 + (n)*0x100)) /* Trigger Set Register */
#define GPIO_PXTRGC(n)	(GPIO_BASE + (0x78 + (n)*0x100)) /* Trigger Set Register */
#define GPIO_PXFLG(n)	(GPIO_BASE + (0x80 + (n)*0x100)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_BASE + (0x14 + (n)*0x100)) /* Port Flag Clear Register */

#define REG_GPIO_PXPIN(n)	REG32(GPIO_PXPIN((n)))  /* PIN level */
#define REG_GPIO_PXDAT(n)	REG32(GPIO_PXDAT((n)))  /* 1: interrupt pending */
#define REG_GPIO_PXDATS(n)	REG32(GPIO_PXDATS((n)))
#define REG_GPIO_PXDATC(n)	REG32(GPIO_PXDATC((n)))
#define REG_GPIO_PXIM(n)	REG32(GPIO_PXIM((n)))   /* 1: mask pin interrupt */
#define REG_GPIO_PXIMS(n)	REG32(GPIO_PXIMS((n)))
#define REG_GPIO_PXIMC(n)	REG32(GPIO_PXIMC((n)))
#define REG_GPIO_PXPE(n)	REG32(GPIO_PXPE((n)))   /* 1: disable pull up/down */
#define REG_GPIO_PXPES(n)	REG32(GPIO_PXPES((n)))
#define REG_GPIO_PXPEC(n)	REG32(GPIO_PXPEC((n)))
#define REG_GPIO_PXFUN(n)	REG32(GPIO_PXFUN((n)))  /* 0:GPIO or intr, 1:FUNC */
#define REG_GPIO_PXFUNS(n)	REG32(GPIO_PXFUNS((n)))
#define REG_GPIO_PXFUNC(n)	REG32(GPIO_PXFUNC((n)))
#define REG_GPIO_PXSEL(n)	REG32(GPIO_PXSEL((n))) /* 0:GPIO/Fun0,1:intr/fun1*/
#define REG_GPIO_PXSELS(n)	REG32(GPIO_PXSELS((n)))
#define REG_GPIO_PXSELC(n)	REG32(GPIO_PXSELC((n)))
#define REG_GPIO_PXDIR(n)	REG32(GPIO_PXDIR((n))) /* 0:input/low-level-trig/falling-edge-trig, 1:output/high-level-trig/rising-edge-trig */
#define REG_GPIO_PXDIRS(n)	REG32(GPIO_PXDIRS((n)))
#define REG_GPIO_PXDIRC(n)	REG32(GPIO_PXDIRC((n)))
#define REG_GPIO_PXTRG(n)	REG32(GPIO_PXTRG((n))) /* 0:level-trigger, 1:edge-trigger */
#define REG_GPIO_PXTRGS(n)	REG32(GPIO_PXTRGS((n)))
#define REG_GPIO_PXTRGC(n)	REG32(GPIO_PXTRGC((n)))
#define REG_GPIO_PXFLG(n)	REG32(GPIO_PXFLG((n))) /* interrupt flag */
#define REG_GPIO_PXFLGC(n)	REG32(GPIO_PXFLGC((n))) /* interrupt flag */

/***************************************************************************
 * GPIO
 ***************************************************************************/
/*----------------------------------------------------------------
 * p is the port number (0,1,2,3,4,5)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-127), regardless of the port
 */

//-------------------------------------------
// Function Pins Mode

#define __gpio_as_func0(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXSELC(p) = (1 << o);		\
} while (0)

#define __gpio_as_func1(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
} while (0)

#define __gpio_as_func2(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXSELC(p) = (1 << o);		\
} while (0)

/*
 * D0 ~ D31, A0 ~ A14, DCS0#, RAS#, CAS#, 
 * RDWE#, WE0#, WE1#, WE2#, WE3#, CKO#, CKE#
 */
#define __gpio_as_sdram_32bit()			\
do {						\
	REG_GPIO_PXFUNS(0) = 0xffffffff;	\
	REG_GPIO_PXSELC(0) = 0xffffffff;	\
	REG_GPIO_PXPES(0) = 0xffffffff;		\
	REG_GPIO_PXFUNS(1) = 0x03ff7fff;	\
	REG_GPIO_PXSELC(1) = 0x03ff7fff;	\
	REG_GPIO_PXPES(1) = 0x03ff7fff;		\
} while (0)

/*
 * D0 ~ D31, A0 ~ A14, DCS0#, RAS#, CAS#, 
 * RDWE#, WE0#, WE1#, WE2#, WE3#, CKO#, CKE#
 * !!!!DCS1#
 */
#define __gpio_as_sdram_x2_32bit()		\
do {						\
	REG_GPIO_PXFUNS(0) = 0xffffffff;	\
	REG_GPIO_PXSELC(0) = 0xffffffff;	\
	REG_GPIO_PXPES(0) = 0xffffffff;		\
	REG_GPIO_PXFUNS(1) = 0x03ff7fff;	\
	REG_GPIO_PXSELC(1) = 0x03ff7fff;	\
	REG_GPIO_PXPES(1) = 0x03ff7fff;		\
	REG_GPIO_PXFUNS(4) = 0x10000000;	\
	REG_GPIO_PXSELC(4) = 0x10000000;	\
	REG_GPIO_PXPES(4) = 0x10000000;		\
} while (0)

/*
 * D0 ~ D15, A0 ~ A14, DCS0#, RAS#, CAS#, 
 * RDWE#, WE0#, WE1#, WE2#, WE3#, CKO#, CKE#
 */
#define __gpio_as_sdram_16bit()						\
do {								        \
		/* 32/16-bit data normal order */			\
	REG_GPIO_PXFUNS(0) = 0x0000ffff;				\
	REG_GPIO_PXSELC(0) = 0x0000ffff;				\
	REG_GPIO_PXPES(0) = 0x0000ffff;					\
	REG_GPIO_PXFUNS(1) = 0x03ff7fff;				\
	REG_GPIO_PXSELC(1) = 0x03ff7fff;				\
	REG_GPIO_PXPES(1) = 0x03ff7fff;					\
} while (0)

/*
 * D0 ~ D7, CS1#, CLE, ALE, FRE#, FWE#, FRB#, RDWE#/BUFD#
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nand_8bit(n)						\
do {		              						\
	/* 32/16-bit data bus */					\
	REG_GPIO_PXFUNS(0) = 0x000000ff; /* D0~D7 */			\
	REG_GPIO_PXSELC(0) = 0x000000ff;				\
	REG_GPIO_PXPES(0) = 0x000000ff;					\
	REG_GPIO_PXFUNS(1) = 0x00008000; /* CLE(A15) */			\
	REG_GPIO_PXSELC(1) = 0x00008000;				\
	REG_GPIO_PXPES(1) = 0x00008000;					\
	REG_GPIO_PXFUNS(2) = 0x00010000; /* ALE(A16) */			\
	REG_GPIO_PXSELC(2) = 0x00010000;				\
	REG_GPIO_PXPES(2) = 0x00010000;					\
									\
	REG_GPIO_PXFUNS(2) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXSELC(2) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPES(2) = 0x00200000 << ((n)-1);			\
									\
        REG_GPIO_PXFUNS(1) = 0x00080000; /* RDWE#/BUFD# */		\
        REG_GPIO_PXSELC(1) = 0x00080000;				\
	REG_GPIO_PXPES(1) = 0x00080000;					\
	REG_GPIO_PXFUNS(2) = 0x30000000; /* FRE#, FWE# */		\
	REG_GPIO_PXSELC(2) = 0x30000000;				\
	REG_GPIO_PXPES(2) = 0x30000000;					\
	REG_GPIO_PXFUNC(2) = 0x08000000; /* FRB#(input) */		\
	REG_GPIO_PXSELC(2) = 0x08000000;				\
	REG_GPIO_PXDIRC(2) = 0x08000000;				\
	REG_GPIO_PXPES(2) = 0x08000000;					\
} while (0)

/*
 * CS4#, RD#, WR#, WAIT#, A0 ~ A22, D0 ~ D7
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nor_8bit(n)						\
do {								        \
	/* 32/16-bit data bus */					\
	REG_GPIO_PXFUNS(0) = 0x000000ff;				\
	REG_GPIO_PXSELC(0) = 0x000000ff;				\
	REG_GPIO_PXPES(0) = 0x000000ff;					\
									\
	REG_GPIO_PXFUNS(2) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXSELC(2) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPES(2) = 0x00200000 << ((n)-1);			\
									\
	REG_GPIO_PXFUNS(1) = 0x0000ffff; /* A0~A15 */			\
	REG_GPIO_PXSELC(1) = 0x0000ffff;				\
	REG_GPIO_PXPES(1) = 0x0000ffff;					\
	REG_GPIO_PXFUNS(2) = 0x06110007; /* RD#, WR#, WAIT#, A20~A22 */	\
	REG_GPIO_PXSELC(2) = 0x06110007;				\
	REG_GPIO_PXPES(2) = 0x06110007;					\
	REG_GPIO_PXFUNS(2) = 0x000e0000; /* A17~A19 */	        	\
	REG_GPIO_PXSELS(2) = 0x000e0000;				\
	REG_GPIO_PXPES(2) = 0x000e0000;					\
} while (0)

/*
 * CS4#, RD#, WR#, WAIT#, A0 ~ A22, D0 ~ D15
 * @n: chip select number(1 ~ 4)
 */
#define __gpio_as_nor_16bit(n)						\
do {	               							\
	/* 32/16-bit data normal order */				\
	REG_GPIO_PXFUNS(0) = 0x0000ffff;				\
	REG_GPIO_PXSELC(0) = 0x0000ffff;				\
	REG_GPIO_PXPES(0) = 0x0000ffff;					\
									\
	REG_GPIO_PXFUNS(2) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXSELC(2) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPES(2) = 0x00200000 << ((n)-1);			\
									\
	REG_GPIO_PXFUNS(1) = 0x0000ffff; /* A0~A15 */			\
	REG_GPIO_PXSELC(1) = 0x0000ffff;				\
	REG_GPIO_PXPES(1) = 0x0000ffff;					\
	REG_GPIO_PXFUNS(2) = 0x06110007; /* RD#, WR#, WAIT#, A20~A22 */	\
	REG_GPIO_PXSELC(2) = 0x06110007;				\
	REG_GPIO_PXPES(2) = 0x06110007;					\
	REG_GPIO_PXFUNS(2) = 0x000e0000; /* A17~A19 */	        	\
	REG_GPIO_PXSELS(2) = 0x000e0000;				\
	REG_GPIO_PXPES(2) = 0x000e0000;					\
} while (0)

/*
 * UART0_TxD, UART0_RxD
 */
#define __gpio_as_uart0()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x30000000;	\
	REG_GPIO_PXSELC(3) = 0x30000000;	\
	REG_GPIO_PXPES(3) = 0x30000000;		\
} while (0)

/*
 * UART0_TxD, UART0_RxD, UART0_CTS, UART0_RTS
 */
#define __gpio_as_uart0_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(3) = 0xf0000000;	\
	REG_GPIO_PXSELC(3) = 0xf0000000;	\
	REG_GPIO_PXPES(3) = 0xf0000000;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD
 */
#define __gpio_as_uart1()			\
do {						\
	REG_GPIO_PXTRGC(4) = 0x02800000;	\
	REG_GPIO_PXFUNS(4) = 0x02800000;	\
	REG_GPIO_PXSELS(4) = 0x02800000;	\
	REG_GPIO_PXPES(4) = 0x02800000;		\
} while (0)

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0000ff00;	\
	REG_GPIO_PXSELS(2) = 0x0000ff00;	\
	REG_GPIO_PXPES(2) = 0x0000ff00;		\
	REG_GPIO_PXFUNS(3) = 0xf0000000;	\
	REG_GPIO_PXSELS(3) = 0xf0000000;	\
	REG_GPIO_PXPES(3) = 0xf0000000;		\
} while (0)

#define __gpio_as_slcd_8bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x001800ff;	\
	REG_GPIO_PXSELC(3) = 0x001800ff;	\
	REG_GPIO_PXTRGC(3) = 0x001800ff;	\
} while (0)

/*
 * LCD_D0~LCD_D7, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_8bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003c00ff;	\
	REG_GPIO_PXTRGC(3) = 0x003c00ff;	\
	REG_GPIO_PXSELC(3) = 0x003c00ff;	\
	REG_GPIO_PXPES(3) = 0x003c00ff;		\
} while (0)

/*
 * LCD_D0~LCD_D15, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_16bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003cffff;	\
	REG_GPIO_PXTRGC(3) = 0x003cffff;	\
	REG_GPIO_PXSELC(3) = 0x003cffff;	\
	REG_GPIO_PXPES(3) = 0x003cffff;		\
} while (0)

/*
 * LCD_R2~LCD_R7, LCD_G2~LCD_G7, LCD_B2~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_18bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x003fffff;	\
	REG_GPIO_PXTRGC(3) = 0x003fffff;	\
	REG_GPIO_PXSELC(3) = 0x003fffff;	\
	REG_GPIO_PXPES(3) = 0x003fffff;		\
} while (0)

/*
 * LCD_R0~LCD_R7, LCD_G0~LCD_G7, LCD_B0~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_24bit()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x0fffffff;	\
	REG_GPIO_PXTRGC(3) = 0x0fffffff;	\
	REG_GPIO_PXSELC(3) = 0x0c3fffff;	\
	REG_GPIO_PXSELS(3) = 0x03c00000;	\
	REG_GPIO_PXPES(3) = 0x0fffffff;		\
} while (0)

/*
 *  LCD_CLS, LCD_SPL, LCD_PS, LCD_REV
 */
#define __gpio_as_lcd_special()			\
do {						\
	REG_GPIO_PXFUNS(3) = 0x03C00000;	\
	REG_GPIO_PXSELC(3) = 0x03C00000;	\
	REG_GPIO_PXPES(3)  = 0x03C00000;	\
} while (0)

/*
 * CIM_D0~CIM_D7, CIM_MCLK, CIM_PCLK, CIM_VSYNC, CIM_HSYNC
 */
#define __gpio_as_cim()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00000fff;	\
	REG_GPIO_PXSELC(4) = 0x00000fff;	\
	REG_GPIO_PXPES(4)  = 0x00000fff;	\
} while (0)

/* 
 * SDATO, SDATI, BCLK, SYNC, SCLK_RSTN(gpio sepc) or
 * SDATA_OUT, SDATA_IN, BIT_CLK, SYNC, SCLK_RESET(aic spec)
 */
#define __gpio_as_aic()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x16c00000;	\
	REG_GPIO_PXTRGC(4) = 0x02c00000;	\
	REG_GPIO_PXTRGS(4) = 0x14000000;	\
	REG_GPIO_PXSELC(4) = 0x14c00000;	\
	REG_GPIO_PXSELS(4) = 0x02000000;	\
	REG_GPIO_PXPES(4)  = 0x16c00000;	\
} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D3
 */
#define __gpio_as_msc0_4bit()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0x00008000;	\
	REG_GPIO_PXTRGS(1) = 0x00008000;	\
	REG_GPIO_PXSELC(1) = 0x00008000;	\
	REG_GPIO_PXPES(1)  = 0x00008000;	\
	REG_GPIO_PXFUNS(2) = 0x38030000;	\
	REG_GPIO_PXTRGS(2) = 0x00010000;	\
	REG_GPIO_PXTRGC(2) = 0x38020000;	\
	REG_GPIO_PXSELC(2) = 0x08010000;	\
	REG_GPIO_PXSELS(2) = 0x30020000;	\
	REG_GPIO_PXPES(2)  = 0x38030000;	\
} while (0)

/*
 * MSC1_CMD, MSC1_CLK, MSC1_D0 ~ MSC1_D3
 */
#define __gpio_as_msc1_1bit()			\
	do {						\
		REG_GPIO_PXFUNS(1) = 0x34000000;	\
		REG_GPIO_PXTRGC(1) = 0x34000000;	\
		REG_GPIO_PXSELS(1) = 0x34000000;	\
		REG_GPIO_PXPES(1)  = 0x34000000;	\
	} while (0)

#define __gpio_as_msc	__gpio_as_msc0_4bit /* default as msc0 4bit */
#define __gpio_as_msc0	__gpio_as_msc0_4bit /* msc0 default as 4bit */
#define __gpio_as_msc1	__gpio_as_msc1_1bit /* msc1 only support 4bit */
/*
 * SSI_CE0, SSI_CE1, SSI_GPC, SSI_CLK, SSI_DT, SSI_DR
 */
#define __gpio_as_ssi()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0xfc000000;	\
	REG_GPIO_PXTRGC(1) = 0xfc000000;	\
	REG_GPIO_PXSELC(1) = 0xfc000000;	\
	REG_GPIO_PXPES(1)  = 0xfc000000;	\
} while (0)

/*
 * SSI_CE0, SSI_CE2, SSI_GPC, SSI_CLK, SSI_DT, SSI1_DR
 */
#define __gpio_as_ssi_1()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x0000fc00;	\
	REG_GPIO_PXTRGC(5) = 0x0000fc00;	\
	REG_GPIO_PXSELC(5) = 0x0000fc00;	\
	REG_GPIO_PXPES(5)  = 0x0000fc00;	\
} while (0)

/*
 * I2C_SCK, I2C_SDA
 */
#define __gpio_as_i2c()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00003000;	\
	REG_GPIO_PXSELC(4) = 0x00003000;	\
	REG_GPIO_PXPES(4)  = 0x00003000;	\
} while (0)

/*
 * PWM0
 */
#define __gpio_as_pwm0()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00100000;	\
	REG_GPIO_PXSELC(4) = 0x00100000;	\
	REG_GPIO_PXPES(4) = 0x00100000;		\
} while (0)

/*
 * PWM1
 */
#define __gpio_as_pwm1()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00000800;	\
	REG_GPIO_PXSELC(5) = 0x00000800;	\
	REG_GPIO_PXPES(5) = 0x00000800;		\
} while (0)

/*
 * PWM2
 */
#define __gpio_as_pwm2()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00400000;	\
	REG_GPIO_PXSELC(4) = 0x00400000;	\
	REG_GPIO_PXPES(4) = 0x00400000;		\
} while (0)

/*
 * PWM3
 */
#define __gpio_as_pwm3()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00800000;	\
	REG_GPIO_PXSELC(4) = 0x00800000;	\
	REG_GPIO_PXPES(4) = 0x00800000;		\
} while (0)

/*
 * PWM4
 */
#define __gpio_as_pwm4()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x01000000;	\
	REG_GPIO_PXSELC(4) = 0x01000000;	\
	REG_GPIO_PXPES(4) = 0x01000000;		\
} while (0)

/*
 * PWM5
 */
#define __gpio_as_pwm5()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x02000000;	\
	REG_GPIO_PXSELC(4) = 0x02000000;	\
	REG_GPIO_PXPES(4) = 0x02000000;		\
} while (0)

/*
 * n = 0 ~ 5
 */
#define __gpio_as_pwm(n)	__gpio_as_pwm##n()

/*
 * DREQ
 */
#define __gpio_as_dreq()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x00040000;	\
	REG_GPIO_PXSELS(2) = 0x00040000;	\
	REG_GPIO_PXPES(2) = 0x00040000;		\
} while (0)

/*
 * DACK
 */
#define __gpio_as_dack()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x00080000;	\
	REG_GPIO_PXSELS(2) = 0x00080000;	\
	REG_GPIO_PXPES(2) = 0x00080000;		\
} while (0)

/*
 * GPIO or Interrupt Mode
 */
#define __gpio_get_port(p)	(REG_GPIO_PXPIN(p))

#define __gpio_port_as_output(p, o)		\
do {						\
    REG_GPIO_PXFUNC(p) = (1 << (o));		\
    REG_GPIO_PXSELC(p) = (1 << (o));		\
    REG_GPIO_PXDIRS(p) = (1 << (o));		\
} while (0)

#define __gpio_port_as_input(p, o)		\
do {						\
    REG_GPIO_PXFUNC(p) = (1 << (o));		\
    REG_GPIO_PXSELC(p) = (1 << (o));		\
    REG_GPIO_PXDIRC(p) = (1 << (o));		\
} while (0)

#define __gpio_as_output(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output(p, o);		\
} while (0)

#define __gpio_as_input(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_input(p, o);		\
} while (0)

#define __gpio_set_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXDATS(p) = (1 << o);		\
} while (0)

#define __gpio_clear_pin(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXDATC(p) = (1 << o);		\
} while (0)

#define __gpio_get_pin(n)			\
({						\
	unsigned int p, o, v;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (__gpio_get_port(p) & (1 << o))	\
		v = 1;				\
	else					\
		v = 0;				\
	v;					\
})

#define __gpio_as_irq_high_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRS(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_low_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRC(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_rise_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRS(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_fall_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRC(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_mask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
} while (0)

#define __gpio_unmask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_ack_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
} while (0)

#define __gpio_get_irq()			\
({						\
	unsigned int p, i, tmp, v = 0;		\
	for (p = 3; p >= 0; p--) {		\
		tmp = REG_GPIO_PXFLG(p);	\
		for (i = 0; i < 32; i++)	\
			if (tmp & (1 << i))	\
				v = (32*p + i);	\
	}					\
	v;					\
})

#define __gpio_group_irq(n)			\
({						\
	register int tmp, i;			\
	tmp = REG_GPIO_PXFLG((n));		\
	for (i=31;i>=0;i--)			\
		if (tmp & (1 << i))		\
			break;			\
	i;					\
})

#define __gpio_enable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPEC(p) = (1 << o);		\
} while (0)

#define __gpio_disable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPES(p) = (1 << o);		\
} while (0)

#endif /* __ASM_JZ4750D_GPIO_H__ */

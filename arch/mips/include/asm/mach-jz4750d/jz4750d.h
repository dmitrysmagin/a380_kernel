/*
 *  linux/include/asm-mips/mach-jz4750d/jz4750d.h
 *
 *  JZ4750 common definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *  Mod: <maddrone@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750_H__
#define __ASM_JZ4750_H__

#include <asm/mach-jz4750d/base.h>
#include <asm/mach-jz4750d/jz4750d_regs.h>

#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>
#include <asm/mach-jz4750d/jz4750d_rtc.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>
#include <asm/mach-jz4750d/jz4750d_wdt.h>
#include <asm/mach-jz4750d/jz4750d_dmac.h>
#include <asm/mach-jz4750d/jz4750d_aic.h>
#include <asm/mach-jz4750d/jz4750d_uart.h>
#include <asm/mach-jz4750d/jz4750d_pcm.h>
#include <asm/mach-jz4750d/jz4750d_i2c.h>
#include <asm/mach-jz4750d/jz4750d_ssi.h>
#include <asm/mach-jz4750d/jz4750d_msc.h>
#include <asm/mach-jz4750d/jz4750d_emc.h>
#include <asm/mach-jz4750d/jz4750d_cim.h>
#include <asm/mach-jz4750d/jz4750d_sadc.h>
#include <asm/mach-jz4750d/jz4750d_lcdc.h>
#include <asm/mach-jz4750d/jz4750d_tve.h>
#include <asm/mach-jz4750d/jz4750d_udc.h>
#include <asm/mach-jz4750d/jz4750d_bch.h>
#include <asm/mach-jz4750d/jz4750d_owi.h>
#include <asm/mach-jz4750d/jz4750d_mc.h>
#include <asm/mach-jz4750d/jz4750d_me.h>
#include <asm/mach-jz4750d/jz4750d_otp.h>
#include <asm/mach-jz4750d/jz4750d_tssi.h>
#include <asm/mach-jz4750d/jz4750d_ipu.h>

#include <asm/mach-jz4750d/dma.h>
 
/*------------------------------------------------------------------
 * Platform definitions
 */

#ifdef CONFIG_JZ4750D_A380
#include <asm/mach-jz4750d/board-a380.h>
#endif

#ifdef CONFIG_JZ4750D_RZX50
#include <asm/mach-jz4750d/board-rzx50.h>
#endif

#endif /* __ASM_JZ4750_H__ */

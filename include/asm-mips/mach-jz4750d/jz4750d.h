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

#include <asm/mach-jz4750d/regs.h>
#include <asm/mach-jz4750d/ops.h>
#include <asm/mach-jz4750d/dma.h>
#include <asm/mach-jz4750d/misc.h>
#include <asm/mach-jz4750d/platform.h>
 
/*------------------------------------------------------------------
 * Platform definitions
 */
#ifdef CONFIG_JZ4750D_A380
#include <asm/mach-jz4750d/board-a380.h>
#endif

#ifdef CONFIG_JZ4750D_RZX50
#include <asm/mach-jz4750d/board-rzx50.h>
#endif


/*------------------------------------------------------------------
 * Follows are related to platform definitions
 */

#include <asm/mach-jz4750d/clock.h>
#include <asm/mach-jz4750d/serial.h>

#endif /* __ASM_JZ4750_H__ */

/*
 * linux/include/asm-mips/mach-jz4750d/regs.h
 *
 * JZ4750D register definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __JZ4750D_REGS_H__
#define __JZ4750D_REGS_H__

#if defined(__ASSEMBLY__) || defined(__LANGUAGE_ASSEMBLY)
#define REG8(addr)	(addr)
#define REG16(addr)	(addr)
#define REG32(addr)	(addr)
#else
#define REG8(addr)	*((volatile unsigned char *)(addr))
#define REG16(addr)	*((volatile unsigned short *)(addr))
#define REG32(addr)	*((volatile unsigned int *)(addr))
#endif

#endif /* __JZ4750D_REGS_H__ */

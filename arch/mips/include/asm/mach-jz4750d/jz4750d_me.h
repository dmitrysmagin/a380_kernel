/*
 * linux/include/asm-mips/mach-jz4750d/jz4750d_me.h
 *
 * JZ4750D ME definition.
 *
 * Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 * Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4750D_ME_H__
#define __ASM_JZ4750D_ME_H__

#include <asm/mach-jz4750d/jz4750d_regs.h>

#define ME_BASE		0xB30A0000

/*************************************************************************
 * ME (Motion Estimation)
 *************************************************************************/
#define ME_CTRL		(ME_BASE + 0x00) /* ME Control Register */
#define ME_REF_ADDR	(ME_BASE + 0x04) /* ME Reference Block Address Register */
#define ME_CURR_ADDR	(ME_BASE + 0x08) /* ME Current Block Address Register */
#define ME_DIFF_ADDR	(ME_BASE + 0x0C) /* ME Difference Address Register */
#define ME_REF_STRD	(ME_BASE + 0x10) /* ME Reference Frame Stride Register */
#define ME_CURR_STRD	(ME_BASE + 0x14) /* ME Current Frame Stride Register */
#define ME_DIFF_STRD	(ME_BASE + 0x18) /* ME Difference Frame Stride Register */
#define ME_SETTINGS	(ME_BASE + 0x1C) /* ME Settings Register */
#define ME_MVD		(ME_BASE + 0x20) /* ME Motion Vector Difference Register */
#define ME_FLAG		(ME_BASE + 0x24) /* ME Flag Register */

#define REG_ME_CTRL		REG32(ME_CTRL)
#define REG_ME_REF_ADDR		REG32(ME_REF_ADDR)
#define REG_ME_CURR_ADDR	REG32(ME_CURR_ADDR)
#define REG_ME_DIFF_ADDR	REG32(ME_DIFF_ADDR)
#define REG_ME_REF_STRD		REG32(ME_REF_STRD)
#define REG_ME_CURR_STRD	REG32(ME_CURR_STRD)
#define REG_ME_DIFF_STRD	REG32(ME_DIFF_STRD)
#define REG_ME_SETTINGS		REG32(ME_SETTINGS)
#define REG_ME_MVD		REG32(ME_MVD)
#define REG_ME_FLAG		REG32(ME_FLAG)


/* ME Control Register */
#define ME_CTRL_FLUSH		(1 << 2) /* ME cache clear */
#define ME_CTRL_RESET		(1 << 1) /* ME reset */
#define ME_CTRL_ENABLE		(1 << 0) /* ME enable */

/* ME Settings Register */
#define ME_SETTINGS_SAD_GATE_BIT	16 /* The max SAD value which can be accepted */
#define ME_SETTINGS_SAD_GATE_MASK	(0xffff << ME_SETTINGS_SAD_GATE_BIT)
#define ME_SETTINGS_STEP_NUM_BIT	0  /* The max step number the search process can not exceed */
#define ME_SETTINGS_STEP_NUM_MASK	(0x3f << ME_SETTINGS_STEP_NUM_BIT)

/* ME Motion Vector Difference Register */
#define ME_MVD_MVDY_BIT		16 /* The MVD value of coordinate-Y */
#define ME_MVD_MVDY_MASK	(0xffff << ME_MVD_MVDY_BIT)
#define ME_MVD_MVDX_BIT		0  /* The MVD value of coordinate-X */
#define ME_MVD_MVDX_MASK	(0xffff << ME_MVD_MVDX_BIT)

/* ME Flag Register */
#define ME_FLAG_INTRA		(1 << 1) /* Indicate the current MB will be predicted in intra mode */
#define ME_FLAG_COMPLETED	(1 << 0) /* The ME of the current part of the MB is completed */

#endif /* __ASM_JZ4750D_ME_H__ */

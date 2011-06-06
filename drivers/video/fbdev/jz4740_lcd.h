/*
 *  Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 *  	JZ4720/JZ4740 SoC LCD framebuffer driver
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __JZ4740_LCD_H__
#define __JZ4740_LCD_H__

#include <linux/bitops.h>

#define JZ_REG_LCD_CFG		0x00
#define JZ_REG_LCD_VSYNC	0x04
#define JZ_REG_LCD_HSYNC	0x08
#define JZ_REG_LCD_VAT		0x0C
#define JZ_REG_LCD_DAH		0x10
#define JZ_REG_LCD_DAV		0x14
#define JZ_REG_LCD_PS		0x18
#define JZ_REG_LCD_CLS		0x1C
#define JZ_REG_LCD_SPL		0x20
#define JZ_REG_LCD_REV		0x24
#define JZ_REG_LCD_CTRL		0x30
#define JZ_REG_LCD_STATE	0x34
#define JZ_REG_LCD_IID		0x38
#define JZ_REG_LCD_DA0		0x40
#define JZ_REG_LCD_SA0		0x44
#define JZ_REG_LCD_FID0		0x48
#define JZ_REG_LCD_CMD0		0x4C
#define JZ_REG_LCD_DA1		0x50
#define JZ_REG_LCD_SA1		0x54
#define JZ_REG_LCD_FID1		0x58
#define JZ_REG_LCD_CMD1		0x5C

#define JZ_LCD_CFG_SLCD			BIT(31)
#define JZ_LCD_CFG_PS_DISABLE		BIT(23)
#define JZ_LCD_CFG_CLS_DISABLE		BIT(22)
#define JZ_LCD_CFG_SPL_DISABLE		BIT(21)
#define JZ_LCD_CFG_REV_DISABLE		BIT(20)
#define JZ_LCD_CFG_HSYNCM		BIT(19)
#define JZ_LCD_CFG_PCLKM		BIT(18)
#define JZ_LCD_CFG_INV			BIT(17)
#define JZ_LCD_CFG_SYNC_DIR		BIT(16)
#define JZ_LCD_CFG_PS_POLARITY		BIT(15)
#define JZ_LCD_CFG_CLS_POLARITY		BIT(14)
#define JZ_LCD_CFG_SPL_POLARITY		BIT(13)
#define JZ_LCD_CFG_REV_POLARITY		BIT(12)
#define JZ_LCD_CFG_HSYNC_ACTIVE_LOW	BIT(11)
#define JZ_LCD_CFG_PCLK_FALLING_EDGE	BIT(10)
#define JZ_LCD_CFG_DE_ACTIVE_LOW	BIT(9)
#define JZ_LCD_CFG_VSYNC_ACTIVE_LOW	BIT(8)
#define JZ_LCD_CFG_18_BIT		BIT(7)
#define JZ_LCD_CFG_PDW			(BIT(5) | BIT(4))
#define JZ_LCD_CFG_MODE_MASK		0xf

#define JZ_LCD_CTRL_BURST_4		(0x0 << 28)
#define JZ_LCD_CTRL_BURST_8		(0x1 << 28)
#define JZ_LCD_CTRL_BURST_16		(0x2 << 28)
#define JZ_LCD_CTRL_RGB555		BIT(27)
#define JZ_LCD_CTRL_OFUP		BIT(26)
#define JZ_LCD_CTRL_FRC_GRAYSCALE_16	(0x0 << 24)
#define JZ_LCD_CTRL_FRC_GRAYSCALE_4	(0x1 << 24)
#define JZ_LCD_CTRL_FRC_GRAYSCALE_2	(0x2 << 24)
#define JZ_LCD_CTRL_PDD_MASK		(0xff << 16)
#define JZ_LCD_CTRL_EOF_IRQ		BIT(13)
#define JZ_LCD_CTRL_SOF_IRQ		BIT(12)
#define JZ_LCD_CTRL_OFU_IRQ		BIT(11)
#define JZ_LCD_CTRL_IFU0_IRQ		BIT(10)
#define JZ_LCD_CTRL_IFU1_IRQ		BIT(9)
#define JZ_LCD_CTRL_DD_IRQ		BIT(8)
#define JZ_LCD_CTRL_QDD_IRQ		BIT(7)
#define JZ_LCD_CTRL_REVERSE_ENDIAN	BIT(6)
#define JZ_LCD_CTRL_LSB_FISRT		BIT(5)
#define JZ_LCD_CTRL_DISABLE		BIT(4)
#define JZ_LCD_CTRL_ENABLE		BIT(3)
#define JZ_LCD_CTRL_BPP_1		0x0
#define JZ_LCD_CTRL_BPP_2		0x1
#define JZ_LCD_CTRL_BPP_4		0x2
#define JZ_LCD_CTRL_BPP_8		0x3
#define JZ_LCD_CTRL_BPP_15_16		0x4
#define JZ_LCD_CTRL_BPP_18_24		0x5

#define JZ_LCD_CMD_SOF_IRQ		BIT(31)
#define JZ_LCD_CMD_EOF_IRQ		BIT(30)
#define JZ_LCD_CMD_ENABLE_PAL		BIT(28)

#define JZ_LCD_SYNC_MASK		0x3ff

#define JZ_LCD_STATE_QUICK_DISABLED	BIT(7)
#define JZ_LCD_STATE_EOF		BIT(5)
#define JZ_LCD_STATE_SOF		BIT(4)
#define JZ_LCD_STATE_OUT_FIFO_UNDERRUN	BIT(3)
#define JZ_LCD_STATE_FIFO0_UNDERRUN	BIT(2)
#define JZ_LCD_STATE_FIFO1_UNDERRUN	BIT(1)
#define JZ_LCD_STATE_DISABLED		BIT(0)

#endif  /*__JZ4740_LCD_H__*/

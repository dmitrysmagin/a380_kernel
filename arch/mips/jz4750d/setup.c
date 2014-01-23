/*
 * linux/arch/mips/jz4750d/common/setup.c
 * 
 * JZ4750D common setup routines.
 * 
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/irq.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>
#include <asm/pgtable.h>
#include <asm/time.h>
#include <asm/jzsoc.h>

#ifdef CONFIG_PC_KEYB
#include <asm/keyboard.h>
#endif

#include "reset.h"

jz_clocks_t jz_clocks;

static void __init sysclocks_setup(void)
{
	jz_clocks.cclk = __cpm_get_cclk();
	jz_clocks.hclk = __cpm_get_hclk();
	jz_clocks.pclk = __cpm_get_pclk();
	jz_clocks.mclk = __cpm_get_mclk();
	jz_clocks.h1clk = __cpm_get_h1clk();
	jz_clocks.pixclk = __cpm_get_pixclk();
	jz_clocks.i2sclk = __cpm_get_i2sclk();
	jz_clocks.usbclk = __cpm_get_usbclk();
	jz_clocks.mscclk = __cpm_get_mscclk(0);
	jz_clocks.extalclk = __cpm_get_extalclk();
	jz_clocks.rtcclk = __cpm_get_rtcclk();

	printk("CPU clock: %dMHz, System clock: %dMHz, "
		"Peripheral clock: %dMHz, Memory clock: %dMHz\n",
		(jz_clocks.cclk + 500000) / 1000000,
		(jz_clocks.hclk + 500000) / 1000000,
		(jz_clocks.pclk + 500000) / 1000000,
		(jz_clocks.mclk + 500000) / 1000000);
}

static void __init soc_cpm_setup(void)
{
	/* Start all module clocks
	 */
	__cpm_start_all();

	/* Enable CKO to external memory */
	__cpm_enable_cko();

	/* CPU enters IDLE mode when executing 'wait' instruction */
	__cpm_idle_mode();

	/* Setup system clocks */
	sysclocks_setup();
}

static void __init soc_harb_setup(void)
{
//	__harb_set_priority(0x00);  /* CIM>LCD>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x03);  /* LCD>CIM>DMA>ETH>PCI>USB>CBB */
//	__harb_set_priority(0x0a);  /* ETH>LCD>CIM>DMA>PCI>USB>CBB */
}

static void __init soc_emc_setup(void)
{
}

static void __init soc_dmac_setup(void)
{
	__dmac_enable_module(0);
	__dmac_enable_module(1);
}

static void __init jz_soc_setup(void)
{
	soc_cpm_setup();
	soc_harb_setup();
	soc_emc_setup();
	soc_dmac_setup();
}

void __init plat_mem_setup(void)
{
	/* IO/MEM resources. Which will be the addtion value in `inX' and
	 * `outX' macros defined in asm/io.h */
	set_io_port_base(0);
	ioport_resource.start	= 0x00000000;
	ioport_resource.end	= 0xffffffff;
	iomem_resource.start	= 0x00000000;
	iomem_resource.end	= 0xffffffff;

	jz4750d_reset_init();

	jz_soc_setup();

	/* FIXME: Add memory size detection */
	add_memory_region(0, 0x04000000 /* 64M */, BOOT_MEM_RAM);
}

const char *get_system_type(void)
{
	return "JZ4750D";
}

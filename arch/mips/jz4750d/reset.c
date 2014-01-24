/*
 * linux/arch/mips/jz4750/reset.c
 *
 * JZ4750 reset routines.
 *
 * Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 * Author: <yliu@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/reboot.h>
#include <asm/system.h>

#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_wdt.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>

void jz_restart(char *command)
{
	printk("Restarting after 4 ms\n");
	REG_WDT_TCSR = WDT_TCSR_PRESCALE4 | WDT_TCSR_EXT_EN;
	REG_WDT_TCNT = 0;
	REG_WDT_TDR = JZ_EXTAL/1000;   /* reset after 4ms */
	REG_TCU_TSCR = TCU_TSCR_WDTSC; /* enable wdt clock */
	REG_WDT_TCER = WDT_TCER_TCEN;  /* wdt start */
	while (1);
}

void jz_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	while (1)
		__asm__(".set\tmips3\n\t"
	                "wait\n\t"
			".set\tmips0");
}

void jz_power_off(void)
{
	jz_halt();
}

void jz4750d_reset_init(void)
{
	_machine_restart = jz_restart;
	_machine_halt = jz_halt;
	pm_power_off = jz_power_off;

}


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

#include <asm/mach-jz4750d/jz4750d_intc.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_wdt.h>
#include <asm/mach-jz4750d/jz4750d_rtc.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>
#include <asm/mach-jz4750d/jz4750d_emc.h>
#include <asm/mach-jz4750d/jz4750d_sadc.h>

static void jz_halt(void)
{
	printk(KERN_NOTICE "\n** You can safely turn off the power\n");

	while (1)
		__asm__(".set\tmips3\n\t"
	                "wait\n\t"
			".set\tmips0");
}

static void jz_power_off(void)
{
	static unsigned int call_times = 0;

	printk("Put CPU into hibernate mode.\n");
	if(call_times > 0) return;
	call_times++;

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/*
	 * RTC Wakeup or 1Hz interrupt can be enabled or disabled
	 * through  RTC driver's ioctl (linux/driver/char/rtc_jz.c).
	 */

	/* Set minimum wakeup_n pin low-level assertion time for wakeup: 100ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWFCR = (100 << RTC_HWFCR_BIT);

	/* Set reset pin low-level assertion time after wakeup: must  > 60ms */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HRCR = (60 << RTC_HRCR_BIT); /* 60 ms */

	/* Scratch pad register to be reserved */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HSPR = 0x12345678;

        /* clear wakeup status register */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HWRSR = 0x0;

	/* Put CPU to power down mode */
	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	REG_RTC_HCR = RTC_HCR_PD;

	while (!(REG_RTC_RCR & RTC_RCR_WRDY));
	while(1);

	/* We can't get here */
	return;
}

static void jz_restart(char *command)
{
	printk("Restarting after 4 ms\n");
	REG_WDT_TCSR = WDT_TCSR_PRESCALE4 | WDT_TCSR_EXT_EN;
	REG_WDT_TCNT = 0;
	REG_WDT_TDR = JZ_EXTAL/1000;   /* reset after 4ms */
	REG_TCU_TSCR = TCU_TSCR_WDTSC; /* enable wdt clock */
	REG_WDT_TCER = WDT_TCER_TCEN;  /* wdt start */
	while (1);
}

void jz4750d_reset_init(void)
{
	_machine_restart = jz_restart;
	_machine_halt = jz_halt;
	pm_power_off = jz_power_off;

}

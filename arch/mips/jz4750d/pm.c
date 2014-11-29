/*
 * linux/arch/mips/jz4750d/pm.c
 *
 * JZ4750D Power Management Routines
 *
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
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
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/delay.h>
#include <linux/suspend.h>

#include <asm/cacheops.h>

#include <asm/mach-jz4750d/jz4750d_intc.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_wdt.h>
#include <asm/mach-jz4750d/jz4750d_rtc.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>
#include <asm/mach-jz4750d/jz4750d_emc.h>
#include <asm/mach-jz4750d/jz4750d_sadc.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

/* Put CPU to SLEEP mode */
static int jz_pm_enter(suspend_state_t state)
{
	unsigned long nfcsr = REG_EMC_NFCSR;
	unsigned long opcr = REG_CPM_OPCR;
	unsigned long imr = REG_INTC_IMR;
	unsigned long sadc = REG_SADC_ENA;

	printk("Put CPU into sleep mode.\n");

	/* Disable nand flash */
	REG_EMC_NFCSR = ~0xff;

	/* stop sadc */
	REG_SADC_ENA &= ~0x7;
	while((REG_SADC_ENA & 0x7) != 0);
 	udelay(100);

	/*stop udc and usb*/
	__cpm_suspend_uhcphy();
	__cpm_suspend_udcphy();

	/* Mask all interrupts */
	REG_INTC_IMSR = 0xffffffff;

	/* Just allow following interrupts to wakeup the system.
	 * Note: modify this according to your system.
	 */

	/* enable RTC alarm */
	__intc_unmask_irq(IRQ_RTC);

	/* disable externel clock Oscillator in sleep mode */
	__cpm_disable_osc_in_sleep();
	/* select 32K crystal as RTC clock in sleep mode */
	__cpm_select_rtcclk_rtc();

	/* Enter SLEEP mode */
	REG_CPM_LCR &= ~CPM_LCR_LPM_MASK;
	REG_CPM_LCR |= CPM_LCR_LPM_SLEEP;
	__asm__(".set\tmips3\n\t"
		"wait\n\t"
		".set\tmips0");

	/* Restore to IDLE mode */
	REG_CPM_LCR &= ~CPM_LCR_LPM_MASK;
	REG_CPM_LCR |= CPM_LCR_LPM_IDLE;

	/* Restore nand flash control register */
	REG_EMC_NFCSR = nfcsr;

	/* Restore interrupts */
	REG_INTC_IMSR = imr;
	REG_INTC_IMCR = ~imr;

	/* Restore sadc */
	REG_SADC_ENA = sadc;

	/* Restore Oscillator and Power Control Register */
	REG_CPM_OPCR = opcr;

	dprintk("%s %d after sleep\n",__FILE__,__LINE__);

	return 0;
}

static struct platform_suspend_ops jz_pm_ops = {
	.valid		= suspend_valid_only_mem,
	.enter		= jz_pm_enter,
};

/*
 * Initialize power interface
 */
int __init jz_pm_init(void)
{
	printk("Power Management for JZ47xx\n");

	suspend_set_ops(&jz_pm_ops);

	return 0;
}

module_init(jz_pm_init);

/*
 * linux/arch/mips/jz4750d/proc.c
 * 
 * /proc/jz/ procfs for jz4750d on-chip modules.
 * 
 * Copyright (C) 2006 Ingenic Semiconductor Inc.
 * Author: <jlwei@ingenic.cn>
 * Mod: <maddrone@gmail.com>
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/page-flags.h>
#include <linux/sched.h>

#include <asm/uaccess.h>
#include <asm/pgtable.h>

#include <asm/mach-jz4750d/jz4750d_emc.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>

/*
 * EMC Modules
 */
static int emc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf(page + len, "SMCR(0-5): "
			"0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
			REG_EMC_SMCR0, REG_EMC_SMCR1, REG_EMC_SMCR2,
			REG_EMC_SMCR3, REG_EMC_SMCR4);
	len += sprintf(page + len, "SACR(0-5): "
			"0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
			REG_EMC_SACR0, REG_EMC_SACR1, REG_EMC_SACR2,
			REG_EMC_SACR3, REG_EMC_SACR4);
	len += sprintf(page + len, "DMCR:      0x%08x\n", REG_EMC_DMCR);
	len += sprintf(page + len, "RTCSR:     0x%04x\n", REG_EMC_RTCSR);
	len += sprintf(page + len, "RTCOR:     0x%04x\n", REG_EMC_RTCOR);

	return len;
}

/* 
 * Power Manager Module
 */
static int pmc_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned long lcr = REG_CPM_LCR;
	unsigned long clkgr = REG_CPM_CLKGR;

	len += sprintf (page + len, "Low Power Mode : %s\n", 
			((lcr & CPM_LCR_LPM_MASK) == (CPM_LCR_LPM_IDLE)) ?
			"IDLE" : (((lcr & CPM_LCR_LPM_MASK) ==
							(CPM_LCR_LPM_SLEEP)) ? 
				  "SLEEP" : "HIBERNATE"));
	len += sprintf (page+len, "Doze Mode      : %s\n", 
			(lcr & CPM_LCR_DOZE_ON) ? "on" : "off");
	if (lcr & CPM_LCR_DOZE_ON)
		len += sprintf (page+len, "     duty      : %d\n",
				(int)((lcr & CPM_LCR_DOZE_DUTY_MASK) >>
				CPM_LCR_DOZE_DUTY_BIT));
	len += sprintf (page+len, "AUX_CPU        : %s\n",
			(clkgr & CPM_CLKGR_AUX_CPU) ? "stopped" : "running");
	len += sprintf (page+len, "AHB1           : %s\n",
			(clkgr & CPM_CLKGR_AHB1) ? "stopped" : "running");
	len += sprintf (page+len, "IDCT           : %s\n",
			(clkgr & CPM_CLKGR_IDCT) ? "stopped" : "running");
	len += sprintf (page+len, "DB             : %s\n",
			(clkgr & CPM_CLKGR_DB) ? "stopped" : "running");
	len += sprintf (page+len, "ME             : %s\n",
			(clkgr & CPM_CLKGR_ME) ? "stopped" : "running");
	len += sprintf (page+len, "MC             : %s\n",
			(clkgr & CPM_CLKGR_MC) ? "stopped" : "running");
	len += sprintf (page+len, "TVE            : %s\n",
			(clkgr & CPM_CLKGR_TVE) ? "stopped" : "running");
	len += sprintf (page+len, "TSSI           : %s\n",
			(clkgr & CPM_CLKGR_TSSI) ? "stopped" : "running");
	len += sprintf (page+len, "IPU            : %s\n",
			(clkgr & CPM_CLKGR_IPU) ? "stopped" : "running");
	len += sprintf (page+len, "DMAC           : %s\n",
			(clkgr & CPM_CLKGR_DMAC) ? "stopped" : "running");
	len += sprintf (page+len, "UDC            : %s\n",
			(clkgr & CPM_CLKGR_UDC) ? "stopped" : "running");
	len += sprintf (page+len, "LCD            : %s\n",
			(clkgr & CPM_CLKGR_LCD) ? "stopped" : "running");
	len += sprintf (page+len, "CIM            : %s\n",
			(clkgr & CPM_CLKGR_CIM) ? "stopped" : "running");
	len += sprintf (page+len, "SADC           : %s\n",
			(clkgr & CPM_CLKGR_SADC) ? "stopped" : "running");
	len += sprintf (page+len, "MSC0           : %s\n",
			(clkgr & CPM_CLKGR_MSC0) ? "stopped" : "running");
	len += sprintf (page+len, "MSC1           : %s\n",
			(clkgr & CPM_CLKGR_MSC1) ? "stopped" : "running");
	len += sprintf (page+len, "SSI           : %s\n",
			(clkgr & CPM_CLKGR_SSI) ? "stopped" : "running");
	len += sprintf (page+len, "I2C            : %s\n",
			(clkgr & CPM_CLKGR_I2C) ? "stopped" : "running");
	len += sprintf (page+len, "RTC            : %s\n",
			(clkgr & CPM_CLKGR_RTC) ? "stopped" : "running");
	len += sprintf (page+len, "TCU            : %s\n",
			(clkgr & CPM_CLKGR_TCU) ? "stopped" : "running");
	len += sprintf (page+len, "UART1          : %s\n",
			(clkgr & CPM_CLKGR_UART1) ? "stopped" : "running");
	len += sprintf (page+len, "UART0          : %s\n",
			(clkgr & CPM_CLKGR_UART0) ? "stopped" : "running");
	return len;
}

static int pmc_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CLKGR = simple_strtoul(buffer, 0, 16);
	return count;
}

/*
 * Clock Generation Module
 */
#define TO_MHZ(x) (x/1000000),(x%1000000)/10000
#define TO_KHZ(x) (x/1000),(x%1000)/10

static int cgm_read_proc (char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	unsigned int cppcr = REG_CPM_CPPCR;  /* PLL Control Register */
	unsigned int cpccr = REG_CPM_CPCCR;  /* Clock Control Register */
	unsigned int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int od[4] = {1, 2, 2, 4};

	len += sprintf (page+len, "CPPCR          : 0x%08x\n", cppcr);
	len += sprintf (page+len, "CPCCR          : 0x%08x\n", cpccr);
	len += sprintf (page+len, "PLL            : %s\n", 
			(cppcr & CPM_CPPCR_PLLEN) ? "ON" : "OFF");
	len += sprintf (page+len, "m:n:o          : %d:%d:%d\n",
			__cpm_get_pllm() + 2,
			__cpm_get_plln() + 2,
			od[__cpm_get_pllod()]
		);
	len += sprintf (page+len, "C:H:M:P        : %d:%d:%d:%d\n", 
			div[__cpm_get_cdiv()],
			div[__cpm_get_hdiv()],
			div[__cpm_get_mdiv()],
			div[__cpm_get_pdiv()]
		);
	len += sprintf (page+len, "PLL Freq        : %3d.%02d MHz\n", TO_MHZ(__cpm_get_pllout()));
	len += sprintf (page+len, "CCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_cclk()));
	len += sprintf (page+len, "HCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_hclk()));
	len += sprintf (page+len, "MCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mclk()));
	len += sprintf (page+len, "PCLK            : %3d.%02d MHz\n", TO_MHZ(__cpm_get_pclk()));
	len += sprintf (page+len, "H1CLK           : %3d.%02d MHz\n", TO_MHZ(__cpm_get_h1clk()));
	len += sprintf (page+len, "PIXCLK          : %3d.%02d KHz\n", TO_KHZ(__cpm_get_pixclk()));
	len += sprintf (page+len, "I2SCLK          : %3d.%02d MHz\n", TO_MHZ(__cpm_get_i2sclk()));
	len += sprintf (page+len, "USBCLK          : %3d.%02d MHz\n", TO_MHZ(__cpm_get_usbclk()));
	len += sprintf (page+len, "MSC0CLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mscclk(0)));
	len += sprintf (page+len, "MSC1CLK         : %3d.%02d MHz\n", TO_MHZ(__cpm_get_mscclk(1)));
	len += sprintf (page+len, "EXTALCLK0       : %3d.%02d MHz\n", TO_MHZ(__cpm_get_extalclk0()));
	len += sprintf (page+len, "EXTALCLK(by CPM): %3d.%02d MHz\n", TO_MHZ(__cpm_get_extalclk()));
	len += sprintf (page+len, "RTCCLK          : %3d.%02d MHz\n", TO_MHZ(__cpm_get_rtcclk()));

	return len;
}

static int cgm_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	REG_CPM_CPCCR = simple_strtoul(buffer, 0, 16);
	return count;
}

/*
 * /proc/jz/xxx entry
 *
 */
static int __init jz_proc_init(void)
{
	struct proc_dir_entry *proc_jz_root;
	struct proc_dir_entry *res;

	proc_jz_root = proc_mkdir("jz", 0);
#if 0
	/* External Memory Controller */
	res = create_proc_entry("emc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = emc_read_proc;
		res->write_proc = NULL;
		res->data = NULL;
	}

	/* Power Management Controller */
	res = create_proc_entry("pmc", 0644, proc_jz_root);
	if (res) {
		res->read_proc = pmc_read_proc;
		res->write_proc = pmc_write_proc;
		res->data = NULL;
	}

	/* Clock Generation Module */
	res = create_proc_entry("cgm", 0644, proc_jz_root);
	if (res) {
		res->read_proc = cgm_read_proc;
		res->write_proc = cgm_write_proc;
		res->data = NULL;
	}
#endif
	return 0;
}

__initcall(jz_proc_init);

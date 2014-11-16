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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/mach-jz4750d/jz4750d_emc.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>

/*
 * EMC Modules
 */
static int emc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "SMCR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
			REG_EMC_SMCR0, REG_EMC_SMCR1, REG_EMC_SMCR2,
			REG_EMC_SMCR3, REG_EMC_SMCR4);

	seq_printf(m, "SACR(0-5): 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x\n",
			REG_EMC_SACR0, REG_EMC_SACR1, REG_EMC_SACR2,
			REG_EMC_SACR3, REG_EMC_SACR4);

	seq_printf(m, "DMCR:      0x%08x\n", REG_EMC_DMCR);
	seq_printf(m, "RTCSR:     0x%04x\n", REG_EMC_RTCSR);
	seq_printf(m, "RTCOR:     0x%04x\n", REG_EMC_RTCOR);

	return 0;
}

static int emc_open(struct inode *inode, struct file *file)
{
	return single_open(file, emc_show, NULL);
}

static const struct file_operations emc_fops = {
	.open		= emc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* 
 * Power Manager Module
 */
static int pmc_show(struct seq_file *m, void *v)
{
	unsigned long lcr = REG_CPM_LCR;
	unsigned long clkgr = REG_CPM_CLKGR;

	seq_printf(m, "Low Power Mode : %s\n", 
			((lcr & CPM_LCR_LPM_MASK) == (CPM_LCR_LPM_IDLE)) ?
			"IDLE" : (((lcr & CPM_LCR_LPM_MASK) ==
							(CPM_LCR_LPM_SLEEP)) ? 
				  "SLEEP" : "HIBERNATE"));

	seq_printf(m, "Doze Mode      : %s\n", 
			(lcr & CPM_LCR_DOZE_ON) ? "on" : "off");

	if (lcr & CPM_LCR_DOZE_ON)
		seq_printf(m, "     duty      : %d\n",
				(int)((lcr & CPM_LCR_DOZE_DUTY_MASK) >>
				CPM_LCR_DOZE_DUTY_BIT));

	seq_printf(m, "AUX_CPU        : %s\n",
			(clkgr & CPM_CLKGR_AUX_CPU) ? "stopped" : "running");
	seq_printf(m, "AHB1           : %s\n",
			(clkgr & CPM_CLKGR_AHB1) ? "stopped" : "running");
	seq_printf(m, "IDCT           : %s\n",
			(clkgr & CPM_CLKGR_IDCT) ? "stopped" : "running");
	seq_printf(m, "DB             : %s\n",
			(clkgr & CPM_CLKGR_DB) ? "stopped" : "running");
	seq_printf(m, "ME             : %s\n",
			(clkgr & CPM_CLKGR_ME) ? "stopped" : "running");
	seq_printf(m, "MC             : %s\n",
			(clkgr & CPM_CLKGR_MC) ? "stopped" : "running");
	seq_printf(m, "TVE            : %s\n",
			(clkgr & CPM_CLKGR_TVE) ? "stopped" : "running");
	seq_printf(m, "TSSI           : %s\n",
			(clkgr & CPM_CLKGR_TSSI) ? "stopped" : "running");
	seq_printf(m, "IPU            : %s\n",
			(clkgr & CPM_CLKGR_IPU) ? "stopped" : "running");
	seq_printf(m, "DMAC           : %s\n",
			(clkgr & CPM_CLKGR_DMAC) ? "stopped" : "running");
	seq_printf(m, "UDC            : %s\n",
			(clkgr & CPM_CLKGR_UDC) ? "stopped" : "running");
	seq_printf(m, "LCD            : %s\n",
			(clkgr & CPM_CLKGR_LCD) ? "stopped" : "running");
	seq_printf(m, "CIM            : %s\n",
			(clkgr & CPM_CLKGR_CIM) ? "stopped" : "running");
	seq_printf(m, "SADC           : %s\n",
			(clkgr & CPM_CLKGR_SADC) ? "stopped" : "running");
	seq_printf(m, "MSC0           : %s\n",
			(clkgr & CPM_CLKGR_MSC0) ? "stopped" : "running");
	seq_printf(m, "MSC1           : %s\n",
			(clkgr & CPM_CLKGR_MSC1) ? "stopped" : "running");
	seq_printf(m, "SSI           : %s\n",
			(clkgr & CPM_CLKGR_SSI) ? "stopped" : "running");
	seq_printf(m, "I2C            : %s\n",
			(clkgr & CPM_CLKGR_I2C) ? "stopped" : "running");
	seq_printf(m, "RTC            : %s\n",
			(clkgr & CPM_CLKGR_RTC) ? "stopped" : "running");
	seq_printf(m, "TCU            : %s\n",
			(clkgr & CPM_CLKGR_TCU) ? "stopped" : "running");
	seq_printf(m, "UART1          : %s\n",
			(clkgr & CPM_CLKGR_UART1) ? "stopped" : "running");
	seq_printf(m, "UART0          : %s\n",
			(clkgr & CPM_CLKGR_UART0) ? "stopped" : "running");

	return 0;
}

static ssize_t pmc_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *pos)
{
	char buf[16];

	copy_from_user(buf, buffer, count);

	REG_CPM_CLKGR = simple_strtoul(buf, 0, 16);

	return count;
}

static int pmc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_show, NULL);
}

static const struct file_operations pmc_fops = {
	.open		= pmc_open,
	.read		= seq_read,
	.write		= pmc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * Clock Generation Module
 */
#define TO_MHZ(x) (x/1000000),(x%1000000)/10000
#define TO_KHZ(x) (x/1000),(x%1000)/10

static int cgm_show(struct seq_file *m, void *v)
{
	unsigned int cppcr = REG_CPM_CPPCR;  /* PLL Control Register */
	unsigned int cpccr = REG_CPM_CPCCR;  /* Clock Control Register */
	unsigned int div[] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	unsigned int od[4] = {1, 2, 2, 4};

	seq_printf(m, "CPPCR          : 0x%08x\n", cppcr);
	seq_printf(m, "CPCCR          : 0x%08x\n", cpccr);
	seq_printf(m, "PLL            : %s\n", 
			(cppcr & CPM_CPPCR_PLLEN) ? "ON" : "OFF");

	seq_printf(m, "m:n:o          : %d:%d:%d\n",
			__cpm_get_pllm() + 2,
			__cpm_get_plln() + 2,
			od[__cpm_get_pllod()]);

	seq_printf(m, "C:H:M:P        : %d:%d:%d:%d\n", 
			div[__cpm_get_cdiv()],
			div[__cpm_get_hdiv()],
			div[__cpm_get_mdiv()],
			div[__cpm_get_pdiv()]);

	seq_printf(m, "PLL Freq        : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_pllout()));
	seq_printf(m, "CCLK            : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_cclk()));
	seq_printf(m, "HCLK            : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_hclk()));
	seq_printf(m, "MCLK            : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_mclk()));
	seq_printf(m, "PCLK            : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_pclk()));
	seq_printf(m, "H1CLK           : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_h1clk()));
	seq_printf(m, "PIXCLK          : %3d.%02d KHz\n",
			TO_KHZ(__cpm_get_pixclk()));
	seq_printf(m, "I2SCLK          : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_i2sclk()));
	seq_printf(m, "USBCLK          : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_usbclk()));
	seq_printf(m, "MSC0CLK         : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_mscclk(0)));
	seq_printf(m, "MSC1CLK         : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_mscclk(1)));
	seq_printf(m, "EXTALCLK0       : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_extalclk0()));
	seq_printf(m, "EXTALCLK(by CPM): %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_extalclk()));
	seq_printf(m, "RTCCLK          : %3d.%02d MHz\n",
			TO_MHZ(__cpm_get_rtcclk()));

	return 0;
}

static ssize_t cgm_write(struct file *file, const char __user *buffer,
			size_t count, loff_t *pos)
{
	char buf[16];

	copy_from_user(buf, buffer, count);

	REG_CPM_CPCCR = simple_strtoul(buf, 0, 16);

	return count;
}

static int cgm_open(struct inode *inode, struct file *file)
{
	return single_open(file, cgm_show, NULL);
}

static const struct file_operations cgm_fops = {
	.open		= cgm_open,
	.read		= seq_read,
	.write		= cgm_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*
 * /proc/jz/xxx entry
 *
 */
static int __init jz_proc_init(void)
{
	struct proc_dir_entry *proc_jz_root;

	proc_jz_root = proc_mkdir("jz", 0);

	/* External Memory Controller */
	proc_create("emc", 0644, proc_jz_root, &emc_fops);

	/* Power Management Controller */
	proc_create("pmc", 0644, proc_jz_root, &pmc_fops);

	/* Clock Generation Module */
	proc_create("cgm", 0644, proc_jz_root, &cgm_fops);

	return 0;
}

__initcall(jz_proc_init);

/*
 * linux/drivers/char/jzchar/jz_sadc.c
 *
 * SAR-ADC driver for JZ4740.
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 * Copyright (C) 2009  Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kthread.h>

#include <asm/irq.h>
#include <asm/uaccess.h>

#include <asm/mach-jz4750d/jz4750d_sadc.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>

// Remove later
#if defined CONFIG_JZ4750D_A380
  #include <asm/mach-jz4750d/board-a380.h>
#elif defined CONFIG_JZ4750D_RZX50
  #include <asm/mach-jz4750d/board-rzx50.h>
#endif

#include "jzchars.h"

MODULE_AUTHOR("Jianli Wei<jlwei@ingenic.cn>");
MODULE_DESCRIPTION("JZ4740 SADC driver");
MODULE_LICENSE("GPL");

#define SADC_NAME        "jz-sadc"

struct sadc_device {
	int mode;
	int dma_chan;
	char *ts_buf;
	char *pbat_buf;
};

static struct sadc_device *sadc_dev;

extern unsigned int (*codec_read_battery)(void);

unsigned int jz4740_read_battery(void);

/*
 * set adc clock to 12MHz/div. A/D works at freq between 500KHz to 6MHz.
 */
static void sadc_init_clock(int div)
{
	if (div < 2) div = 2;
	if (div > 23) div = 23;
#if defined(CONFIG_SOC_JZ4740)
	REG_SADC_CFG &= ~SADC_CFG_CLKDIV_MASK;
	REG_SADC_CFG |= (div - 1) << SADC_CFG_CLKDIV_BIT;
#endif
#if defined(CONFIG_SOC_JZ4750) || defined(CONFIG_SOC_JZ4750D)
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_MASK;
	REG_SADC_ADCLK |= (div - 1) << SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK &= ~SADC_ADCLK_CLKDIV_BIT;
	REG_SADC_ADCLK |= 39 << SADC_ADCLK_CLKDIV_10_BIT;  /* if div ==3,here is 39 */
#endif
}

void start_sadcin(void)
{
	REG_SADC_CTRL &= ~SADC_CTRL_SRDYM; /* enable interrupt */
	REG_SADC_ENA |= SADC_ENA_SADCINEN;
}

void start_pbat_adc(void)
{
	//REG_SADC_CFG |= SADC_CFG_PBAT_HIGH;   /* full baterry voltage >= 2.5V */
  	REG_SADC_CFG |= SADC_CFG_PBAT_LOW;    /* full baterry voltage < 2.5V */
  	REG_SADC_ENA |= SADC_ENA_PBATEN;      /* Enable pbat adc */
}

/*------------------------------------------------------------
 * Read the battery voltage
 */
unsigned int jz4740_read_battery(void)
{
	unsigned int v;
	unsigned int timeout = 0x3ff;
	u16 pbat;

	if(!(REG_SADC_STATE & SADC_STATE_PBATRDY) == 1)
		start_pbat_adc();

	/* Be nice and yield, we are not in a hurry here, it's just the battery !!! */
	while(!(REG_SADC_STATE & SADC_STATE_PBATRDY) && --timeout)
		yield();

	pbat = REG_SADC_BATDAT;
	v = pbat & 0x0fff;
	REG_SADC_STATE = SADC_STATE_PBATRDY;
	return v;
}

/*
 * Device file operations
 */
static int sadc_open(struct inode *inode, struct file *filp);
static int sadc_release(struct inode *inode, struct file *filp);
static long sadc_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations sadc_fops =
{
	open:           sadc_open,
	release:        sadc_release,
	unlocked_ioctl: sadc_ioctl
};

static int sadc_open(struct inode *inode, struct file *filp)
{
 	try_module_get(THIS_MODULE);
	return 0;
}

static int sadc_release(struct inode *inode, struct file *filp)
{
 	module_put(THIS_MODULE);
	return 0;
}

static long sadc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	default:
		printk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return 0;
}

/*
 * procfs interface file operations
 */

static int proc_sadc_battery_open(struct inode *inode, struct file *file);

static const struct file_operations proc_sadc_battery_fops = {
	.open		= proc_sadc_battery_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static unsigned int battery_mv = (41500 - 250) / 4; // default full power
extern void udc_pnp_set_gpio(void);

static int proc_sadc_battery_show(struct seq_file *m, void *v)
{
	unsigned int mv = (battery_mv - 180) / 4;// = (jz4740_read_battery() * 7500 + 2048) / 4096;

	//maddrone add charge status
	__gpio_as_input(GPIO_USB_DETE);
	__gpio_disable_pull(GPIO_USB_DETE);

	if(__gpio_get_pin(GPIO_USB_DETE))
		mv |= 0x80000000;

	udc_pnp_set_gpio();
#ifdef GPIO_CHARG_STAT_N
	__gpio_as_input(GPIO_CHARG_STAT_N);
	__gpio_disable_pull(GPIO_CHARG_STAT_N);
	if(__gpio_get_pin(GPIO_CHARG_STAT_N))  mv |= 0x40000000;
#endif
	seq_printf(m, "%u\n", mv);
	return 0;
}

static int proc_sadc_battery_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sadc_battery_show, NULL);
}

/*------------------ Common routines -------------------*/

static struct task_struct * battery_monitor;

#define POWEROFF_VOL 3450

extern int jz_pm_hibernate(void);

static int battery_track_timer(void *data)
{
	int over_time = 0;

	//printk("kernel battery_track_time thread start!\n");

	while (1) {
		unsigned int mv;

		battery_mv = jz4740_read_battery();
		mv = (battery_mv * 10000 / 4096) + 80; //battery_mv * 4 + 250;
		battery_mv = mv;

		if (mv < POWEROFF_VOL) {
			over_time++;
			if (over_time > 5) {
				//printk("low power !\n");

				if (__gpio_get_pin(GPIO_CHARG_STAT_N)) {
					printk("the power is too low do hibernate!!!!!\n");
#ifdef CONFIG_JZ_POWEROFF
					extern void run_sbin_poweroff();
					run_sbin_poweroff();
#endif
					jz_pm_hibernate();
				}
			}
		} else {
			over_time = 0;
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ*10);
	}

	return 0;
}

/*
 * Module init and exit
 */
static int __init sadc_init(void)
{
	struct sadc_device *dev;
	struct proc_dir_entry *res;
	int ret;

	/* allocate device */
	dev = kmalloc(sizeof(struct sadc_device), GFP_KERNEL);
	if (!dev) return -ENOMEM;

	sadc_dev = dev;
	ret = jz_register_chrdev(SADC_MINOR, SADC_NAME, &sadc_fops, dev);
	if (ret < 0) {
		kfree(dev);
		return ret;
	}

	res = create_proc_entry("jz/battery", 0, NULL);
	if (res) {
		res->proc_fops = &proc_sadc_battery_fops;
	}

	sadc_init_clock(3);

	battery_monitor = kthread_run(&battery_track_timer, NULL, "battery _monitor");
	if (IS_ERR(battery_monitor)) {
		printk("Kernel battery _monitor thread start error!\n");
		return -1;
	}

	printk("JZ4740 SAR-ADC driver registered\n");
	return 0;
}

static void __exit sadc_exit(void)
{
	struct sadc_device *dev = sadc_dev;

	remove_proc_entry("jz/battery", NULL);

	free_irq(IRQ_SADC, dev);
	jz_unregister_chrdev(SADC_MINOR, SADC_NAME);
	kfree(dev);
}

module_init(sadc_init);
module_exit(sadc_exit);

/*
 *  Keyboard driver for the IngeniC JZ SoC
 *
 *  Copyright (c) 2009 Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 *  Mod: <maddrone@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  TODO(IGP):
 *  - On power slider long press, use 'o' instead of 'b' to power off instead of reboot.
 *  - Button wake up (when idle and low power stuff works).
 *
 */

#include <linux/init.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/sysrq.h>
#include <linux/sched.h>

#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

#define SCAN_INTERVAL		(20)	/* (ms) */
#define POWER_INTERVAL		(2000)	/* (ms) */
#define POFF_INTERVAL		(5000)	/* (ms) */
#define POWER_COUNT		(POWER_INTERVAL / SCAN_INTERVAL)
#define POFF_COUNT		(POFF_INTERVAL / SCAN_INTERVAL)



/*
 * NOTE: maximum 32 GPIO, since we use bits in an unsigned long to store states
 */

static const struct {
	unsigned int gpio;
	unsigned int actlow;	/* Active low */
	unsigned int ncode;	/* Normal keycode */
	unsigned int scode;	/* Special keycode */
	unsigned int sysrq;	/* SYSRQ code */
	unsigned int wakeup;
} jz_button[] = {

#ifdef CONFIG_JZ4740_A320
	{ .gpio = 102,	.actlow = 1,	.ncode = KEY_UP,	.scode = KEY_VOLUMEUP,		.sysrq = 's'	}, /* D-pad up */
	{ .gpio = 123,	.actlow = 1, 	.ncode = KEY_DOWN,	.scode = KEY_VOLUMEDOWN,	.sysrq = 'u'	}, /* D-pad down */
	{ .gpio = 101,	.actlow = 1,	.ncode = KEY_LEFT,	.scode = KEY_BRIGHTNESSDOWN,	.sysrq = 'e'	}, /* D-pad left */
	{ .gpio = 114,	.actlow = 1,	.ncode = KEY_RIGHT,	.scode = KEY_BRIGHTNESSUP,	.sysrq = 'i'	}, /* D-pad right */
	{ .gpio = 96,	.actlow = 1,	.ncode = KEY_LEFTCTRL,							}, /* A button */
	{ .gpio = 97,	.actlow = 1,	.ncode = KEY_LEFTALT,							}, /* B button */
	{ .gpio = 115,	.actlow = 1,	.ncode = KEY_SPACE,							}, /* X button */
	{ .gpio = 98,	.actlow = 1,	.ncode = KEY_LEFTSHIFT,							}, /* Y button */
	{ .gpio = 110,	.actlow = 1,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = 111,	.actlow = 1,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = 81,	.actlow = 1,	.ncode = KEY_ENTER,							}, /* START button(SYSRQ) */
	{ .gpio = 113,	.actlow = 1,	.ncode = KEY_ESC,	.scode = KEY_MENU,		.sysrq = 'b'	}, /* SELECT button */
	{ .gpio = 118,	.actlow = 1,	.ncode = KEY_PAUSE,							}, /* HOLD */

#define GPIO_POWER		GPIO_WAKEUP	/* Power slider */
#define GPIO_POWER_ACTLOW	1
#define SYSRQ_ALT		10	/* Alternate sysrq button (index in jz_button table) */
#endif

#ifdef CONFIG_JZ4740_PAVO
	{ .gpio = 96,	.actlow = 1,	.ncode = KEY_1 },
	{ .gpio = 97,	.actlow = 1,	.ncode = KEY_2 },
	{ .gpio = 98,	.actlow = 1,	.ncode = KEY_3 },
	{ .gpio = 99,	.actlow = 1,	.ncode = KEY_4 },
#endif

#ifdef CONFIG_JZ4750D_A380
	{ .gpio = (32*4+0)/*GPE0*/,	.actlow = 1,	.ncode = KEY_UP,	.scode = KEY_VOLUMEUP,		.sysrq = 's'	}, /* D-pad up */
	{ .gpio = (32*4+1)/*GPE1*/,	.actlow = 1, 	.ncode = KEY_DOWN,	.scode = KEY_VOLUMEDOWN,	.sysrq = 'u'	}, /* D-pad down */
	{ .gpio = (32*4+2)/*GPE2*/,	.actlow = 1,	.ncode = KEY_LEFT,	.scode = KEY_BRIGHTNESSDOWN,	.sysrq = 'e'	}, /* D-pad left */
	{ .gpio = (32*4+3)/*GPE3*/,	.actlow = 1,	.ncode = KEY_RIGHT,	.scode = KEY_BRIGHTNESSUP,	.sysrq = 'i'	}, /* D-pad right */
	{ .gpio = (32*2+31)/*GPC31*/,	.actlow = 1,	.ncode = KEY_LEFTCTRL,							}, /* A button */
	{ .gpio = (32*3+16)/*GPE11*/,	.actlow = 1,	.ncode = KEY_LEFTALT,							}, /* B button */
	{ .gpio = (32*3+17)/*GPD16*/,	.actlow = 1,	.ncode = KEY_SPACE,							}, /* X button */
	{ .gpio = (32*4+11)/*GPD17*/,	.actlow = 1,	.ncode = KEY_LEFTSHIFT,							}, /* Y button */
	{ .gpio = (32*4+10) /*GPE7*/,	.actlow = 1,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = (32*4+7)/*GPE10*/,	.actlow = 1,	.ncode = KEY_BACKSPACE ,							}, /* Right shoulder button */
	{ .gpio = (32*3+21)/*GPD21*/,	.actlow = 1,	.ncode = KEY_ENTER,							}, /* START button(SYSRQ) */
	{ .gpio = (32*4+8)/*GPE8*/,	.actlow = 1,	.ncode = KEY_ESC,	.scode = KEY_MENU,		.sysrq = 'b'	}, /* SELECT button */
//	{ .gpio = (32*3+18)/*GPD18*/,	.actlow = 1,	.ncode = KEY_PAUSE,							}, /* HOLD */

#define GPIO_POWER		(125)	/* Power slider */
#define GPIO_POWER_ACTLOW	1
//#define SYSRQ_ALT		10	/* Alternate sysrq button (index in jz_button table) */
#endif
};




struct jz_kbd {
	unsigned int			keycode [2 * ARRAY_SIZE(jz_button)];
	struct input_polled_dev *	poll_dev;
	unsigned long			normal_state;	/* Normal key state */
	unsigned long			special_state;	/* Special key state */
	unsigned long			sysrq_state;	/* SYSRQ key state */
	unsigned long			actlow;		/* Active low mask */
	unsigned int			power_state;	/* Power slider state */
	unsigned int			power_count;	/* Power slider active count */
};

static unsigned int sdl_key_enable = 1;
static int proc_alt_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", sdl_key_enable);
}

static int proc_alt_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	sdl_key_enable =  simple_strtoul(buffer, 0, 10);
	return count;
}


/*
 poweroff = SIGUSR2
 reboot   = SIGINT
*/

static void send_sig_to_init( int sig )
{
	struct task_struct *p;

	for_each_process(p)
	{
		if (is_global_init(p))
			force_sig(sig, p);
	}
}


static void send_sig_all( int sig )
{
#if 0
	struct task_struct *p;

	for_each_process(p)
	{
		if (p->mm && !is_global_init(p)) /* without important stuff */
			force_sig(sig, p);
	}
#endif
}

static void prepare_for_restart(void)
{
#ifdef CONFIG_JZ4740_A320
	/* TODO : rewrite more smart code */
	__gpio_as_output(GPIO_LCD_BACKLIGHT);
	__gpio_clear_pin(GPIO_LCD_BACKLIGHT);
#endif

//maddrone
//	send_sig_all(SIGTERM);
//	send_sig_all(SIGKILL);
}

//maddrone add elan wireless pad
extern unsigned int elan_key_val;
extern unsigned long elan_state_flag;
extern unsigned int l009_gsensor_flag;
extern unsigned int l009_gsensor_read();
#define HOLD_DETET 32*3+18

static void jz_kbd_poll (struct input_polled_dev *dev)
{
	if(sdl_key_enable)
	{
	struct jz_kbd *kbd = dev->private;
	struct input_dev *input = kbd->poll_dev->input;
	unsigned int i, p, sync = 0;
	unsigned long s, m, x;

	/* TODO: lock */

	//maddrone: Add wireless gamepad here

	/* Scan raw key states */
	for (s = 0, m = 1, i = 0; i < ARRAY_SIZE(jz_button); i++, m <<= 1)
	{
		if(i == 4)
		{
			if (!__gpio_get_pin(jz_button[i].gpio)) s |= m;
		}
		else
		{
			if (__gpio_get_pin(jz_button[i].gpio)) s |= m;
		}
	}
#if 1
	if(elan_state_flag > 0 && elan_key_val != 0)
	{

                //printk("%s %d %d %d \n",__FILE__,__LINE__,elan_state_flag,elan_key_val);
		if(s == 0xfff) s=0;
		//printk("elan_key_val = 0x%x, s=0x%x\n",elan_key_val,s);
		s |= ((~elan_key_val) & 0xfff);
		//printk("s = 0x%x\n",s);
	}
#endif

#if 1
	//printk("\n\n\n--------- s = 0x%x\n",s);
	if(l009_gsensor_flag)
	{
		unsigned int val;
		val = l009_gsensor_read();
		s &= (~val);
	}
	//printk("+++++++++ s = 0x%x\n",s);
#endif

	/* Invert active low buttons */
	s ^= kbd->actlow;

        extern void draw_lock_picture(void);

        if(s != 0)
        {
          if(!__gpio_get_pin(HOLD_DETET))
          {
            draw_lock_picture();
			mdelay(500);
			return;
		}
	}
	/* Read power slider state */
#ifdef GPIO_POWER
#ifdef GPIO_POWER_ACTLOW
	p = !__gpio_get_pin(GPIO_POWER);
#else
	p = __gpio_get_pin(GPIO_POWER);
#endif
#else
	p = 0;
#endif

	if (p) {

#if 0
		/* If power slider and SYSRQ_ALT button are pressed... */
#ifdef SYSRQ_ALT
		if (s & (1 << SYSRQ_ALT)) {

			/* Calculate changed button state for system requests */
			x = s ^ kbd->sysrq_state;

			/* Generate system requests (only on keypress) */
			for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
				if ((x & s & m) && jz_button[i].sysrq)
				{
					/* added safety code for reboot */
					if (jz_button[i].sysrq == 'b')
					{
						prepare_for_restart();
						send_sig_to_init(SIGINT);
					}
					else
					{
						handle_sysrq(jz_button[i].sysrq, NULL);
					}
				}
			}

			kbd->power_count = 0;	/* Stop power slider pressed counter */
			kbd->sysrq_state = s;	/* Update current sysrq button state */
		}

		else
#endif
		/* If power slider is pressed... */
		{

			/* Calculate changed button state for special keycodes */
			x = s ^ kbd->special_state;

			/* Generate special keycodes for changed keys */
			for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
				if ((x & m) && jz_button[i].scode) {
					input_report_key(input, jz_button[i].scode, s & m);
					sync++;
				}
			}

			/* If power state just pressed, start counter, otherwise increase it */
			if (!kbd->power_state) kbd->power_count = 1;
			else {
				if (kbd->power_count > 0) {
					if (kbd->power_count == POWER_COUNT) {
						input_report_key(input, KEY_POWER, 1);
						input_report_key(input, KEY_POWER, 0);
						sync++;
					}
					if (kbd->power_count == POFF_COUNT)
					{
						prepare_for_restart();
						send_sig_to_init(SIGUSR2);
 					}
					kbd->power_count++;
				}
			}

			if (s) kbd->power_count = 0;	/* If any button pressed, stop counter */
			kbd->special_state = s;		/* Update current special button state */
			kbd->power_state = p;		/* Update power slider state */
		}
#endif
	}

	/* If power slider is NOT pressed... */
	else {

		/* Calculate changed button state for normal keycodes */
		x = s ^ kbd->normal_state;

		/* Generate normal keycodes for changed keys */
		for (i = 0, m = 1; i < ARRAY_SIZE(jz_button); i++, m <<= 1) {
			if ((x & m) && jz_button[i].ncode) {
				input_report_key(input, jz_button[i].ncode, s & m);
				sync++;
			}
		}

		kbd->power_count = 0;	/* Stop power slider pressed counter */
		kbd->normal_state = s;	/* Update current normal button state */
		kbd->power_state = p;	/* Update power slider state */
	}

	/* Synchronize input if any keycodes sent */
	if (sync) input_sync(input);

	/* TODO: unlock */
	}
}

static struct jz_kbd jz_kbd;

static int __init jz_kbd_init(void)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input_dev;
	int i, j, error;

	printk("jz-gpio-keys: scan interval %ums\n", SCAN_INTERVAL);

	//Maddrone: add gpio init
	for(i=0; i<ARRAY_SIZE(jz_button); i++)
	{
          if(i != 4)
          {
            __gpio_as_input(jz_button[i].gpio);
            __gpio_enable_pull(jz_button[i].gpio);
          }
	}
        //Nathan: add GPD18 init
        __gpio_as_input(HOLD_DETET);
        __gpio_enable_pull(HOLD_DETET);


	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		error = -ENOMEM;
		goto fail;
	}

	jz_kbd.poll_dev = poll_dev;

	poll_dev->private = &jz_kbd;
	poll_dev->poll = jz_kbd_poll;
	poll_dev->poll_interval = SCAN_INTERVAL;

	input_dev = poll_dev->input;
	input_dev->name = "JZ GPIO keys";
	input_dev->phys = "jz-gpio-keys/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	/* Prepare active low mask and keycode array/bits */
	for (i = j = 0; i < ARRAY_SIZE(jz_button); i++) {
		if (jz_button[i].actlow) jz_kbd.actlow |= 1 << i;
		if (jz_button[i].ncode) {
			jz_kbd.keycode[j++] = jz_button[i].ncode;
			__set_bit(jz_button[i].ncode, input_dev->keybit);
		}
		if (jz_button[i].scode) {
			jz_kbd.keycode[j++] = jz_button[i].scode;
			__set_bit(jz_button[i].scode, input_dev->keybit);
		}
	}

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP) | BIT_MASK(EV_SYN);
	input_dev->keycode = jz_kbd.keycode;
	input_dev->keycodesize = sizeof(jz_kbd.keycode[0]);
	input_dev->keycodemax = j;

	error = input_register_polled_device(jz_kbd.poll_dev);
	if (error) goto fail;

//maddrone add
	struct proc_dir_entry *res;
	res = create_proc_entry("jz/alt", 0, NULL);
	if(res)
	{
		//res->owner = THIS_MODULE;
		res->read_proc = proc_alt_read_proc;
		res->write_proc = proc_alt_write_proc;
	}

	return 0;

 fail:	input_free_polled_device(poll_dev);
	return error;
}

static void __exit jz_kbd_exit(void)
{
	input_unregister_polled_device(jz_kbd.poll_dev);
	input_free_polled_device(jz_kbd.poll_dev);
}

module_init(jz_kbd_init);
module_exit(jz_kbd_exit);

MODULE_AUTHOR("Ignacio Garcia Perez <iggarpe@gmail.com>");
MODULE_DESCRIPTION("JZ GPIO keys driver");
MODULE_LICENSE("GPLv2");

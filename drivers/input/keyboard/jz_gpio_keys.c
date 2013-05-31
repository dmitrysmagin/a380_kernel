/*
 *  Keyboard driver for the IngeniC JZ SoC
 *
 *  Copyright (c) 2009 Ignacio Garcia Perez <iggarpe@gmail.com>
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

//kaka porting here 

#include <linux/init.h>
#include <linux/input-polldev.h>
#include <linux/module.h>
#include <linux/sysrq.h>
#include <linux/sched.h>

#include <asm/jzsoc.h>

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

#ifdef CONFIG_JZ4750D_320E
	{ .gpio = 32*3+26,	.actlow = 1,	.ncode = KEY_UP,	.scode = KEY_VOLUMEUP,		.sysrq = 's'	}, /* D-pad up */
	{ .gpio = 32*4+19,	.actlow = 1, 	.ncode = KEY_DOWN,	.scode = KEY_VOLUMEDOWN,	.sysrq = 'u'	}, /* D-pad down */
	{ .gpio = 32*3+27,	.actlow = 1,	.ncode = KEY_LEFT,	.scode = KEY_BRIGHTNESSDOWN,	.sysrq = 'e'	}, /* D-pad left */
	{ .gpio = 32*4+18,	.actlow = 1,	.ncode = KEY_RIGHT,	.scode = KEY_BRIGHTNESSUP,	.sysrq = 'i'	}, /* D-pad right */
	{ .gpio = 32*3+17,	.actlow = 1,	.ncode = KEY_LEFTCTRL,							}, /* A button */
	{ .gpio = 32*4+0,	.actlow = 1,	.ncode = KEY_LEFTALT,							}, /* B button */
	{ .gpio = 32*2+23,	.actlow = 1,	.ncode = KEY_SPACE,							}, /* X button */
	{ .gpio = 32*2+24,	.actlow = 1,	.ncode = KEY_LEFTSHIFT,							}, /* Y button */
	{ .gpio = 32*3+25,	.actlow = 1,	.ncode = KEY_TAB,	.scode = KEY_EXIT				}, /* Left shoulder button */
	{ .gpio = 32*3+16,	.actlow = 1,	.ncode = KEY_BACKSPACE,							}, /* Right shoulder button */
	{ .gpio = 32*4+11,	.actlow = 1,	.ncode = KEY_ENTER,							}, /* START button(SYSRQ) */
	{ .gpio = 32*4+10,	.actlow = 1,	.ncode = KEY_ESC,	.scode = KEY_MENU,		.sysrq = 'b'	}, /* SELECT button */
	{ .gpio = 32*3+24,	.actlow = 1,	.ncode = KEY_PAUSE,							}, /* HOLD */

#define GPIO_POWER		(32*4+30)	/* Power slider */
#define GPIO_POWER_ACTLOW	1
#define SYSRQ_ALT		10	/* Alternate sysrq button (index in jz_button table) */
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
	struct task_struct *p;

	for_each_process(p) 
	{
		if (p->mm && !is_global_init(p)) /* without important stuff */
			force_sig(sig, p); 
	}
}

static void prepare_for_restart(void)
{
#ifdef CONFIG_JZ4750D_320E
	/* TODO : rewrite more smart code */
	__gpio_as_output(GPIO_LCD_PWM);
	__gpio_clear_pin(GPIO_LCD_PWM);
#endif

	send_sig_all(SIGTERM);
	send_sig_all(SIGKILL);
}

static void jz_kbd_poll (struct input_polled_dev *dev)
{
	struct jz_kbd *kbd = dev->private;
	struct input_dev *input = kbd->poll_dev->input;
	unsigned int i, p, sync = 0;
	unsigned long s, m, x;

	/* TODO: lock */
	
	/* Scan raw key states */
	for (s = 0, m = 1, i = 0; i < ARRAY_SIZE(jz_button); i++, m <<= 1)
		if (__gpio_get_pin(jz_button[i].gpio)) s |= m;

	/* Invert active low buttons */
	s ^= kbd->actlow;

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

static struct jz_kbd jz_kbd;

static int __init jz_kbd_init(void)
{
	struct input_polled_dev *poll_dev;
	struct input_dev *input_dev;
	int i, j, error;

	printk("jz-gpio-keys: scan interval %ums\n", SCAN_INTERVAL); 

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

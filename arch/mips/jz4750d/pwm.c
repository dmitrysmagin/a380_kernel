/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2012, Paul Cercueil <paul@crapouillou.net>
 *  JZ4750d platform PWM support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mutex.h>

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/gpio.h>

#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>

static struct clk *jz_pwm_clk;

DEFINE_MUTEX(jz4750d_pwm_mutex);

struct pwm_device {
	unsigned int id;
	unsigned int gpio;
	bool used;
};

static struct pwm_device jz4750d_pwm_list[] = {
	{ 0, JZ_GPIO_PORTE(20),  false },
	//{ 1, JZ_GPIO_PORTF(11),  false },
	{ 2, JZ_GPIO_PORTE(22),  false }, /* LCD power */
	{ 3, JZ_GPIO_PORTE(23),  false },
	{ 4, JZ_GPIO_PORTE(24),  false },
	{ 5, JZ_GPIO_PORTE(25),  false },
};

struct pwm_device *pwm_request(int id, const char *label)
{
	int ret = 0;
	struct pwm_device *pwm;

	if (id < 0 || id >= ARRAY_SIZE(jz4750d_pwm_list) || !jz_pwm_clk)
		return ERR_PTR(-ENODEV);

	mutex_lock(&jz4750d_pwm_mutex);

	pwm = &jz4750d_pwm_list[id];
	if (pwm->used)
		ret = -EBUSY;
	else
		pwm->used = true;

	mutex_unlock(&jz4750d_pwm_mutex);

	if (ret)
		return ERR_PTR(ret);

	ret = gpio_request(pwm->gpio, label);

	if (ret) {
		printk(KERN_ERR "Failed to request pwm gpio: %d\n", ret);
		pwm->used = false;
		return ERR_PTR(ret);
	}

	__gpio_as_func0(pwm->gpio);
	__tcu_start_timer_clock(id);

	return pwm;
}

void pwm_free(struct pwm_device *pwm)
{
	pwm_disable(pwm);

	__tcu_stop_timer_clock(pwm->id);
	__gpio_as_input(pwm->gpio);
	gpio_free(pwm->gpio);

	pwm->used = false;
}

int pwm_config(struct pwm_device *pwm, int duty_ns, int period_ns)
{
	unsigned long long tmp;
	unsigned long period, duty;
	unsigned int prescaler = 0;
	unsigned int id = pwm->id;

	if (duty_ns < 0 || duty_ns > period_ns)
		return -EINVAL;

	tmp = (unsigned long long) clk_get_rate(jz_pwm_clk) * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;

	while (period > 0xffff && prescaler < 6) {
		period >>= 2;
		++prescaler;
	}

	if (prescaler == 6)
		return -EINVAL;

	tmp = (unsigned long long) period * duty_ns;
	do_div(tmp, period_ns);
	duty = period - tmp;

	if (duty >= period)
		duty = period - 1;

	if (__tcu_counter_enabled(id))
		pwm_disable(pwm);

	__tcu_set_count(id, 0);
	__tcu_set_half_data(id, duty);
	__tcu_set_full_data(id, period);
	__tcu_select_extalclk(id);
	__tcu_select_clk_div(id, prescaler);
	__tcu_set_pwm_output_shutdown_abrupt(id);
	return 0;
}

int pwm_enable(struct pwm_device *pwm)
{
	__tcu_enable_pwm_output(pwm->id);
	__tcu_start_counter(pwm->id);
	return 0;
}

void pwm_disable(struct pwm_device *pwm)
{
	__tcu_stop_counter(pwm->id);
	__tcu_disable_pwm_output(pwm->id);
}

static int __init jz_pwm_init(void)
{
	int ret = 0;

	jz_pwm_clk = clk_get(NULL, "ext");

	if (IS_ERR(jz_pwm_clk)) {
		ret = PTR_ERR(jz_pwm_clk);
		jz_pwm_clk = NULL;
	}

	return ret;
}
subsys_initcall(jz_pwm_init);

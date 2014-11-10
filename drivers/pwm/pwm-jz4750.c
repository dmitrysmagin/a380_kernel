/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Copyright (C) 2012, Paul Cercueil <paul@crapouillou.net>
 *  Copyright (C) 2014, Dmitry Smagin <dmitry.s.smagin@gmail.com>
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
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/gpio.h>

#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/jz4750d_tcu.h>

#define NUM_PWM 6

static const unsigned int jz4750_pwm_gpio_list[NUM_PWM] = {
	JZ_GPIO_PORTE(20),
	JZ_GPIO_PORTF(11),
	JZ_GPIO_PORTE(22), /* LCD power */
	JZ_GPIO_PORTE(23),
	JZ_GPIO_PORTE(24),
	JZ_GPIO_PORTE(25),
};

struct jz4750_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
};

static inline struct jz4750_pwm_chip *to_jz4750(struct pwm_chip *chip)
{
	return container_of(chip, struct jz4750_pwm_chip, chip);
}

static int jz4750_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int gpio = jz4750_pwm_gpio_list[pwm->hwpwm];
	int ret;

	ret = gpio_request(gpio, pwm->label);
	if (ret) {
		dev_err(chip->dev, "Failed to request GPIO#%u for PWM: %d\n",
			gpio, ret);
		return ret;
	}

	/*
	 * CHECKIT: according to jz4750d_gpio.h PWM1 gpio has to be func1
	 * but __gpio_as_pwm(1) sets it to func0 though
	 */
	__gpio_as_func0(gpio);
	__gpio_disable_pull(gpio);

	__tcu_start_timer_clock(pwm->hwpwm);

	return 0;
}

static void jz4750_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int gpio = jz4750_pwm_gpio_list[pwm->hwpwm];

	__tcu_stop_timer_clock(pwm->hwpwm);
	__gpio_as_input(gpio);
	gpio_free(gpio);
}

static int jz4750_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	__tcu_enable_pwm_output(pwm->hwpwm);
	__tcu_start_counter(pwm->hwpwm);
	return 0;
}

static void jz4750_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	__tcu_stop_counter(pwm->hwpwm);
	__tcu_disable_pwm_output(pwm->hwpwm);
}

static int jz4750_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	struct jz4750_pwm_chip *jz4750 = to_jz4750(pwm->chip);
	unsigned int gpio = jz4750_pwm_gpio_list[pwm->hwpwm];
	unsigned long long tmp;
	unsigned long period, duty;
	unsigned int prescaler = 0;
	bool is_enabled;

	if (duty_ns < 0 || duty_ns > period_ns)
		return -EINVAL;

	tmp = (unsigned long long)clk_get_rate(jz4750->clk) * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;

	while (period > 0xffff && prescaler < 6) {
		period >>= 2;
		++prescaler;
	}

	if (prescaler == 6)
		return -EINVAL;

	tmp = (unsigned long long)period * duty_ns;
	do_div(tmp, period_ns);
	duty = tmp;

	if (duty >= period)
		duty = period - 1;

	is_enabled = __tcu_counter_enabled(pwm->hwpwm);
	if (is_enabled)
		jz4750_pwm_disable(chip, pwm);

	/* FIXME: check why removing these two lines
		  makes backlight non-working */
	__gpio_as_func0(gpio);
	__gpio_disable_pull(gpio);

	__tcu_init_pwm_output_high(pwm->hwpwm);
	__tcu_set_count(pwm->hwpwm, 0);
	__tcu_set_half_data(pwm->hwpwm, duty);
	__tcu_set_full_data(pwm->hwpwm, period);
	__tcu_select_extalclk(pwm->hwpwm);
	__tcu_select_clk_div(pwm->hwpwm, prescaler);
	__tcu_set_pwm_output_shutdown_abrupt(pwm->hwpwm);

	if (is_enabled)
		jz4750_pwm_enable(chip, pwm);

	return 0;
}

static const struct pwm_ops jz4750_pwm_ops = {
	.request	= jz4750_pwm_request,
	.free		= jz4750_pwm_free,
	.config		= jz4750_pwm_config,
	.enable		= jz4750_pwm_enable,
	.disable	= jz4750_pwm_disable,
	.owner		= THIS_MODULE,
};

static int __devinit jz4750_pwm_probe(struct platform_device *pdev)
{
	struct jz4750_pwm_chip *jz4750;
	int ret;

	jz4750 = devm_kzalloc(&pdev->dev, sizeof(*jz4750), GFP_KERNEL);
	if (!jz4750)
		return -ENOMEM;

	jz4750->clk = clk_get(NULL, "ext");
	if (IS_ERR(jz4750->clk))
		return PTR_ERR(jz4750->clk);

	jz4750->chip.dev = &pdev->dev;
	jz4750->chip.ops = &jz4750_pwm_ops;
	jz4750->chip.npwm = NUM_PWM;
	jz4750->chip.base = -1;

	ret = pwmchip_add(&jz4750->chip);
	if (ret < 0) {
		clk_put(jz4750->clk);
		return ret;
	}

	platform_set_drvdata(pdev, jz4750);

	return 0;
}

static int __devexit jz4750_pwm_remove(struct platform_device *pdev)
{
	struct jz4750_pwm_chip *jz4750 = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&jz4750->chip);
	if (ret < 0)
		return ret;

	clk_put(jz4750->clk);

	return 0;
}

static struct platform_driver jz4750_pwm_driver = {
	.driver	= {
		.name	= "jz4750-pwm",
		.owner	= THIS_MODULE,
	},
	.probe	= jz4750_pwm_probe,
	.remove	= __devexit_p(jz4750_pwm_remove),
};
module_platform_driver(jz4750_pwm_driver);

MODULE_AUTHOR("Paul Cercuei <paul@crapouillou.net>");
MODULE_DESCRIPTION("Ingenic JZ4750 PWM driver");
MODULE_ALIAS("platform:jz4750-pwm");
MODULE_LICENSE("GPL");

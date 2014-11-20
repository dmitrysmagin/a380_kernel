/*
 * a380.c  --  SoC audio for Dingoo A380
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define A380_HP_DETECT_GPIO	JZ_GPIO_PORTC(23)
#define A380_HP_GPIO		JZ_GPIO_PORTE(5)
#define A380_SPK_GPIO		JZ_GPIO_PORTE(9)

/* Headphone jzck: plug insert detection */

static struct snd_soc_jack a380_hp_jack;

static struct snd_soc_jack_pin a380_hp_jack_pins[] = {
	{
		.pin	= "Headphones",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Speakers",
		.mask	= SND_JACK_HEADPHONE,
		//.invert	= 1, // will be valid for RZX45
	},
};

static struct snd_soc_jack_gpio a380_hp_jack_gpios[] = {
	{
		.name		= "Headphones Detect",
		.report		= SND_JACK_HEADPHONE,
		.gpio		= A380_HP_DETECT_GPIO,
		.invert		= 1,
		.debounce_time	= 200,
	},
};

/* Headphones and speakers switches */

static int a380_spk_event(struct snd_soc_dapm_widget *widget,
	struct snd_kcontrol *ctrl, int event)
{
	printk("a380_spk_event: %d\n", SND_SOC_DAPM_EVENT_ON(event));

	gpio_set_value(A380_SPK_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static int a380_hp_event(
			struct snd_soc_dapm_widget *widget,
			struct snd_kcontrol *ctrl, int event)
{
	printk("a380_hp_event: %d\n", SND_SOC_DAPM_EVENT_ON(event));

	if (SND_SOC_DAPM_EVENT_ON(event))
		msleep(50);

	/*
	 * A380:  0 - hp off; 1 - hp on
	 * RZX50: 0 - hp on;  1 - hp off
	 */
	gpio_set_value(A380_HP_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static const struct snd_kcontrol_new a380_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speakers"),
	SOC_DAPM_PIN_SWITCH("Headphones"),
};

static const struct snd_soc_dapm_widget a380_widgets[] = {
//	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_SPK("Speakers", a380_spk_event),
	SND_SOC_DAPM_HP("Headphones", a380_hp_event),
};

static const struct snd_soc_dapm_route a380_routes[] = {
//	{"Mic", NULL, "MIC"},
	{"Speakers", NULL, "LOUT"},
	{"Speakers", NULL, "ROUT"},
	{"Headphones", NULL, "LHPOUT"},
	{"Headphones", NULL, "RHPOUT"},
};

static int a380_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	/* Set up Headphones plug detection */
	snd_soc_jack_new(codec, "Headphones Jack",
			 SND_JACK_HEADPHONE, &a380_hp_jack);

	snd_soc_jack_add_pins(&a380_hp_jack,
			      ARRAY_SIZE(a380_hp_jack_pins),
			      a380_hp_jack_pins);

	snd_soc_jack_add_gpios(&a380_hp_jack,
			       ARRAY_SIZE(a380_hp_jack_gpios),
			       a380_hp_jack_gpios);

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
					   SND_SOC_DAIFMT_NB_NF | 
					   SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_dai_link a380_dai = {
	.name		= "jz4750",
	.stream_name	= "jz4750",
	.cpu_dai_name	= "jz4750-i2s",
	.platform_name	= "jz4750-pcm-audio",
	.codec_dai_name	= "jz4750-hifi",
	.codec_name	= "jz4750-codec",
	.init		= a380_codec_init,
};

static struct snd_soc_card a380 = {
	.name		= "Dingoo A380",
	.owner		= THIS_MODULE,
	.dai_link	= &a380_dai,
	.num_links	= 1,

	.controls	= a380_controls,
	.num_controls	= ARRAY_SIZE(a380_controls),

	.dapm_widgets		= a380_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(a380_widgets),

	.dapm_routes		= a380_routes,
	.num_dapm_routes	= ARRAY_SIZE(a380_routes),
};

static int a380_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &a380;
	int ret;

	ret = gpio_request(A380_SPK_GPIO, "SPK");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request SPK GPIO(%d): %d\n",
			A380_SPK_GPIO, ret);
		return ret;
	}

	ret = gpio_request(A380_HP_GPIO, "HP");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request HP GPIO(%d): %d\n",
			A380_HP_GPIO, ret);
		goto err_gpio_free_spk;
	}

	jz_gpio_enable_pullup(A380_HP_DETECT_GPIO);
	jz_gpio_enable_pullup(A380_SPK_GPIO);
	jz_gpio_enable_pullup(A380_HP_GPIO);

	//gpio_direction_input(A380_HP_DETECT_GPIO);
	gpio_direction_output(A380_SPK_GPIO, 0);
	gpio_direction_output(A380_HP_GPIO, 0);

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed :%d\n",
			ret);
		goto err_gpio_free_hptv;
	}

	return 0;

err_gpio_free_hptv:
	gpio_free(A380_HP_GPIO);
err_gpio_free_spk:
	gpio_free(A380_SPK_GPIO);

	return ret;
}

static int a380_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	gpio_free(A380_HP_GPIO);
	gpio_free(A380_SPK_GPIO);
	return 0;
}

static struct platform_driver a380_driver = {
	.driver		= {
		.name	= "a380-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= a380_probe,
	.remove		= a380_remove,
};

module_platform_driver(a380_driver);

MODULE_AUTHOR("Richard, <cjfeng@ingenic.cn>");
MODULE_DESCRIPTION("ALSA SoC Dingoo A380 Audio support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:a380-audio");

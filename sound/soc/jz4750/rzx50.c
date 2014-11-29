/*
 * rzx50.c  --  SoC audio for Ritmix RZX-50
 *
 * Based on APUS JZ4750 board
 * Copyright (C) Ingenic Semiconductor Inc.
 *
 * Dingoo A380 specific board support:
 * Copyright (C) 2012, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2014, Dmitry Smagin <dmitry.s.smagin@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

/* FIXME: Move it to platform data? */
#define RZX50_HP_DETECT_GPIO	JZ_GPIO_PORTE(9)
#define RZX50_HP_GPIO		JZ_GPIO_PORTE(2)
#define RZX50_SPK_GPIO		JZ_GPIO_PORTE(5)

/* Headphone jack: plug insert detection */

static struct snd_soc_jack rzx50_hp_jack;

static struct snd_soc_jack_pin rzx50_hp_jack_pins[] = {
	{
		.pin	= "Headphones",
		.mask	= SND_JACK_HEADPHONE,
	},
	{
		.pin	= "Speakers",
		.mask	= SND_JACK_HEADPHONE,
		.invert	= 1,
	},
};

static struct snd_soc_jack_gpio rzx50_hp_jack_gpios[] = {
	{
		.name		= "Headphones Detect",
		.report		= SND_JACK_HEADPHONE,
		.gpio		= RZX50_HP_DETECT_GPIO,
		.invert		= 1,
		.debounce_time	= 200,
	},
};

/* Headphones and speakers switches */

static int rzx50_spk_event(struct snd_soc_dapm_widget *widget,
	struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(RZX50_SPK_GPIO, !!SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static int rzx50_hp_event(
			struct snd_soc_dapm_widget *widget,
			struct snd_kcontrol *ctrl, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		msleep(50);

	gpio_set_value(RZX50_HP_GPIO, !SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static const struct snd_soc_dapm_widget rzx50_widgets[] = {
	SND_SOC_DAPM_MIC("Built-in Mic", NULL),
	SND_SOC_DAPM_LINE("FM Radio", NULL),
	SND_SOC_DAPM_SPK("Speakers", rzx50_spk_event),
	SND_SOC_DAPM_HP("Headphones", rzx50_hp_event),
};

static const struct snd_soc_dapm_route rzx50_routes[] = {
	{"MIC", NULL, "Built-in Mic"},
	{"LIN", NULL, "FM Radio" },
	{"RIN", NULL, "FM Radio" },
	{"Speakers", NULL, "LOUT"},
	{"Speakers", NULL, "ROUT"},
	{"Headphones", NULL, "LHPOUT"},
	{"Headphones", NULL, "RHPOUT"},
};

static int rzx50_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	/* Set up Headphones plug detection */
	snd_soc_jack_new(codec, "Headphones Jack",
			 SND_JACK_HEADPHONE, &rzx50_hp_jack);

	snd_soc_jack_add_pins(&rzx50_hp_jack,
			      ARRAY_SIZE(rzx50_hp_jack_pins),
			      rzx50_hp_jack_pins);

	snd_soc_jack_add_gpios(&rzx50_hp_jack,
			       ARRAY_SIZE(rzx50_hp_jack_gpios),
			       rzx50_hp_jack_gpios);

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
					   SND_SOC_DAIFMT_NB_NF | 
					   SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_dai_link rzx50_dai = {
	.name		= "jz4750",
	.stream_name	= "jz4750",
	.cpu_dai_name	= "jz4750-i2s",
	.platform_name	= "jz4750-pcm-audio",
	.codec_dai_name	= "jz4750-hifi",
	.codec_name	= "jz4750-codec",
	.init		= rzx50_codec_init,
};

static struct snd_soc_card a380 = {
	.name		= "Ritmix RZX-50",
	.owner		= THIS_MODULE,
	.dai_link	= &rzx50_dai,
	.num_links	= 1,

	.dapm_widgets		= rzx50_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(rzx50_widgets),

	.dapm_routes		= rzx50_routes,
	.num_dapm_routes	= ARRAY_SIZE(rzx50_routes),
};

static int rzx50_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &a380;
	int ret;

	ret = gpio_request(RZX50_SPK_GPIO, "SPK");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request SPK GPIO(%d): %d\n",
			RZX50_SPK_GPIO, ret);
		return ret;
	}

	ret = gpio_request(RZX50_HP_GPIO, "HP");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request HP GPIO(%d): %d\n",
			RZX50_HP_GPIO, ret);
		goto err_gpio_free_spk;
	}

	jz_gpio_enable_pullup(RZX50_SPK_GPIO);
	jz_gpio_enable_pullup(RZX50_HP_GPIO);
	jz_gpio_enable_pullup(RZX50_HP_DETECT_GPIO);

	gpio_direction_output(RZX50_SPK_GPIO, 0);
	gpio_direction_output(RZX50_HP_GPIO, 0);

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed :%d\n",
			ret);
		goto err_gpio_free_hptv;
	}

	return 0;

err_gpio_free_hptv:
	gpio_free(RZX50_HP_GPIO);
err_gpio_free_spk:
	gpio_free(RZX50_SPK_GPIO);

	return ret;
}

static int rzx50_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	gpio_free(RZX50_HP_GPIO);
	gpio_free(RZX50_SPK_GPIO);
	return 0;
}

static struct platform_driver rzx50_driver = {
	.driver		= {
		.name	= "rzx50-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= rzx50_probe,
	.remove		= rzx50_remove,
};

module_platform_driver(rzx50_driver);

MODULE_AUTHOR("Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("ALSA SoC Ritmix RZX-50 Audio support");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:a380-audio");

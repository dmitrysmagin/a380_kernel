/*
 * Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2010-2011, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2011, Paul Cercueil <paul@crapouillou.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/gpio.h>

#define A320_SPK_GPIO JZ_GPIO_PORTC(27)
#define A320_HPTV_GPIO JZ_GPIO_PORTD(7)

static int a320_spk_event(struct snd_soc_dapm_widget *widget,
			  struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(A320_SPK_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static int a320_hptv_event(
			struct snd_soc_dapm_widget *widget,
			struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(A320_HPTV_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static const struct snd_kcontrol_new a320_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphones + TV-out"),
};

static const struct snd_soc_dapm_widget a320_widgets[] = {
	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", a320_spk_event),
	SND_SOC_DAPM_LINE("Headphones + TV-out", a320_hptv_event),
};

static const struct snd_soc_dapm_route a320_routes[] = {
	{"Mic", NULL, "MIC"},
	{"Speaker", NULL, "LOUT"},
	{"Speaker", NULL, "ROUT"},
	{"Headphones + TV-out", NULL, "LOUT"},
	{"Headphones + TV-out", NULL, "ROUT"},
};

#define A320_DAIFMT (SND_SOC_DAIFMT_I2S | \
		     SND_SOC_DAIFMT_NB_NF | \
		     SND_SOC_DAIFMT_CBM_CFM)

static int a320_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, A320_DAIFMT);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	snd_soc_add_card_controls(rtd->card, a320_controls,
				  ARRAY_SIZE(a320_controls));

	snd_soc_dapm_new_controls(dapm, a320_widgets, ARRAY_SIZE(a320_widgets));
	snd_soc_dapm_add_routes(dapm, a320_routes, ARRAY_SIZE(a320_routes));
	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link a320_dai = {
	.name = "jz4740",
	.stream_name = "jz4740",
	.cpu_dai_name = "jz4740-i2s",
	.platform_name = "jz4740-pcm-audio",
	.codec_dai_name = "jz4740-hifi",
	.codec_name = "jz4740-codec",
	.init = a320_codec_init,
};

static struct snd_soc_card a320 = {
	.name = "Dingoo A320",
	.owner = THIS_MODULE,
	.dai_link = &a320_dai,
	.num_links = 1,
};

static int __devinit a320_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &a320;
	int ret;

	ret = gpio_request(A320_SPK_GPIO, "SPK");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request SPK GPIO(%d): %d\n",
			A320_SPK_GPIO, ret);
		return ret;
	}

	ret = gpio_request(A320_HPTV_GPIO, "HPTV");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request HPTV GPIO(%d): %d\n",
			A320_HPTV_GPIO, ret);
		goto err_gpio_free_spk;
	}

	gpio_direction_output(A320_SPK_GPIO, 0);
	gpio_direction_output(A320_HPTV_GPIO, 0);

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		goto err_gpio_free_hptv;
	}

	return 0;

err_gpio_free_hptv:
	gpio_free(A320_HPTV_GPIO);
err_gpio_free_spk:
	gpio_free(A320_SPK_GPIO);

	return ret;
}

static int __devexit a320_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	gpio_free(A320_HPTV_GPIO);
	gpio_free(A320_SPK_GPIO);
	return 0;
}

static struct platform_driver a320_driver = {
	.driver		= {
		.name	= "a320-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= a320_probe,
	.remove		= __devexit_p(a320_remove),
};

module_platform_driver(a320_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>, Maarten ter Huurne <maarten@treewalker.org>, Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("ALSA SoC Dingoo A320 Audio support");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:a320-audio");

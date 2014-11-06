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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <linux/gpio.h>

#define A380_SPK_GPIO JZ_GPIO_PORTE(9)
#define A380_HPTV_GPIO JZ_GPIO_PORTE(5)

static int a380_spk_event(struct snd_soc_dapm_widget *widget,
	struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(A380_SPK_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static int a380_hptv_event(
			struct snd_soc_dapm_widget *widget,
			struct snd_kcontrol *ctrl, int event)
{
	gpio_set_value(A380_HPTV_GPIO, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

static const struct snd_kcontrol_new a380_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphones + TV-out"),
};

static const struct snd_soc_dapm_widget a380_widgets[] = {
	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", a380_spk_event),
	SND_SOC_DAPM_LINE("Headphones + TV-out", a380_hptv_event),
};

static const struct snd_soc_dapm_route a380_routes[] = {
	{"Mic", NULL, "MIC"},
	{"Speaker", NULL, "LOUT"},
	{"Speaker", NULL, "ROUT"},
	{"Headphones + TV-out", NULL, "LOUT"},
	{"Headphones + TV-out", NULL, "ROUT"},
};

#define A380_DAIFMT (SND_SOC_DAIFMT_I2S | \
		     SND_SOC_DAIFMT_NB_NF | \
		     SND_SOC_DAIFMT_CBM_CFM)

static int a380_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, A380_DAIFMT);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	snd_soc_add_card_controls(rtd->card, a380_controls,
				  ARRAY_SIZE(a380_controls));

	snd_soc_dapm_new_controls(dapm, a380_widgets, ARRAY_SIZE(a380_widgets));
	snd_soc_dapm_add_routes(dapm, a380_routes, ARRAY_SIZE(a380_routes));
	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link a380_dai = {
	.name = "jz4750",
	.stream_name = "jz4750",
	.cpu_dai_name = "jz4750-i2s",
	.platform_name = "jz4750-pcm-audio",
	.codec_dai_name = "jz4750-hifi",
	.codec_name = "jz4750-codec",
	.init = a380_codec_init,
};

static struct snd_soc_card a380 = {
	.name = "Dingoo A380",
	.dai_link = &a380_dai,
	.num_links = 1,
};

static int __devinit a380_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &a380;
	int ret;

	ret = gpio_request(A380_SPK_GPIO, "SPK");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request SPK GPIO(%d): %d\n",
			A380_SPK_GPIO, ret);
		return ret;
	}

	ret = gpio_request(A380_HPTV_GPIO, "HPTV");
	if (ret) {
		dev_err(&pdev->dev, "Failed to request HPTV GPIO(%d): %d\n",				A380_HPTV_GPIO, ret);
		goto err_gpio_free_spk;
	}

	gpio_direction_output(A380_SPK_GPIO, 0);
	gpio_direction_output(A380_HPTV_GPIO, 0);

	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed :%d\n",
			ret);
		goto err_gpio_free_hptv;
	}

	return 0;

err_gpio_free_hptv:
	gpio_free(A380_HPTV_GPIO);
err_gpio_free_spk:
	gpio_free(A380_SPK_GPIO);

	return ret;
}

static int __devexit a380_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	gpio_free(A380_HPTV_GPIO);
	gpio_free(A380_SPK_GPIO);
	return 0;
}

static struct platform_driver a380_driver = {
	.driver		= {
		.name	= "a380-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= a380_probe,
	.remove		= __devexit_p(a380_remove),
};

module_platform_driver(a380_driver);

MODULE_AUTHOR("Richard, <cjfeng@ingenic.cn>");
MODULE_DESCRIPTION("ALSA SoC Dingoo A380 Audio support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:a380-audio");

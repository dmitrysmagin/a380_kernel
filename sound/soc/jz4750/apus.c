/*
 * apus.c  --  SoC audio for APUS
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
#include <sound/soc-dapm.h>
#include <linux/gpio.h>

#include "../codecs/jzdlv.h"
#include "jz4750-pcm.h"
#include "jz4750-i2s.h"

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
	int ret;

	snd_soc_dapm_nc_pin(codec, "LIN");
	snd_soc_dapm_nc_pin(codec, "RIN");
	
	ret = snd_soc_dai_set_fmt(cpu_dai, A380_DAIFMT);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	snd_soc_add_controls(codec, a380_controls, ARRAY_SIZE(a380_controls));

	snd_soc_dapm_new_controls(codec, a380_widgets, ARRAY_SIZE(a380_widgets));
	snd_soc_dapm_add_routes(codec, a380_routes, ARRAY_SIZE(a380_routes));
	snd_soc_dapm_sync(codec);

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

static struct platform_device *a380_snd_device;

static int __init a380_init(void)
{
	int ret;

	a380_snd_device = platform_device_alloc("soc-audio", -1);

	if (!a380_snd_device)
		return -ENOMEM;

	ret = gpio_request(A380_SPK_GPIO, "SPK");
	if (ret) {
		pr_err("a380 snd: Failed to request SND GPIO(%d): %d\n",
				A380_SPK_GPIO, ret);
		goto err_device_put;
	}

	ret = gpio_request(A380_HPTV_GPIO, "HPTV");
	if (ret) {
		pr_err("a380 snd: Failed to request AMP GPIO(%d): %d\n",
				A380_HPTV_GPIO, ret);
		goto err_gpio_free_spk;
	}

	gpio_direction_output(A380_SPK_GPIO, 0);
	gpio_direction_output(A380_HPTV_GPIO, 0);

	platform_set_drvdata(a380_snd_device, &a380);

	ret = platform_device_add(a380_snd_device);
	if (ret) {
		pr_err("a380 snd: Failed to add snd soc device: %d\n", ret);
		goto err_unset_pdata;
	}

	 return 0;

err_unset_pdata:
	platform_set_drvdata(a380_snd_device, NULL);
/*err_gpio_free_hptv:*/
	gpio_free(A380_HPTV_GPIO);
err_gpio_free_spk:
	gpio_free(A380_SPK_GPIO);
err_device_put:
	platform_device_put(a380_snd_device);

	return ret;
}
module_init(a380_init);

static void __exit a380_exit(void)
{
	gpio_free(A380_HPTV_GPIO);
	gpio_free(A380_SPK_GPIO);
	platform_device_unregister(a380_snd_device);
}
module_exit(a380_exit);

MODULE_AUTHOR("Richard");
MODULE_DESCRIPTION("ALSA SoC Apus");
MODULE_LICENSE("GPL");

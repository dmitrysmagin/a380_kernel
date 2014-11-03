/*
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/mach-jz4750d/jz4750d_aic.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_dmac.h>
#include <asm/mach-jz4750d/dma.h>

#include "jz4750-pcm.h"

/* I2S clock */
#define JZ4750_I2S_SYSCLK		0

static int jz_i2s_debug = 1;
module_param(jz_i2s_debug, int, 0644);
#define I2S_DEBUG_MSG(msg...)			\
	do {					\
		if (jz_i2s_debug)		\
			printk("I2S: " msg);	\
	} while(0)

struct jz4750_i2s {
	struct resource *mem;
	void __iomem *base;
#if 0
	dma_addr_t phys_base;

	struct clk *clk_aic;
	struct clk *clk_i2s;

	struct jz4740_pcm_config pcm_config_playback;
	struct jz4740_pcm_config pcm_config_capture;
#endif
};

static struct jz4750_pcm_dma_params jz4750_i2s_pcm_stereo_out = {
	.channel	= DMA_ID_AIC_TX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static struct jz4750_pcm_dma_params jz4750_i2s_pcm_stereo_in = {
	.channel	= DMA_ID_AIC_RX,
	.dma_addr	= AIC_DR,
	.dma_size	= 2,
};

static void jz4750_snd_tx_ctrl(int on)
{
	I2S_DEBUG_MSG("enter %s, on = %d\n", __func__, on);
	if (on) { 
                /* enable replay */
	        __i2s_enable_transmit_dma();
		__i2s_enable_replay();
		__i2s_enable();

	} else {
		/* disable replay & capture */
		__i2s_disable_replay();
		__i2s_disable_record();
		__i2s_disable_receive_dma();
		__i2s_disable_transmit_dma();
		__i2s_disable();
	}
}

static void jz4750_snd_rx_ctrl(int on)
{
	I2S_DEBUG_MSG("enter %s, on = %d\n", __func__, on);
	if (on) { 
                /* enable capture */
		__i2s_enable_receive_dma();
		__i2s_enable_record();
		__i2s_enable();

	} else { 
                /* disable replay & capture */
		__i2s_disable_replay();
		__i2s_disable_record();
		__i2s_disable_receive_dma();
		__i2s_disable_transmit_dma();
		__i2s_disable();
	}
}

static int jz4750_i2s_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	return 0;
}

static void jz4750_i2s_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
	} else {
	}

	return;
}

static int jz4750_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int ret = 0;

	I2S_DEBUG_MSG("enter %s, substream = %s cmd = %d\n",
		      __func__,
		      (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				? "playback" : "capture",
		      cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		//__aic_flush_fifo_tx(); // ?
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jz4750_snd_rx_ctrl(1);
		else
			jz4750_snd_tx_ctrl(1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			jz4750_snd_rx_ctrl(0);
		else
			jz4750_snd_tx_ctrl(0);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int jz4750_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	int channels = params_channels(params);

	jz4750_snd_rx_ctrl(0);
	jz4750_snd_rx_ctrl(0);

	I2S_DEBUG_MSG("enter %s, substream = %s\n",
		      __func__,
		      (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture");

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_dai_set_dma_data(dai, substream,
					&jz4750_i2s_pcm_stereo_out);
		
		if (channels == 1)
			__aic_enable_mono2stereo();
		else
			__aic_disable_mono2stereo();
	} else
		snd_soc_dai_set_dma_data(dai, substream,
					&jz4750_i2s_pcm_stereo_in);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		__i2s_set_transmit_trigger(4);
		__i2s_set_receive_trigger(3);
		__i2s_set_oss_sample_size(8);
		__i2s_set_iss_sample_size(8);
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		/* playback sample:16 bits, burst:16 bytes */
		__i2s_set_transmit_trigger(4);
		/* capture sample:16 bits, burst:16 bytes */
		__i2s_set_receive_trigger(3);
		__i2s_set_oss_sample_size(16);
		__i2s_set_iss_sample_size(16);
		break;
	}

	return 0;
}

static int jz4750_i2s_set_dai_fmt(struct snd_soc_dai *dai,
	unsigned int fmt)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* 1 : ac97 , 0 : i2s */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
	        /* 0 : slave */
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/* 1 : master */
		break;
	default:
		break;
	}

	return 0;
}

static int jz4750_i2s_set_dai_sysclk(struct snd_soc_dai *dai,
	int clk_id, unsigned int freq, int dir)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	return 0;
}

static int jz4750_i2s_dai_probe(struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	//clk_enable(i2s->clk_aic);

	//jz4750_i2c_init_pcm_config(i2s);

	__cpm_select_i2sclk_exclk();
	__cpm_start_aic();
	__i2s_enable_sysclk();

	__i2s_disable();
	__aic_disable_transmit_dma();
	__aic_disable_receive_dma();
	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();

	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();
        __aic_play_lastsample();
	__i2s_set_transmit_trigger(7);
	__i2s_set_receive_trigger(7);

	__aic_write_tfifo(0x0);
	__aic_write_tfifo(0x0);
	__i2s_enable_replay();
	__i2s_enable();
	mdelay(1);

	jz4750_snd_tx_ctrl(0);
	jz4750_snd_rx_ctrl(0);

	return 0;
}

static int jz4750_i2s_dai_remove(struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	//clk_disable(i2s->clk_aic);
	return 0;
}

#ifdef CONFIG_PM
static int jz4750_i2s_dai_suspend(struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (!dai->active)
		return 0;

	return 0;
}

static int jz4750_i2s_dai_resume(struct snd_soc_dai *dai)
{
	//struct jz4750_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (!dai->active)
		return 0;

	return 0;
}

#else
#define jz4750_i2s_dai_suspend	NULL
#define jz4750_i2s_dai_resume	NULL
#endif

static struct snd_soc_dai_ops jz4750_i2s_dai_ops = {
	.startup = jz4750_i2s_startup,
	.shutdown = jz4750_i2s_shutdown,
	.trigger = jz4750_i2s_trigger,
	.hw_params = jz4750_i2s_hw_params,
	.set_fmt = jz4750_i2s_set_dai_fmt,
	.set_sysclk = jz4750_i2s_set_dai_sysclk,
};

#define JZ4750_I2S_FMTS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_driver jz4750_i2s_dai = {
	.probe = jz4750_i2s_dai_probe,
	.remove = jz4750_i2s_dai_remove,
	.suspend = jz4750_i2s_dai_suspend,
	.resume = jz4750_i2s_dai_resume,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = JZ4750_I2S_FMTS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = JZ4750_I2S_FMTS,
	},
	.ops = &jz4750_i2s_dai_ops,
};

static int __devinit jz4750_i2s_dev_probe(struct platform_device *pdev)
{
	struct jz4750_i2s *i2s;
	int ret;

	i2s = kzalloc(sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!i2s->mem) {
		dev_err(&pdev->dev, "Failed to get DAI registers resource\n");
		ret = -ENOENT;
		goto err_free;
	}

	i2s->mem = request_mem_region(i2s->mem->start, resource_size(i2s->mem),
				pdev->name);
	if (!i2s->mem) {
		ret = -EBUSY;
		goto err_free;
	}

	i2s->base = ioremap_nocache(i2s->mem->start, resource_size(i2s->mem));
	if (!i2s->base) {
		dev_err(&pdev->dev, "Failed to request and map DAI registers\n");
		ret = -EBUSY;
		goto err_release_mem_region;
	}
#if 0
	i2s->phys_base = i2s->mem->start;

	i2s->clk_aic = devm_clk_get(&pdev->dev, "aic");
	if (IS_ERR(i2s->clk_aic)) {
		ret = PTR_ERR(i2s->clk_aic);
		goto err_iounmap;
	}

	i2s->clk_i2s = devm_clk_get(&pdev->dev, "i2s");
	if (IS_ERR(i2s->clk_i2s)) {
		ret = PTR_ERR(i2s->clk_i2s);
		goto err_clk_put_aic;
	}
#endif
	platform_set_drvdata(pdev, i2s);

	ret = snd_soc_register_dai(&pdev->dev, &jz4750_i2s_dai);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register DAI\n");
		return ret;
	}

	return 0;
#if 0
err_clk_put_i2s:
	clk_put(i2s->clk_i2s);
err_clk_put_aic:
	clk_put(i2s->clk_aic);
err_iounmap:
#endif
	iounmap(i2s->base);
err_release_mem_region:
	release_mem_region(i2s->mem->start, resource_size(i2s->mem));
err_free:
	kfree(i2s);

	return ret;
}

static int __devexit jz4750_i2s_dev_remove(struct platform_device *pdev)
{
	struct jz4750_i2s *i2s = platform_get_drvdata(pdev);

	snd_soc_unregister_dai(&pdev->dev);
#if 0
	clk_put(i2s->clk_i2s);
	clk_put(i2s->clk_aic);
#endif
	iounmap(i2s->base);
	release_mem_region(i2s->mem->start, resource_size(i2s->mem));

	platform_set_drvdata(pdev, NULL);
	kfree(i2s);

	return 0;
}

static struct platform_driver jz4750_i2s_driver = {
	.probe = jz4750_i2s_dev_probe,
	.remove = __devexit_p(jz4750_i2s_dev_remove),
	.driver = {
		.name = "jz4750-i2s",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(jz4750_i2s_driver);

MODULE_AUTHOR("Richard, <cjfeng@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic JZ4750 SoC I2S driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4750-i2s");

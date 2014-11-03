/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/initval.h>

#include <asm/mach-jz4750d/jz4750d_aic.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>

#include "../jz4750/jz4750-pcm.h"
#include "jz4750.h"

/* JZ4750 codec register space */
#define DLV_AICR    0x00
#define DLV_CR1     0x01
#define DLV_CR2     0x02
#define DLV_CCR1    0x03
#define DLV_CCR2    0x04
#define DLV_PMR1    0x05
#define DLV_PMR2    0x06
#define DLV_CRR     0x07
#define DLV_ICR     0x08
#define DLV_IFR     0x09
#define DLV_CGR1    0x0a
#define DLV_CGR2    0x0b
#define DLV_CGR3    0x0c
#define DLV_CGR4    0x0d
#define DLV_CGR5    0x0e
#define DLV_CGR6    0x0f
#define DLV_CGR7    0x10
#define DLV_CGR8    0x11
#define DLV_CGR9    0x12
#define DLV_CGR10   0x13
#define DLV_TR1     0x14
#define DLV_TR2     0x15
#define DLV_CR3     0x16
#define DLV_AGC1    0x17
#define DLV_AGC2    0x18
#define DLV_AGC3    0x19
#define DLV_AGC4    0x1a
#define DLV_AGC5    0x1b

#define JZDLV_CACHEREGNUM  (DLV_AGC5+1)
#define JZDLV_SYSCLK	0

static const uint8_t jz4750_codec_regs[JZDLV_CACHEREGNUM] = {
	0x0C, 0xAA, 0x78, 0x00, 0x00, 0xFF, 0x03, 0x51,
	0x3F, 0x00, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x0A, 0x0A, 0x00, 0xC0, 0x34, 0x07, 0x44,
	0x1F, 0x00
};

struct jz4750_codec {
	void __iomem *base;
	struct resource *mem;

	unsigned int sysclk;
};

int read_codec_file(int addr)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	mdelay(1);
	return(__icdc_get_value());
}

void write_codec_file(int addr, int val)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(addr);
	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);
}
EXPORT_SYMBOL(write_codec_file);

int write_codec_file_bit(int addr, int bitval, int mask_bit)
{
	int val;

	val = read_codec_file(addr);
	val &= ~(1 << mask_bit);

	if (bitval == 1)
		val |= 1 << mask_bit;

	write_codec_file(addr, val);
	val = read_codec_file(addr);

	if (((val >> mask_bit) & bitval) == bitval)
		return 1;
	else 
		return 0;
}
EXPORT_SYMBOL(write_codec_file_bit);

static inline unsigned int jzdlv_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	//struct jz4750_codec *jz4750_codec = snd_soc_codec_get_drvdata(codec);
	uint8_t *reg_cache = codec->reg_cache;

	if (reg >= JZDLV_CACHEREGNUM)
		return -1;

	return reg_cache[reg];
}

static inline void jzdlv_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, u16 value)
{
	//struct jz4750_codec *jz4750_codec = snd_soc_codec_get_drvdata(codec);
	uint8_t *reg_cache = codec->reg_cache;

	if (reg >= JZDLV_CACHEREGNUM) {
		return;
	}

	reg_cache[reg] = value;
}

static int jzdlv_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	jzdlv_write_reg_cache(codec, reg, value);
	write_codec_file(reg, value);

	return 0;
}

void set_audio_data_replay(void)
{
	write_codec_file(9, 0xff);
	write_codec_file(8, 0x20);// only CCMC
	mdelay(10);

	write_codec_file_bit(1, 0, 4);//CR1.HP_DIS->0
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1

	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 1, 3);//CR1.DACSEL->1

	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	mdelay(100);
	write_codec_file_bit(1, 0, 5);//DAC_MUTE->0
}

void unset_audio_data_replay(void)
{
	write_codec_file_bit(1, 1, 5);//DAC_MUTE->1
	mdelay(200);
	write_codec_file_bit(5, 1, 6);//SB_OUT->1
	write_codec_file_bit(5, 1, 7);//SB_DAC->1
	write_codec_file_bit(5, 1, 4);//SB_MIX->1
	write_codec_file_bit(6, 1, 0);//SB_SLEEP->1
	write_codec_file_bit(6, 1, 1);//SB->1

	write_codec_file(9, 0xff);
	write_codec_file(8, 0x3f);
}

static void set_record_mic_input_audio_without_playback(void)
{
	/* ADC path for MIC IN */
	write_codec_file_bit(1, 1, 2);
	write_codec_file_bit(1, 0, 7);//CR1.SB_MICBIAS->0
	//write_codec_file_bit(1, 1, 6);//CR1.MONO->1

	write_codec_file(22, 0x40);//mic 1
	write_codec_file_bit(23, 0, 7);//AGC1.AGC_EN->0
	write_codec_file_bit(3, 1, 7);//CR1.HP_DIS->1
	write_codec_file_bit(5, 1, 3);//PMR1.SB_LIN->1
	write_codec_file_bit(5, 1, 0);//PMR1.SB_IND->1

	write_codec_file_bit(1, 0, 2);//CR1.BYPASS->0
	write_codec_file_bit(1, 0, 3);//CR1.DACSEL->0
	write_codec_file_bit(6, 1, 3);// gain set

	write_codec_file_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	write_codec_file_bit(5, 0, 6);//PMR1.SB_OUT->0
	write_codec_file(1, 0x4);
}

static void unset_record_mic_input_audio_without_playback(void)
{
	/* ADC path for MIC IN */
	write_codec_file_bit(5, 1, 4);//SB_ADC->1
	write_codec_file_bit(1, 1, 7);//CR1.SB_MICBIAS->1
	write_codec_file(22, 0xc0);//CR3.SB_MIC1
}

static irqreturn_t aic_codec_irq(int irq, void *dev_id)
{
	u8 file_9 = read_codec_file(9);
	u8 file_8 = read_codec_file(8);

	if ((file_9 & 0x1f) == 0x10) {
		write_codec_file(8, 0x3f);
		write_codec_file_bit(5, 1, 6);//SB_OUT->1
		mdelay(300);
		while ((read_codec_file(9) & 0x4) != 0x4);
		while ((read_codec_file(9) & 0x10) == 0x10) {
			write_codec_file(9, 0x10);
		}
		write_codec_file_bit(5, 0, 6);//SB_OUT->0
		mdelay(300);
		while ((read_codec_file(9) & 0x8) != 0x8);
		write_codec_file(9, file_9);
		write_codec_file(8, file_8);

		return IRQ_HANDLED;
	}
	/*if (file_9 & 0x8)
		ramp_up_end = jiffies;
	else if (file_9 & 0x4)
		ramp_down_end = jiffies;
	else if (file_9 & 0x2)
		gain_up_end = jiffies;
	else if (file_9 & 0x1)
	gain_down_end = jiffies;*/

	write_codec_file(9, file_9);
	/*if (file_9 & 0xf)
	  wake_up(&pop_wait_queue);*/
	while (REG_ICDC_RGDATA & 0x100);

	return IRQ_HANDLED;
}

static void jzdlv_power_on(void)
{
	mdelay(10);
	REG_AIC_I2SCR = 0x10;
	mdelay(20);

	/* power on DLV */
	write_codec_file(8, 0x3f);
	write_codec_file(9, 0xff);
	mdelay(10);
}

static void init_codec(void)
{
	/* reset DLV codec. from hibernate mode to sleep mode */
	write_codec_file(0, 0xf);
	write_codec_file_bit(6, 0, 0);
	write_codec_file_bit(6, 0, 1);
	mdelay(200);
	//write_codec_file(0, 0xf);
	write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
	write_codec_file_bit(5, 0, 4);//PMR1.SB_ADC->0
	mdelay(10);//wait for stability
}

static int jzdlv_reset(struct snd_soc_codec *codec)
{
	/*REG_CPM_CPCCR &= ~(1 << 31);
	  REG_CPM_CPCCR &= ~(1 << 30);*/
	write_codec_file(0, 0xf);

	REG_AIC_I2SCR = 0x10;
	__i2s_internal_codec();
	__i2s_as_slave();
	__i2s_select_i2s();
	__aic_select_i2s();
	__aic_reset();

	jzdlv_power_on();

	init_codec();

	return 0;
}

static const struct snd_kcontrol_new jzdlv_snd_controls[] = {

	SOC_DOUBLE_R("Master Playback Volume", DLV_CGR8, DLV_CGR9, 0, 31, 0),
	SOC_DOUBLE_R("Line", DLV_CGR10, DLV_CGR10, 0, 31, 0),
};

static const struct snd_soc_dapm_widget jzdlv_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),
	SND_SOC_DAPM_INPUT("MIC"),
	SND_SOC_DAPM_INPUT("LIN"),
	SND_SOC_DAPM_INPUT("RIN"),
};

static const struct snd_soc_dapm_route intercon_routes [] = {
	{"Line Input", NULL, "LIN"},
	{"Line Input", NULL, "RIN"},

	{"Input Mixer", "Line Capture Switch", "Line Input"},
	{"Input Mixer", "Mic Capture Switch", "MIC"},

	{"ADC", NULL, "Input Mixer"},

	{"Output Mixer", "Bypass Switch", "Input Mixer"},
	{"Output Mixer", "DAC Switch", "DAC"},

	{"LOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},
};

static int jzdlv_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int speed = 0;
	int val = 0;
	
	switch (params_channels(params)) {
	case 1:
		write_codec_file_bit(1, 1, 6);//CR1.MONO->1 for Mono
		break;
	case 2:
		write_codec_file_bit(1, 0, 6);//CR1.MONO->0 for Stereo
		break;
	}

	switch (params_rate(params)) {
	case 8000:
		speed = 10;
		break;
	case 9600:
		speed = 9;
		break;
	case 11025:
		speed = 8;
		break;
	case 12000:
		speed = 7;
		break;
	case 16000:
		speed = 6;
		break;
	case 22050:
		speed = 5;
		break;
	case 24000:
		speed = 4;
		break;
	case 32000:
		speed = 3;
		break;
	case 44100:
		speed = 2;
		break;
	case 48000:
		speed = 1;
		break;
	case 96000:
		speed = 0;
		break;
	default:
		printk(" invalid rate :0x%08x\n",params_rate(params));
	}

	val = (speed << 4) | speed;
	jzdlv_write(codec, DLV_CCR2, val);

	return 0;
}

static int jzdlv_pcm_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		
		init_codec();
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			set_audio_data_replay();
			write_codec_file_bit(5, 0, 7);//PMR1.SB_DAC->0
			mdelay(300);
			REG_AIC_I2SCR = 0x10;
			mdelay(20);
			__aic_flush_fifo_tx();
		} else {
			set_record_mic_input_audio_without_playback();
			mdelay(10);
			REG_AIC_I2SCR = 0x10;
			mdelay(20);
			__aic_flush_fifo_tx();
			write_codec_file_bit(5, 1, 7);
		}
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			unset_audio_data_replay();
		} else {
			unset_record_mic_input_audio_without_playback();
		}
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int jzdlv_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	//struct snd_soc_pcm_runtime *rtd = substream->private_data;
	//struct snd_soc_codec *codec = rtd->codec;

	return 0;
}

static void jzdlv_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
	}
}

static int jzdlv_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	uint8_t reg_val = jzdlv_read_reg_cache(codec, 2/*DLV_1_LOW*/);

	if (mute != 0) 
		mute = 1;
	if (mute)
		reg_val = reg_val | (0x1 << 14);
	else
		reg_val = reg_val & ~(0x1 << 14);

	//jzdlv_write(codec, DLV_1_LOW, reg_val);
	return 0;
}

static int jzdlv_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	/*struct snd_soc_codec *codec = codec_dai->codec;
	struct jz4750_codec *jz4750_codec = codec->private_data;

	jz4750_codec->sysclk = freq;*/
	return 0;
}

/*
 * Set's ADC and Voice DAC format. called by apus_hw_params() in apus.c
 */
static int jzdlv_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	/* struct snd_soc_codec *codec = codec_dai->codec; */

	/* set master/slave audio interface. codec side */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
                /* set master mode for codec */
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* set slave mode for codec */
		break;
	default:
		return -EINVAL;
	}

	/* interface format . set some parameter for codec side */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S: 
		/* set I2S mode for codec */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		/* set right J mode */
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* set left J mode */
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* set dsp A mode */
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* set dsp B mode */
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion. codec side */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:		
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		break;
	default:
		return -EINVAL;
		}

	return 0;
}

static struct snd_soc_dai_ops jz4750_codec_dai_ops = {
	.trigger = jzdlv_pcm_trigger,
	.prepare = jzdlv_pcm_prepare,
	.hw_params = jzdlv_hw_params,
	.shutdown = jzdlv_shutdown,
	.digital_mute = jzdlv_mute,
	.set_sysclk = jzdlv_set_dai_sysclk,
	.set_fmt = jzdlv_set_dai_fmt,
};

#define JZ4750_CODEC_FMTS (SNDRV_PCM_FMTBIT_S8 | \
			   SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_driver jz4750_codec_dai = {
	.name = "jz4750-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = JZ4750_CODEC_FMTS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = JZ4750_CODEC_FMTS,
	},
	.ops = &jz4750_codec_dai_ops,
	.symmetric_rates = 1,
};

static int jz4750_codec_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
#if 0
	unsigned int mask;
	unsigned int value;
#endif
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
#if 0
		mask = JZ4740_CODEC_1_VREF_DISABLE |
				JZ4740_CODEC_1_VREF_AMP_DISABLE |
				JZ4740_CODEC_1_HEADPHONE_POWERDOWN_M;
		value = 0;

		snd_soc_update_bits(codec, JZ4740_REG_CODEC_1, mask, value);
#endif
		break;
	case SND_SOC_BIAS_STANDBY:
		/* The only way to clear the suspend flag is to reset the codec */
#if 0
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF)
			jz4740_codec_wakeup(codec);

		mask = JZ4740_CODEC_1_VREF_DISABLE |
			JZ4740_CODEC_1_VREF_AMP_DISABLE |
			JZ4740_CODEC_1_HEADPHONE_POWERDOWN_M;
		value = JZ4740_CODEC_1_VREF_DISABLE |
			JZ4740_CODEC_1_VREF_AMP_DISABLE |
			JZ4740_CODEC_1_HEADPHONE_POWERDOWN_M;

		snd_soc_update_bits(codec, JZ4740_REG_CODEC_1, mask, value);
#endif
		break;
	case SND_SOC_BIAS_OFF:
#if 0
		mask = JZ4740_CODEC_1_SUSPEND;
		value = JZ4740_CODEC_1_SUSPEND;

		snd_soc_update_bits(codec, JZ4740_REG_CODEC_1, mask, value);
#endif
		break;
	default:
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}


static int jzdlv_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	//snd_soc_update_bits(codec, JZ4740_REG_CODEC_1,
	//		JZ4740_CODEC_1_SW2_ENABLE, JZ4740_CODEC_1_SW2_ENABLE);
	jzdlv_reset(codec);

	snd_soc_add_controls(codec, jzdlv_snd_controls,
		ARRAY_SIZE(jzdlv_snd_controls));

	snd_soc_dapm_new_controls(dapm, jzdlv_dapm_widgets,
		ARRAY_SIZE(jzdlv_dapm_widgets));

	snd_soc_dapm_add_routes(dapm, intercon_routes,
		ARRAY_SIZE(intercon_routes));

	jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	jzdlv_power_on();
#if 0
	ret = request_irq(IRQ_AIC, aic_codec_irq, IRQF_DISABLED,
			"aic_codec_irq", NULL);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't get aic codec irq %d\n", IRQ_AIC);
		return ret;
	}
#endif
	return 0;
}

static int jzdlv_remove(struct snd_soc_codec *codec)
{
	jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int jzdlv_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	return jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int jzdlv_resume(struct snd_soc_codec *codec)
{
	return jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
}
#else
#define jzdlv_suspend	NULL
#define jzdlv_resume	NULL
#endif

static struct snd_soc_codec_driver soc_codec_dev_jzdlv = {
	.probe			= jzdlv_probe,
	.remove			= jzdlv_remove,
	.suspend		= jzdlv_suspend,
	.resume			= jzdlv_resume,
	.read			= jzdlv_read_reg_cache,
	.write			= jzdlv_write,
	.set_bias_level		= jz4750_codec_set_bias_level,
	.reg_cache_default	= jz4750_codec_regs,
	.reg_word_size		= sizeof(uint8_t),
	.reg_cache_size		= JZDLV_CACHEREGNUM,
};

static int __devinit jz4750_codec_probe(struct platform_device *pdev)
{
	int ret;
	struct jz4750_codec *jz4750_codec;

	jz4750_codec = kzalloc(sizeof(*jz4750_codec), GFP_KERNEL);
	if (!jz4750_codec)
		return -ENOMEM;

	ret = snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_jzdlv, &jz4750_codec_dai, 1);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register codec.\n");
		return ret;
	}

	return 0;

err_free_codec:
	kfree(jz4750_codec);

	return ret;
}

static int __devexit jz4750_codec_remove(struct platform_device *pdev)
{
	struct jz4750_codec *jz4750_codec = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);

	kfree(jz4750_codec);

	return 0;
}

static struct platform_driver jz4750_codec_driver = {
	.probe = jz4750_codec_probe,
	.remove = __devexit_p(jz4750_codec_remove),
	.driver = {
		.name = "jz4750-codec",
		.owner = THIS_MODULE,
	},
};

static int __init jz4750_codec_init(void)
{
	return platform_driver_register(&jz4750_codec_driver);
}
module_init(jz4750_codec_init);

static void __exit jz4750_codec_exit(void)
{
	platform_driver_unregister(&jz4750_codec_driver);
}
module_exit(jz4750_codec_exit);

MODULE_DESCRIPTION("JZ4750 SoC internal driver");
MODULE_AUTHOR("Richard");
MODULE_LICENSE("GPL");

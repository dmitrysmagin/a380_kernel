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

/* For debugging */
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include <asm/mach-jz4750d/jz4750d_aic.h>
#include <asm/mach-jz4750d/jz4750d_intc.h>

/* JZ4750 codec register space */
#define REG_AICR    0x00
#define REG_CR1     0x01
#define REG_CR2     0x02
#define REG_CCR1    0x03
#define REG_CCR2    0x04
#define REG_PMR1    0x05
#define REG_PMR2    0x06
#define REG_CRR     0x07
#define REG_ICR     0x08
#define REG_IFR     0x09
#define REG_CGR1    0x0a
#define REG_CGR2    0x0b
#define REG_CGR3    0x0c
#define REG_CGR4    0x0d
#define REG_CGR5    0x0e
#define REG_CGR6    0x0f
#define REG_CGR7    0x10
#define REG_CGR8    0x11
#define REG_CGR9    0x12
#define REG_CGR10   0x13
#define REG_TR1     0x14
#define REG_TR2     0x15
#define REG_CR3     0x16
#define REG_AGC1    0x17
#define REG_AGC2    0x18
#define REG_AGC3    0x19
#define REG_AGC4    0x1a
#define REG_AGC5    0x1b

#define JZ4750_REGS_NUM		(REG_AGC5 + 1)
#define JZ4750_CODEC_SYSCLK	0

#define REG_AICR_CONFIG1(A)	(((A) & 0x0f) << 0)

#define REG_CR1_BYPASS_OFFSET	2
#define REG_CR1_BYPASS		(1 << 2)
#define REG_CR1_DACSEL_OFFSET	3
#define REG_CR1_DACSEL		(1 << 3)
#define REG_CR1_HP_DIS_OFFSET	4
#define REG_CR1_HP_DIS		(1 << 4)
#define REG_CR1_DAC_MUTE	(1 << 5)
#define REG_CR1_MONO		(1 << 6)
#define REG_CR1_SB_MICBIAS	(1 << 7)

#define REG_CR2_ADC_HPF		(1 << 2)
#define REG_CR2_ADC_ADWL(A)	(((A) & 3) << 3)
#define REG_CR2_DAC_ADWL(A)	(((A) & 3) << 5)
#define REG_CR2_DAC_DEEMP	(1 << 7)

#define REG_CR3_INSEL(A)	(((A) & 3) << 0)
#define REG_CR3_MICSTEREO	(1 << 2)
#define REG_CR3_MICDIFF		(1 << 3)
#define REG_CR3_SIDETONE2	(1 << 4)
#define REG_CR3_SIDETONE1	(1 << 5)
#define REG_CR3_SB_MIC2		(1 << 6)
#define REG_CR3_SB_MIC1		(1 << 7)

#define REG_CCR1_CONFIG4(A)	(((A) & 0x0f) << 0)

#define REG_CCR2_AFREQ(A)	(((A) & 0x0f) << 0)
#define REG_CCR2_DFREQ(A)	(((A) & 0x0f) << 4)

#define REG_PMR1_SB_IND		(1 << 0)
#define REG_PMR1_SB_LIN		(1 << 3)
#define REG_PMR1_SB_ADC_OFFSET	4
#define REG_PMR1_SB_ADC		(1 << 4)
#define REG_PMR1_SB_MIX_OFFSET	5
#define REG_PMR1_SB_MIX		(1 << 5)
#define REG_PMR1_SB_OUT		(1 << 6)
#define REG_PMR1_SB_DAC_OFFSET	7
#define REG_PMR1_SB_DAC		(1 << 7)

#define REG_PMR2_SB_SLEEP	(1 << 0)
#define REG_PMR2_SB		(1 << 1)
#define REG_PMR2_SB_MC		(1 << 2)
#define REG_PMR2_GIM		(1 << 3)
#define REG_PMR2_GOD(A)		(((A) & 3) << 4)
#define REG_PMR2_GI(A)		(((A) & 3) << 5)

#define REG_CRR_THRESH(A)	(((A) & 3) << 0)
#define REG_CRR_KFAST(A)	(((A) & 7) << 2)
#define REG_CRR_RATIO(A)	(((A) & 3) << 5)

#define REG_ICR_GDD		(1 << 0)
#define REG_ICR_GUD		(1 << 1)
#define REG_ICR_RDD		(1 << 2)
#define REG_ICR_RUD		(1 << 3)
#define REG_ICR_CCMC		(1 << 4)
#define REG_ICR_JACK		(1 << 5)
#define REG_ICR_INT_FORM(A)	(((A) & 3) << 6)

#define REG_IFR_GDD		(1 << 0)
#define REG_IFR_GUD		(1 << 1)
#define REG_IFR_RDD		(1 << 2)
#define REG_IFR_RUD		(1 << 3)
#define REG_IFR_CCMC		(1 << 4)
#define REG_IFR_JACK_EVENT	(1 << 5)
#define REG_IFR_JACK		(1 << 6)

#define REG_CR3_INSEL(A)	(((A) & 3) << 0)
#define REG_CR3_MICSTEREO	(1 << 2)
#define REG_CR3_MICDIFF		(1 << 3)
#define REG_CR3_SIDETONE2	(1 << 4)
#define REG_CR3_SIDETONE1	(1 << 5)
#define REG_CR3_SB_MIC2		(1 << 6)
#define REG_CR3_SB_MIC1		(1 << 7)

static const uint8_t jz4750_codec_regs[JZ4750_REGS_NUM] = {
	0x0C, 0xAA, 0x78, 0x00, 0x00, 0xFF, 0x03, 0x51,
	0x3F, 0x00, 0x00, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0xC0, 0x34,
	0x07, 0x44, 0x1F, 0x00
};

static int jz_codec_debug = 1;
module_param(jz_codec_debug, int, 0644);

#define DEBUG_MSG(msg...)			\
	do {					\
		if (jz_codec_debug)		\
			printk("ICDC: " msg);	\
	} while(0)

struct jz4750_codec {
	void __iomem *base;
	struct resource *mem;
};

static int bypass_to_hp = 0;
static int bypass_to_lineout = 0;

static int jz4750_codec_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level);

static inline int read_codec_file(int reg)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(reg);
	mdelay(1);
	return(__icdc_get_value());
}

static inline void write_codec_file(int reg, int val)
{
	while (__icdc_rgwr_ready());
	__icdc_set_addr(reg);
	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);
}

static int codec_debug_show(struct seq_file *m, void *v)
{
	unsigned int cr1, cr2, cr3, pmr1, pmr2, icr, ifr;

	cr1 = read_codec_file(REG_CR1);
	cr2 = read_codec_file(REG_CR2);
	cr3 = read_codec_file(REG_CR3);
	pmr1 = read_codec_file(REG_PMR1);
	pmr2 = read_codec_file(REG_PMR2);
	icr = read_codec_file(REG_ICR);
	ifr = read_codec_file(REG_IFR);

	seq_printf(m,
		"CR1_SB_MICBIAS=%i\n"
		"CR1_MONO=%i\n"
		"CR1_DAC_MUTE=%i\n"
		"CR1_HP_DIS=%i\n"
		"CR1_DACSEL=%i\n"
		"CR1_BYPASS=%i\n",
		!!(cr1 & REG_CR1_SB_MICBIAS),
		!!(cr1 & REG_CR1_MONO),
		!!(cr1 & REG_CR1_DAC_MUTE),
		!!(cr1 & REG_CR1_HP_DIS),
		!!(cr1 & REG_CR1_DACSEL),
		!!(cr1 & REG_CR1_BYPASS)
	);

	seq_printf(m,
		"CR2_DAC_DEEMP=%i\n"
		"CR2_DAC_ADWL=%i\n"
		"CR2_ADC_ADWL=%i\n"
		"CR2_ADC_HPF=%i\n",
		!!(cr2 & REG_CR2_DAC_DEEMP),
		(cr2 >> 5) & 3,
		(cr2 >> 3) & 3,
		!!(cr2 & REG_CR2_ADC_HPF)
	);

	seq_printf(m,
		"CR3_SB_MIC1=%i\n"
		"CR3_SB_MIC2=%i\n"
		"CR3_SIDETONE1=%i\n"
		"CR3_SIDETONE2=%i\n"
		"CR3_MICDIFF=%i\n"
		"CR3_MICSTEREO=%i\n"
		"CR3_INSEL=%i\n",
		!!(cr3 & REG_CR3_SB_MIC1),
		!!(cr3 & REG_CR3_SB_MIC2),
		!!(cr3 & REG_CR3_SIDETONE1),
		!!(cr3 & REG_CR3_SIDETONE2),
		!!(cr3 & REG_CR3_MICDIFF),
		!!(cr3 & REG_CR3_MICSTEREO),
		cr3 & 3
	);

	seq_printf(m,
		"PMR1_SB_DAC=%i\n"
		"PMR1_SB_OUT=%i\n"
		"PMR1_SB_MIX=%i\n"
		"PMR1_SB_ADC=%i\n"
		"PMR1_SB_LIN=%i\n"
		"PMR1_SB_IND=%i\n",
		!!(pmr1 & REG_PMR1_SB_DAC),
		!!(pmr1 & REG_PMR1_SB_OUT),
		!!(pmr1 & REG_PMR1_SB_MIX),
		!!(pmr1 & REG_PMR1_SB_ADC),
		!!(pmr1 & REG_PMR1_SB_LIN),
		!!(pmr1 & REG_PMR1_SB_IND)
	);

	seq_printf(m,
		"PMR2_GI=%i\n"
		"PMR2_GOD=%i\n"
		"PMR2_GIM=%i\n"
		"PMR2_SB_MC=%i\n"
		"PMR2_SB=%i\n"
		"PMR2_SB_SLEEP=%i\n",
		(pmr2 >> 6) & 3,
		(pmr2 >> 4) & 3,
		!!(pmr2 & REG_PMR2_GIM),
		!!(pmr2 & REG_PMR2_SB_MC),
		!!(pmr2 & REG_PMR2_SB),
		!!(pmr2 & REG_PMR2_SB_SLEEP)
	);

	seq_printf(m,
		"ICR=0x%02x\n"
		"IFR=0x%02x, IFR_CCMC=%i\n",
		icr, ifr,
		!!(ifr & REG_IFR_CCMC)
	);

	return 0;
}

static int codec_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, codec_debug_show, NULL);
}

static const struct file_operations codec_debug_fops = {
	.open		= codec_debug_open,
	.read		= seq_read,
	//.write		= seq_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static unsigned int jz4750_codec_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	//struct jz4750_codec *jz4750_codec = snd_soc_codec_get_drvdata(codec);
	uint8_t *reg_cache = codec->reg_cache;

	if (reg >= JZ4750_REGS_NUM)
		return -1;

	reg_cache[reg] = read_codec_file(reg);

	return reg_cache[reg];
}

static int jz4750_codec_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	//struct jz4750_codec *jz4750_codec = snd_soc_codec_get_drvdata(codec);
	uint8_t *reg_cache = codec->reg_cache;

	if (reg >= JZ4750_REGS_NUM) {
		return -1;
	}

	DEBUG_MSG("write reg=0x%02x to val=0x%02x\n", reg, value);

	reg_cache[reg] = (uint8_t)value;
	write_codec_file(reg, (uint8_t)value);

	return 0;
}

static const DECLARE_TLV_DB_SCALE(dac_tlv, -2250, 150, 0);

static const unsigned int in_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	//31-31, 31-20, TLV_DB_SCALE_ITEM(-2250, 0, 0),
	31-19, 31-00, TLV_DB_SCALE_ITEM(-2250, 150, 0),
};

static const DECLARE_TLV_DB_SCALE(line_tlv, 0, 150, 0);

static const struct snd_kcontrol_new jz4750_codec_controls[] = {
	SOC_DOUBLE_TLV("DAC Mixing", REG_CGR1, 4, 0, 15, 1, dac_tlv),
//	SOC_DOUBLE_R_TLV("Line 1 Mixing", REG_CGR2, REG_CGR3, 0, 31, 1, in_tlv),
//	SOC_DOUBLE_R_TLV("Mic 1 Mixing", REG_CGR4, REG_CGR5, 0, 31, 1, in_tlv),
//	SOC_DOUBLE_R_TLV("Mic 2 Mixing", REG_CGR6, REG_CGR7, 0, 31, 1, in_tlv),
	SOC_DOUBLE_R_TLV("Master Playback", REG_CGR9, REG_CGR8,
			 0, 31, 1, in_tlv),
#if 0
	SOC_SINGLE("Master Playback Switch", REG_CR1,
			 REG_CR1_HP_DIS_OFFSET, 1, 1),
#endif
//	SOC_DOUBLE_TLV("Line", REG_CGR10, 4, 0, 15, 0, line_tlv),
};

static const struct snd_kcontrol_new jz4750_codec_output_controls[] = {
	SOC_DAPM_SINGLE("Bypass Switch", REG_CR1,
			REG_CR1_BYPASS_OFFSET, 1, 1),
	SOC_DAPM_SINGLE("DAC Switch", REG_CR1,
			REG_CR1_DACSEL_OFFSET, 1, 0),
};

static const struct snd_soc_dapm_widget jz4750_codec_dapm_widgets[] = {
	//SND_SOC_DAPM_ADC("ADC", "Capture", REG_PMR1,
	//		REG_PMR1_SB_ADC_OFFSET, 1),
	SND_SOC_DAPM_DAC("DAC", "Playback", REG_PMR1,
			REG_PMR1_SB_DAC_OFFSET, 1),

	SND_SOC_DAPM_MIXER("Output Mixer", SND_SOC_NOPM /*REG_PMR1*/,
			0/*REG_PMR1_SB_MIX_OFFSET*/, 1,
			jz4750_codec_output_controls,
			ARRAY_SIZE(jz4750_codec_output_controls)),

	//SND_SOC_DAPM_MIXER("Line Input", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("LHPOUT"),
	SND_SOC_DAPM_OUTPUT("RHPOUT"),

	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),

	//SND_SOC_DAPM_INPUT("MIC"),

	//SND_SOC_DAPM_INPUT("LIN"),
	//SND_SOC_DAPM_INPUT("RIN"),
};

static const struct snd_soc_dapm_route jz4750_codec_dapm_routes[] = {
	/*{"Line Input", NULL, "LIN"},
	{"Line Input", NULL, "RIN"},

	{"Input Mixer", "Line Capture Switch", "Line Input"},
	{"Input Mixer", "Mic Capture Switch", "MIC"},

	{"ADC", NULL, "Input Mixer"},

	{"Output Mixer", "Bypass Switch", "Input Mixer"},*/
	{"Output Mixer", "DAC Switch", "DAC"},

	{"LOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},

	{"LHPOUT", NULL, "Output Mixer"},
	{"RHPOUT", NULL, "Output Mixer"},
};

static int jz4750_codec_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;

	return 0;
}

static void jz4750_codec_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;

	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	DEBUG_MSG("enter jz4750_codec_shutdown, playback = %d\n", playback);
}

static int jz4750_codec_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int bit_width;
	int duplicate = 0; /* stereo to mono */
	int speed;

	/* check bit width */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width = 2;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width = 3;
		break;
	default:
		return -EINVAL;
	}

	/* check channels */
	switch (params_channels(params)) {
	case 1:
		duplicate = REG_CR1_MONO;
		break;
	case 2:
		duplicate = 0;
		break;
	}

	/* check sample rate */
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
		printk("Invalid rate: %d\n", params_rate(params));
		return -EINVAL;
	}

	/* apply bit width */
	snd_soc_update_bits(codec, REG_CR2,
			REG_CR2_DAC_ADWL(3) |
			REG_CR2_ADC_ADWL(3),
			REG_CR2_DAC_ADWL(bit_width) |
			REG_CR2_ADC_ADWL(bit_width));

	/* apply channels */
	snd_soc_update_bits(codec, REG_CR1, REG_CR1_MONO, duplicate);

	/* apply sample rate */
	snd_soc_write(codec, REG_CCR2, (speed << 4) | speed);

	return 0;
}

static int jz4750_codec_pcm_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret = 0;

	DEBUG_MSG("enter %s:%d substream = %s bypass_to_hp = %d "
		  "bypassto_lineout = %d cmd = %d\n",
		  __func__, __LINE__,
		  (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		  "playback" : "capture"),
		  bypass_to_hp, bypass_to_lineout,
		  cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_ON);
			mdelay(2);
		}

		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		/* do nothing */
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int jz4750_codec_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int gain_bit = mute ? 0 /* GDO */ : 1 /* GUP */;
	int change;

	change = snd_soc_update_bits(codec, REG_CR1,
			REG_CR1_DAC_MUTE, mute << 5);

	if (change == 1 &&
	    !(jz4750_codec_read(codec, REG_PMR1) & REG_PMR1_SB_DAC)) {
		/* wait for gain up/down complete (GUP/GDO) */
		while (!(jz4750_codec_read(codec, REG_IFR) & (1 << gain_bit)))
			mdelay(10);

		mdelay(1);

		/* clear GUP_GDO flag */
		snd_soc_update_bits(codec, REG_IFR,
			1 << gain_bit, 1 << gain_bit);

		return 0;
	} else {
		return change;
	}
}

static struct snd_soc_dai_ops jz4750_codec_dai_ops = {
	.startup	= jz4750_codec_startup,
	.shutdown	= jz4750_codec_shutdown,
	.hw_params	= jz4750_codec_hw_params,
	.trigger	= jz4750_codec_pcm_trigger,
	.digital_mute	= jz4750_codec_mute,
};

#define JZ4750_CODEC_FMTS (SNDRV_PCM_FMTBIT_S16_LE | \
			   SNDRV_PCM_FMTBIT_S18_3LE | \
			   SNDRV_PCM_FMTBIT_S20_3LE | \
			   SNDRV_PCM_FMTBIT_S24_3LE)

static struct snd_soc_dai_driver jz4750_codec_dai = {
	.name = "jz4750-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = JZ4750_CODEC_FMTS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = JZ4750_CODEC_FMTS,
	},
	.ops = &jz4750_codec_dai_ops,
	.symmetric_rates = 1,
};

static int jz4750_codec_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		/* PMR2.SB = 0 */
		snd_soc_update_bits(codec, REG_PMR2,
			REG_PMR2_SB, 0);
		mdelay(300);

		/* PMR2.SB_SLEEP = 0 */
		snd_soc_update_bits(codec, REG_PMR2,
			REG_PMR2_SB_SLEEP, 0);
		mdelay(400);
		break;
	case SND_SOC_BIAS_OFF:
		/* PMR2.SB_SLEEP = 1 */
		snd_soc_update_bits(codec, REG_PMR2,
			REG_PMR2_SB_SLEEP, REG_PMR2_SB_SLEEP);
		mdelay(10);

		/* PMR2.SB = 1 */
		snd_soc_update_bits(codec, REG_PMR2,
			REG_PMR2_SB, REG_PMR2_SB);
		break;
	default:
		break;
	}

	codec->dapm.bias_level = level;

	return 0;
}


static int jz4750_codec_dev_probe(struct snd_soc_codec *codec)
{
	/* magic value AICR.CONTROL1 = 0x0f */
	snd_soc_write(codec, REG_AICR, 0xf);
	/* magic value CCR1.CONFIG4 = 0 */
	snd_soc_write(codec, REG_CCR1, 0);

	/* Mask all interrupts except CCMC */
	snd_soc_write(codec, REG_ICR, 0x2f);
	snd_soc_write(codec, REG_IFR, 0xff);
	mdelay(10);

	snd_soc_update_bits(codec, REG_CR1,
			REG_CR1_SB_MICBIAS | // CR1.MICBIAS = 1
			REG_CR1_DAC_MUTE |   // CR1.DACMUTE = 0
			REG_CR1_HP_DIS |     // CR1.HP_DIS = 0
			REG_CR1_DACSEL |     // CR1.DACSEL = 1
			REG_CR1_BYPASS,      // CR1.BYPASS = 0
			REG_CR1_SB_MICBIAS |
			REG_CR1_DACSEL);
	mdelay(10);

	snd_soc_update_bits(codec, REG_CR2,
			REG_CR2_DAC_DEEMP |   // CR2.DAC_DEEMP = 0
			REG_CR2_DAC_ADWL(3) | // CR2.DAC_ADWL = 0
			REG_CR2_ADC_ADWL(3) | // CR2.ADC_ADWL = 0
			REG_CR2_ADC_HPF,      // CR2.ADC_HPF = 0
			0);
	mdelay(10);

	snd_soc_write(codec, REG_CR3, 0xc0); // magic value for replay
	mdelay(10);

	/* later rework this */
	snd_soc_update_bits(codec, REG_PMR1,
			REG_PMR1_SB_DAC |    // PMR1.SB_DAC = 0
			REG_PMR1_SB_OUT |    // PMR1.SB_OUT = 0
			REG_PMR1_SB_MIX |    // PMR1.SB_MIX = 0
			REG_PMR1_SB_ADC |    // PMR1.SB_ADC = 0
			REG_PMR1_SB_LIN |    // PMR1.SB_LIN = 1
			REG_PMR1_SB_IND,     // PMR1.SB_IND = 1
			REG_PMR1_SB_LIN |
			REG_PMR1_SB_IND);

	mdelay(10);

	snd_soc_update_bits(codec, REG_PMR2,
			REG_PMR2_GI(3) |     // PMR2.GI    = 0
			REG_PMR2_GOD(3) |    // PMR2.GOD   = 0
			REG_PMR2_GIM |       // PMR.GIM    = 0
			REG_PMR2_SB_MC,      // PMR2.SB_MC = 0
			0);
	mdelay(10);

	snd_soc_write(codec, REG_CRR, 0x51); // reduce pop noise
	mdelay(10);

	snd_soc_update_bits(codec, REG_CGR2, 0xc0, 0);
	snd_soc_update_bits(codec, REG_CGR4, 0xc0, 0);
	snd_soc_update_bits(codec, REG_CGR6, 0xc0, 0);
	snd_soc_write(codec, REG_CGR8, 12);
	snd_soc_write(codec, REG_CGR9, 12);

	jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int jz4750_codec_dev_remove(struct snd_soc_codec *codec)
{
	jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int jz4750_codec_suspend(struct snd_soc_codec *codec)
{
	return jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int jz4750_codec_resume(struct snd_soc_codec *codec)
{
	return jz4750_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
}
#else
#define jz4750_codec_suspend	NULL
#define jz4750_codec_resume	NULL
#endif

static struct snd_soc_codec_driver soc_codec_dev_jzdlv = {
	.probe			= jz4750_codec_dev_probe,
	.remove			= jz4750_codec_dev_remove,
	.suspend		= jz4750_codec_suspend,
	.resume			= jz4750_codec_resume,
	.read			= jz4750_codec_read,
	.write			= jz4750_codec_write,
	.set_bias_level		= jz4750_codec_set_bias_level,
	.reg_cache_default	= jz4750_codec_regs,
	.reg_word_size		= sizeof(uint8_t),
	.reg_cache_size		= JZ4750_REGS_NUM,

	.controls		= jz4750_codec_controls,
	.num_controls		= ARRAY_SIZE(jz4750_codec_controls),
	.dapm_widgets		= jz4750_codec_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(jz4750_codec_dapm_widgets),
	.dapm_routes		= jz4750_codec_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(jz4750_codec_dapm_routes),
};

static int jz4750_codec_probe(struct platform_device *pdev)
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

	proc_create("jz/codec", 0644, 0, &codec_debug_fops);

	return 0;

err_free_codec:
	kfree(jz4750_codec);

	return ret;
}

static int jz4750_codec_remove(struct platform_device *pdev)
{
	struct jz4750_codec *jz4750_codec = platform_get_drvdata(pdev);

	snd_soc_unregister_codec(&pdev->dev);

	kfree(jz4750_codec);

	return 0;
}

static struct platform_driver jz4750_codec_driver = {
	.probe = jz4750_codec_probe,
	.remove = jz4750_codec_remove,
	.driver = {
		.name = "jz4750-codec",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(jz4750_codec_driver);

MODULE_AUTHOR("Richard, <cjfeng@ingenic.cn>");
MODULE_DESCRIPTION("JZ4750 SoC internal driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:jz4750-codec");

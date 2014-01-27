#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>

#include <asm/mach-jz4750d/jz4750d_aic.h>
#include <asm/mach-jz4750d/jz4750d_cpm.h>
#include <asm/mach-jz4750d/jz4750d_gpio.h>
#include <asm/mach-jz4750d/i2c.h>

MODULE_AUTHOR("caijicheng<caijicheng2006@163.com>");
MODULE_DESCRIPTION("RDA5807P radio Driver");
MODULE_LICENSE("GPL");

#define FM_DEV_NAME "fm_rda5807p"
#define FM_MINOR 0x42

#define FM_I2C_ADDR 0x11

#define AUTO_SEEK 0x0
#define AUTO_SEEK_JAPAN 0x01
#define SET_FREQ 0x02
#define SET_VOLUME 0x03
#define READ_VOLUME 0x04
#define SET_STEREO 0x05
//#define READ_STEREO 0x06
#define FM_POWER_ON 0x06
#define FM_POWER_OFF 0x07
#define SET_MUTE     0x08
#define SET_THRESHOLD     0x09
#define SET_AREA 0x0a

unsigned char reset_chip[2] = {0x00, 0x02};
unsigned char power_on_chip[2] = {0xc0, 0x01};
unsigned char read_data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char set_tune[2] = {0x00, 0x10};
unsigned char set_threshold[2]={0x08, 0xa8};

unsigned int freq[220];

/****    codec set input line  start   ****/
void radio_write_reg(int addr, int val)
{
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	REG_ICDC_RGADW = ((addr << ICDC_RGADW_RGADDR_BIT) | val);
	__icdc_set_rgwr();
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
}

static int radio_write_reg_bit(int addr, int bitval, int mask_bit)
{
	int val;

	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	mdelay(1);
	/* read */
	val = __icdc_get_value();
	while (__icdc_rgwr_ready()) {
		;//nothing...
	}

	__icdc_set_addr(addr);
	val &= ~(1 << mask_bit);
	if (bitval == 1) {
		val |= 1 << mask_bit;
	}

	__icdc_set_cmd(val); /* write */
	mdelay(1);
	__icdc_set_rgwr();
	mdelay(1);

	while (__icdc_rgwr_ready()) {
		;//nothing...
	}
	__icdc_set_addr(addr);
	val = __icdc_get_value(); /* read */

	if (((val >> mask_bit) & bitval) == bitval) {
		return 1;
	} else {
		return 0;
	}
}

static void radio_reset(void)
{
	/* reset DLV codec. from hibernate mode to sleep mode */
	radio_write_reg(0, 0xf);
	radio_write_reg_bit(6, 0, 0);
	radio_write_reg_bit(6, 0, 1);

	//2010-01-31 Jason add
	radio_write_reg(22, 0x40);//mic 1

	schedule_timeout(20);
	//radio_write_reg(0, 0xf);
	radio_write_reg_bit(5, 0, 7);//PMR1.SB_DAC->0
	radio_write_reg_bit(5, 0, 4);//PMR1.SB_ADC->0
	schedule_timeout(2); ;//wait for stability
}

static void set_playback_line_input_audio(void)
{
	// need fix !!!
	//jz_audio_reset();//or init_codec()
	REG_AIC_I2SCR = 0x10;
	radio_write_reg(9, 0xff);
	radio_write_reg(8, 0x3f);
	mdelay(10);
	radio_write_reg(22, 0xf6);//line in 1
	radio_write_reg_bit(23, 0, 7);//AGC1.AGC_EN->0
	mdelay(10);
	radio_write_reg_bit(1, 1, 2);//CR1.HP_BYPASS->1
	radio_write_reg_bit(1, 0, 4);//CR1.HP_DIS->0
	radio_write_reg_bit(1, 0, 3);//CR1.DACSEL->0
	radio_write_reg_bit(5, 1, 0);//PMR1.SB_IND->1
	radio_write_reg_bit(5, 0, 3);//PMR1.SB_LIN->0

	radio_write_reg_bit(5, 0, 5);//PMR1.SB_MIX->0
	mdelay(100);
	radio_write_reg_bit(5, 0, 6);//PMR1.SB_OUT->0
}

static void radio_set_volume(int val)
{
	int cur_vol;

	cur_vol = val;
	cur_vol &= 0x1f;
	radio_write_reg(11, cur_vol);
	radio_write_reg(12, cur_vol);
}

/****    codec set input line  end   ****/
u16 rda5807p_freq_to_chan(u16 frequency)
{
	unsigned int channel_spacing=0;
	unsigned int bottom_band=0;
	unsigned int channel=0;

	if ((set_tune[1] & 0x0c) == 0x00)
		bottom_band = 870;
	else if ((set_tune[1] & 0x0c) == 0x04)
		bottom_band = 760;
	else if ((set_tune[1] & 0x0c) == 0x08)
		bottom_band = 760;

	/* because we can not use float, so channel spacing is 2x */
	switch(set_tune[1] & 0x03) {
	case 0x0:
		channel_spacing = 200;
		break;
	case 0x1:
		channel_spacing = 400;
		break;
	case 0x2:
		channel_spacing = 100;
		break;
	case 0x3:
		channel_spacing = 25;

	}

	channel = ((frequency - bottom_band) * 2 * 100) / channel_spacing;

	return (channel);
}

/* cur_freq 870~1080 */
int rda5807p_set_freq(u16 cur_freq, int wait_true)
{
	int i = 0;
	int wait_time;
	u16 cur_chan;

	cur_chan = rda5807p_freq_to_chan(cur_freq);

	set_tune[0] = cur_chan >> 2;

	/* NOTE: lower 4bits of reg03 is set in fm_ioctl, and can NOT change in other place */
	set_tune[1] |= 0x10;  /* tune enable */
	set_tune[1] |= ((cur_chan & 0x3) << 6);

	i2c_write(FM_I2C_ADDR, set_tune, 0x03, 2);

	if (wait_true)
		wait_time = 25; /* 25ms */
	else
		wait_time = 15; /* 15ms */

	for (i = 0; i < wait_time; i++) {
		memset(read_data, 0, 4);
		i2c_read(FM_I2C_ADDR, read_data, 0x0a, 4);

#define FM_STC		(1 << 6) /* 0x0a, bit14 */

#define FM_TRUE		(1 << 0) /* 0x0b, bit8 */
#define FM_READY	(1 << 7) /* 0x0b, bit7 */

		if ((read_data[0]) & FM_STC) { /* tune complete */
			if (!wait_true)
				return 1;

			if ((read_data[2] & FM_TRUE) &&
			    (read_data[3] & FM_READY)) {
				return 1;
			}
		}

		/* continue */
		msleep(1);
	}

	return 0;
	}

static int fm_open(struct inode *inode, struct file *filp)
{
	printk("fm_open!\n");
	__gpio_as_i2c();
	udelay(200);
	__cpm_start_i2c();
	i2c_close();
	i2c_open();

	radio_reset();
	set_playback_line_input_audio();

	// reset chip
	i2c_write(FM_I2C_ADDR, reset_chip, 0x02, 2);

	// power on chip
	i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);

	i2c_write(FM_I2C_ADDR, set_threshold, 0x05, 2);
	return 0;
}

static ssize_t fm_read(struct file *filp, char __user *buf, size_t count, loff_t *f_ops)
{
	return 4;
}

static ssize_t fm_write(struct file *filp, char __user *buf, size_t count, loff_t *f_ops)
{
	return 4;
}

static int fm_release(struct inode *inode, struct file *filp)
{
	printk("fm_release!\n");

	radio_reset();

	i2c_close();
	__cpm_stop_i2c();
	return 0;
}

static unsigned int area_flag = 0;   //2 : japen  1 : other
static int fm_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int threshold = 2;
	int seek=0;
	int stereo=2;
	int volume=0;
	int set_freq = 870;
	int mute = 2;

	switch (cmd) {
	case SET_AREA:
		get_user(area_flag, (int*)arg);
		printk("cur area flag  is %d\n",area_flag);
		break;
	case AUTO_SEEK:
		get_user(seek, (int*)arg);
		set_tune[0] = 0x0;
		set_tune[1] = 0x10;
		printk("rda5807p set freq = %d\n", seek);

		return rda5807p_set_freq(seek,1);

	case AUTO_SEEK_JAPAN:
		get_user(seek, (int*)arg);
		set_tune[0] = 0x0;
		set_tune[1] = 0x14;

		return rda5807p_set_freq(seek,1);

	case SET_FREQ:
		get_user(set_freq, (int*)arg);

		//medive change
		if (area_flag == 1){
			set_tune[0] = 0x0;
			set_tune[1] = 0x10;
		}else if (area_flag == 2){
			set_tune[0] = 0x0;
			set_tune[1] = 0x14;
		}else{
			set_tune[0] = 0x0;
			set_tune[1] = 0x10;
		}
		//end

		rda5807p_set_freq(set_freq,0);

		break;

	case SET_VOLUME:
		get_user(volume, (int*)arg); // volume : 0 ~ 31

		if (volume < 0)
			volume = 0;
		if (volume > 31)
			volume = 31;

		radio_set_volume(volume);

		mdelay(2);
		break;

	case READ_VOLUME:
		mdelay(2);
		put_user(volume, (int*)arg);
		break;

	case SET_MUTE:
		get_user(mute, (int*)arg);
		if (mute == 0) {
			power_on_chip[0] &= ~(1<<7);
			printk("set mute = %d\n", mute);
		} else {
			power_on_chip[0] |= (1<<7);
			printk("set normal = %d\n", mute);
		}
		i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		mdelay(50);
		break;

	case SET_STEREO:
		get_user(stereo, (int*)arg);
		if (stereo == 0) { // 0:set stereo  1: set mono
			power_on_chip[0] &= ~(1<<6);
			printk("set stereo = %d\n", stereo);
		} else {
			power_on_chip[0] |= (1<<6);
			printk("set mono = %d\n", stereo);
		}
		i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		mdelay(50);
		break;

	case FM_POWER_ON:
		power_on_chip[0] =0xc0;
		power_on_chip[1] =0x01;
		set_threshold[0] =0x14;
		set_threshold[1] =0xa8;

		__gpio_as_i2c();
		udelay(200);
		__cpm_start_i2c();
		i2c_close();
		i2c_open();

		i2c_write(FM_I2C_ADDR, reset_chip, 0x02, 2);
		i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		i2c_write(FM_I2C_ADDR, set_threshold, 0x05, 2);

		break;

	case FM_POWER_OFF:
		power_on_chip[1] &= (~0x01);
		i2c_write(FM_I2C_ADDR, power_on_chip, 0x02, 2);
		mdelay(20);
		printk("FM_POWER_OFF\n");
		break;

	case SET_THRESHOLD:
		get_user(threshold, (int*)arg);
		if (threshold == 1) { // 1: use auto seek, 0:use manual seek
			set_threshold[0] = 0x0d;
			set_threshold[1] = 0xa8;
		} else if (threshold == 2) {
			set_threshold[0] = 0x0b;
			set_threshold[1] = 0xa8;
		} else if (threshold == 3) {
			set_threshold[0] = 0x08;
			set_threshold[1] = 0xa8;
		}

		i2c_write(FM_I2C_ADDR, set_threshold, 0x05, 2);
		break;

	default:
		printk("can not support fm ioctl\n");
		return -EINVAL;
		break;
	}
	return 0;
}

static long
fm_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;

	lock_kernel();
	ret = fm_ioctl(file, cmd, arg);
	unlock_kernel();

	return ret;
}

static struct file_operations fm_fops = {
	.owner   = THIS_MODULE,
	.open    = fm_open,
	.read    = fm_read,
	.write   = fm_write,
	.unlocked_ioctl   = fm_unlocked_ioctl,
	.release = fm_release,
};

static struct miscdevice fm_device = {
	.minor = FM_MINOR,
	.name  = FM_DEV_NAME,
	.fops  = &fm_fops,
};

static int __init rda5807p_init(void)
{
	int ret = 0;

	printk("init rda5807p \n");
	ret = misc_register(&fm_device);
	if (ret < 0) {
		printk("misc register fm err!\n");
		return ret;
	}

	return 0;
}

static void __exit rda5807p_exit(void)
{
	printk("exit rda5807p!\n");
	misc_deregister(&fm_device);
}

module_init(rda5807p_init);
module_exit(rda5807p_exit);

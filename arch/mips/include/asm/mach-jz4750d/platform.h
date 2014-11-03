#ifndef __JZ4750D_PLATFORM_H__
#define __JZ4750D_PLATFORM_H__

#include <linux/platform_device.h>

extern struct platform_device jz_udc_device;
extern struct platform_device jz_msc0_device;
extern struct platform_device jz_msc1_device;
extern struct platform_device jz_rtc_device;
extern struct platform_device jz_i2c_device;
extern struct platform_device jz_lcd_device;
extern struct platform_device jz_i2s_device;
extern struct platform_device jz_pcm_device;
extern struct platform_device jz_codec_device;
extern struct platform_device jz_adc_device;
extern struct platform_device jz_wdt_device;

void jz4750d_serial_device_register(void);

#endif /* __JZ4750D_PLATFORM_H__ */

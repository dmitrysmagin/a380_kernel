#ifndef __JZ4750D_PLATFORM_H__
#define __JZ4750D_PLATFORM_H__

#include <linux/platform_device.h>

extern struct platform_device jz_lcd_device;
extern struct platform_device jz_usb_gdt_device;
extern struct platform_device jz_msc0_device;
extern struct platform_device jz_msc1_device;
extern struct platform_device jz_i2c_device;

void jz4750d_serial_device_register(void);

#endif /* __JZ4750D_PLATFORM_H__ */

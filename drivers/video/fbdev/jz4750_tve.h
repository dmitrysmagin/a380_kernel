#ifndef __JZ4750_TVE_H__
#define __JZ4750_TVE_H__

#define PANEL_MODE_LCD_PANEL	0
#define PANEL_MODE_TVE_PAL	1
#define PANEL_MODE_TVE_NTSC	2

/* TV parameter */
#define TVE_WIDTH_PAL 		720
#define TVE_HEIGHT_PAL 		573
#define TVE_DISP_WIDTH_PAL      672
#define TVE_DISP_HEIGHT_PAL     551
#define TVE_FREQ_PAL 		50

#define TVE_WIDTH_NTSC 		720
#define TVE_HEIGHT_NTSC 	482
#define TVE_FREQ_NTSC 		60

extern void jz4750tve_enable_tve(void);
extern void jz4750tve_disable_tve(void);

extern void jz4750tve_init(int tve_mode);

extern struct panel_ops jz4750_tve_panel_ops;

#endif	/* __JZ4750_TVE_H__ */

#ifndef _A380_ILI9331_H_
#define _A380_ILI9331_H_

#if defined (CONFIG_JZ4750_SLCD_A380_ILI9331)
#define WR_GRAM_CMD	0x22

#if defined(CONFIG_JZ4750D_A380)
#define PIN_CS_N	(32*3+22)		/* GPD22 */
#define PIN_RESET_N	(32*3+23)		/* GPD23 */
#else
#error "Define special lcd pins for your platform."
#endif

#if 0
static void Mcupanel_Data(unsigned int data)
{
	while ((REG_SLCD_STATE)&(1 << 0));
	REG_SLCD_DATA = (0 << 31) | (data&0xffff);
}

/* Sent a command with data (18-bit bus, 16-bit index, 16-bit register value) */
static void Mcupanel_RegSet(unsigned int cmd, unsigned int data)
{
	/*printk("CMD = 0x%x, data=x%x\n",cmd,data);*/

	while ((REG_SLCD_STATE)&(1 << 0));
	REG_SLCD_DATA = (1 << 31) | ((cmd&0xff) >> 0);
	while ((REG_SLCD_STATE)&(1 << 0));
	REG_SLCD_DATA=(0 << 31) | (data&0x00ff);
}

/* Sent a command without data  (18-bit bus, 16-bit index) */
static void Mcupanel_Command(unsigned int cmd) {
	while ((REG_SLCD_STATE)&(1 << 0));
	REG_SLCD_DATA =  (1 << 31) | ((cmd&0xff00) >> 8);
	while ((REG_SLCD_STATE)&(1 << 0));
	REG_SLCD_DATA =(1 << 31) | ((cmd&0xff) >> 0);
}
#endif

static void LCD_Config_Data(unsigned int data_h,unsigned int data_l)
{
	while((REG_SLCD_STATE) & (1 << 0));
	REG_SLCD_DATA = (0 << 31) | ((data_l & 0xff) >> 0);
}

static void LCD_Config_Command(unsigned int cmd_h,unsigned int cmd_l)
{
	while((REG_SLCD_STATE) & (1 << 0));
	REG_SLCD_DATA = (1 << 31) | ((cmd_l & 0xff) >> 0);
}

/* Set the start address of screen, for example (0, 0) */
void Mcupanel_SetAddr(u32 x, u32 y) //u32
{
	int x1 = x + 32;
	//Mcupanel_RegSet(0x20,x) ;
	LCD_Config_Command(0x00,0x2b);
	LCD_Config_Data(0x00,(y & 0xff00) >> 8);
	LCD_Config_Data(0x00,(y & 0xff));
	LCD_Config_Data(0x00,0x00);
	LCD_Config_Data(0x00,0xef);

	udelay(1);

	//Mcupanel_RegSet(0x21,y) ;
	LCD_Config_Command(0x00,0x2a);
	LCD_Config_Data(0x00,(x1 & 0xff00) >> 8);
	LCD_Config_Data(0x00,(x1 & 0xff));
	LCD_Config_Data(0x00,0x01);
	LCD_Config_Data(0x00,0xaf);

	udelay(1);
	LCD_Config_Command(0x00,0x2c);
}

#undef __lcd_special_pin_init
#define __lcd_special_pin_init()			\
do {							\
	__gpio_as_output(PIN_CS_N);			\
	__gpio_as_output(PIN_RESET_N);			\
	__gpio_clear_pin(PIN_CS_N); /* Clear CS */	\
	mdelay(100);					\
	__gpio_set_pin(PIN_RESET_N);			\
	mdelay(50);					\
	__gpio_clear_pin(PIN_RESET_N);			\
	mdelay(50);					\
	__gpio_set_pin(PIN_RESET_N);			\
	mdelay(100);					\
} while(0)

#undef SlcdInit
#define SlcdInit()                     \
do {                                   \
	mdelay(50);                    \
	LCD_Config_Command(0x00,0xE9); \
	LCD_Config_Data(0x00,0x20);    \
	LCD_Config_Command(0x00,0x11); \
	mdelay(100);                   \
	LCD_Config_Command(0x00,0xd0); \
	LCD_Config_Data(0x00,0x07);    \
	LCD_Config_Data(0x00,0x01);    \
	LCD_Config_Data(0x00,0x88);    \
	LCD_Config_Command(0x00,0xd1); \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x71);    \
	LCD_Config_Data(0x00,0x19);    \
	LCD_Config_Command(0x00,0x36); \
	LCD_Config_Data(0x00,0xe8);    \
	LCD_Config_Command(0x00,0x3a); \
	LCD_Config_Data(0x00,0x55);    \
	LCD_Config_Command(0x00,0xc1); \
	LCD_Config_Data(0x00,0x10);    \
	LCD_Config_Data(0x00,0x10);    \
	LCD_Config_Data(0x00,0x02);    \
	LCD_Config_Data(0x00,0x02);    \
	LCD_Config_Command(0x00,0xc0); \
	LCD_Config_Data(0x00,0x10);    \
	LCD_Config_Data(0x00,0x35);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x01);    \
	LCD_Config_Data(0x00,0x02);    \
	LCD_Config_Command(0x00,0xc5); \
	LCD_Config_Data(0x00,0x02);    \
	LCD_Config_Command(0x00,0xd2); \
	LCD_Config_Data(0x00,0x01);    \
	LCD_Config_Data(0x00,0x44);    \
	LCD_Config_Command(0x00,0xc8); \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x40);    \
	LCD_Config_Data(0x00,0x24);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x0c);    \
	LCD_Config_Data(0x00,0x02);    \
	LCD_Config_Data(0x00,0x35);    \
	LCD_Config_Data(0x00,0x73);    \
	LCD_Config_Data(0x00,0x77);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x02);    \
	LCD_Config_Data(0x00,0x0c);    \
	LCD_Config_Data(0x00,0x08);    \
	LCD_Config_Data(0x00,0x80);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Command(0x00,0xea); \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0xc0);    \
	LCD_Config_Command(0x00,0x21); \
	LCD_Config_Command(0x00,0x29); \
	LCD_Config_Data(0x00,0x80);    \
	LCD_Config_Command(0x00,0x2a); \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x20);    \
	LCD_Config_Data(0x00,0x01);    \
	LCD_Config_Data(0x00,0xaf);    \
	LCD_Config_Command(0x00,0x2b); \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0x00);    \
	LCD_Config_Data(0x00,0xef);    \
	LCD_Config_Command(0x00,0x2c); \
} while(0);

/*---- LCD Initial ----*/
#undef __lcd_slcd_pin_init
#define __lcd_slcd_pin_init()					\
do {								\
	__gpio_as_slcd_8bit();					\
	mdelay(10);						\
	__lcd_special_pin_init();				\
}while (0)

#undef __init_slcd_bus
#define __init_slcd_bus()\
do{\
	__slcd_set_data_8bit_x2();\
	__slcd_set_cmd_8bit();\
	__slcd_set_cs_low();\
	__slcd_set_rs_low();\
	__slcd_set_clk_falling();\
	__slcd_set_parallel_type();\
} while(0)

#undef __lcd_slcd_special_on
#define __lcd_slcd_special_on()						\
do {									\
	__lcd_slcd_pin_init();						\
	REG_LCD_CTRL &= ~(LCD_CTRL_ENA|LCD_CTRL_DIS); /* disable lcdc */ \
	REG_LCD_CFG = LCD_CFG_LCDPIN_SLCD | 0x0D; /* LCM */		\
	REG_SLCD_CTRL &= ~SLCD_CTRL_DMA_EN; /* disable slcd dma */	\
	REG_SLCD_CFG = SLCD_CFG_DWIDTH_8BIT_x1 | SLCD_CFG_CWIDTH_8BIT |	\
			SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW |	\
			SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL; \
	REG_LCD_REV = 0x04;	/* lcd clock??? */			\
	SlcdInit();							\
	/*alterac add*/							\
	Mcupanel_SetAddr(0, 0);						\
	REG_SLCD_CFG = SLCD_CFG_DWIDTH_8BIT_x2 | SLCD_CFG_CWIDTH_8BIT |	\
			SLCD_CFG_CS_ACTIVE_LOW | SLCD_CFG_RS_CMD_LOW | 	\
			SLCD_CFG_CLK_ACTIVE_FALLING | SLCD_CFG_TYPE_PARALLEL; \
	/*Mcupanel_Command(0x22);*/					\
	/*while ((REG_SLCD_STATE)&(1 << 0));*/				\
	/*__slcd_enable_dma();	*/					\
	REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; /* slcdc dma enable */	\
	REG_LCD_CTRL  |= (LCD_CTRL_ENA|LCD_CTRL_DIS); /* disable lcdc */ \
} while (0)

#endif	/* CONFIG_JZ4750_SLCD_A380_ILI9331 */

#endif  /* _A380_ILI9331_H_ */

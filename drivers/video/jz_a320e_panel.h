#ifndef __JZ_8352B_H__
#define __JZ_8352B_H__

/* 400x240 board A320E */
#if defined(CONFIG_JZ4750_SLCD_400X240_8352B)
#define WR_GRAM_CMD	0x22

#define PIN_CS_N 	(32*3+18)	// GPD18
#define PIN_RESET_N 	(32*3+21)	// GPD21

/* Sent a command with data (18-bit bus, 16-bit index, 16-bit register value) */
static void Mcupanel_RegSet(unsigned int cmd, unsigned int data)
{
	cmd = (cmd & 0xff);
	data = (data & 0xff);
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | cmd;
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_DATA | data;
}

/* Sent a command without data  (18-bit bus, 16-bit index) */
static void Mcupanel_Command(unsigned int cmd) {
	while (REG_SLCD_STATE & SLCD_STATE_BUSY);
	REG_SLCD_DATA = SLCD_DATA_RS_COMMAND | (cmd&0xff);
}

/* Set the start address of screen, for example (0, 0) */
void Mcupanel_SetAddr(u32 x, u32 y) //u32
{

	Mcupanel_RegSet(0x02,0x00); // SC[15:8]
	Mcupanel_RegSet(0x03,0x00); // SC[7:0]
	Mcupanel_RegSet(0x04,0x01); // EC[15:8]
	Mcupanel_RegSet(0x05,0x8F); // EC[7:0]
	Mcupanel_RegSet(0x06,0x00); // SP[15:8]
	Mcupanel_RegSet(0x07,0x00); // SP[7:0]
	Mcupanel_RegSet(0x08,0x00); // EP[15:8]
	Mcupanel_RegSet(0x09,0xEF); // EP[7:0]
	Mcupanel_Command(0x22);

}

void Mcupanel_SetAddr_Reset(u32 x, u32 y) //u32
{

	Mcupanel_RegSet(0x02,0x00); // SC[15:8]
	Mcupanel_RegSet(0x03,0x00); // SC[7:0]
	Mcupanel_RegSet(0x04,0x00); // EC[15:8]
	Mcupanel_RegSet(0x05,0x00); // EC[7:0]
	Mcupanel_RegSet(0x06,0x00); // SP[15:8]
	Mcupanel_RegSet(0x07,0x00); // SP[7:0]
	Mcupanel_RegSet(0x08,0x00); // EP[15:8]
	Mcupanel_RegSet(0x09,0x00); // EP[7:0]
	Mcupanel_Command(0x22);

}
#undef __lcd_special_pin_init
#define __lcd_special_pin_init() \
	do {	\
		__gpio_as_output(PIN_CS_N);	\
		__gpio_as_output(PIN_RESET_N);	\
		__gpio_clear_pin(PIN_CS_N); /* Clear CS */	\
		mdelay(100);	\
		__gpio_set_pin(PIN_RESET_N);	\
		mdelay(10);	\
		__gpio_clear_pin(PIN_RESET_N);	\
		mdelay(100);	\
		__gpio_set_pin(PIN_RESET_N);	\
		mdelay(10);	\
	} while(0)

#define RegSetInit()\
	do {      \
		Mcupanel_RegSet(0xE5,0x18);\
		Mcupanel_RegSet(0xE7,0x18);\
		Mcupanel_RegSet(0xE8,0x64);\
		Mcupanel_RegSet(0xEC,0x08);\
		Mcupanel_RegSet(0xED,0x47);\
		Mcupanel_RegSet(0xEE,0x20);\
		Mcupanel_RegSet(0xEF,0x50);\
\
		Mcupanel_RegSet(0x23,0x83);\
		Mcupanel_RegSet(0x24,0x79);\
		Mcupanel_RegSet(0x25,0x4F);\
		Mcupanel_RegSet(0x29,0x00);\
		Mcupanel_RegSet(0x2B,0x03);\
		Mcupanel_RegSet(0xE2,0x1A);\
		Mcupanel_RegSet(0x1B,0x2B);\
\
		Mcupanel_RegSet(0x36,0x00);\
\
\
		Mcupanel_RegSet(0x01,0x00);\
\
		Mcupanel_RegSet(0x1A,0x02);\
		Mcupanel_RegSet(0x1C,0x03);\
		Mcupanel_RegSet(0x19,0x01);\
		Mcupanel_RegSet(0x18,0x0C);\
		mdelay(5);\
		Mcupanel_RegSet(0x1F,0x90);\
		mdelay(10);\
		Mcupanel_RegSet(0x1F,0xD4);\
		mdelay(5);\
\
		Mcupanel_RegSet(0x40,0x00);\
		Mcupanel_RegSet(0x41,0x29);\
		Mcupanel_RegSet(0x42,0x26);\
		Mcupanel_RegSet(0x43,0x3E);\
		Mcupanel_RegSet(0x44,0x3D);\
		Mcupanel_RegSet(0x45,0x3F);\
		Mcupanel_RegSet(0x46,0x1B);\
		Mcupanel_RegSet(0x47,0x68);\
		Mcupanel_RegSet(0x48,0x04);\
		Mcupanel_RegSet(0x49,0x05);\
		Mcupanel_RegSet(0x4A,0x06);\
		Mcupanel_RegSet(0x4B,0x0C);\
		Mcupanel_RegSet(0x4C,0x17);\
		Mcupanel_RegSet(0x50,0x00);\
		Mcupanel_RegSet(0x51,0x02);\
		Mcupanel_RegSet(0x52,0x01);\
		Mcupanel_RegSet(0x53,0x19);\
		Mcupanel_RegSet(0x54,0x16);\
		Mcupanel_RegSet(0x55,0x3F);\
		Mcupanel_RegSet(0x56,0x17);\
		Mcupanel_RegSet(0x57,0x64);\
\
		Mcupanel_RegSet(0x58,0x08);\
		Mcupanel_RegSet(0x59,0x13);\
		Mcupanel_RegSet(0x5A,0x19);\
		Mcupanel_RegSet(0x5B,0x1A);\
		Mcupanel_RegSet(0x5C,0x1B);\
		Mcupanel_RegSet(0x5D,0xFF);\
\
		Mcupanel_RegSet(0x28,0x20);\
		mdelay(10);\
		Mcupanel_RegSet(0x28,0x38);\
		mdelay(10);\
		Mcupanel_RegSet(0x28,0x3C);\
		Mcupanel_RegSet(0x17,0x05);\
		Mcupanel_RegSet(0x16,0x68);\
	}while(0);

#define SlcdInit()	\
	do {      \
		RegSetInit();\
		Mcupanel_SetAddr(0,0);\
			udelay(100);\
	} while(0)

	
#define SlcdReInit()	\
	do {      \
		RegSetInit();\
		/*Mcupanel_SetAddr_Reset(0,0);*/\
		udelay(100);\
	} while(0)

/*---- LCD Initial ----*/
#undef __lcd_slcd_pin_init
#define __lcd_slcd_pin_init()						\
	do {								\
		__lcd_special_pin_init();				\
	}while (0)

#undef __lcd_slcd_special_on
#define __lcd_slcd_special_on()						\
	do {	\
		SlcdInit();						\
		REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; /* slcdc dma enable */ \
	} while (0)


#define __lcd_slcd_special_reset()						\
	do {	\
		SlcdReInit();						\
		REG_SLCD_CTRL |= SLCD_CTRL_DMA_EN; /* slcdc dma enable */ \
	} while (0)

#define __init_slcd_bus()\
	do{\
		__slcd_set_data_16bit();\
		__slcd_set_cmd_16bit();\
		__slcd_set_cs_low();\
		__slcd_set_rs_low();\
		__slcd_set_clk_falling();\
		__slcd_set_parallel_type();\
	}while(0)

#endif  /* CONFIG_JZ4750_SLCD_400X240_8352B */

#endif  /* __JZ_8352B_H__ */

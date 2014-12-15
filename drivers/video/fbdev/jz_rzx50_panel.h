#ifndef __JZ_PT035TN01_H__
#define __JZ_PT035TN01_H__

/* 480x272 board RZX50 */
#if defined(CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL)

#define MODE 0xc9		/* 8bit serial RGB */

/* SPI pins */
#define SPEN		(32*3+22)	/*LCD_CS*/
#define SPCK		(32*4+13)	/*LCD_SCL*/
#define SPDA		(32*4+12)	/*LCD_SDA*/
#define LCD_RET 	(32*4+2)	/*LCD_DISP_N use for lcd reset*/

static void __spi_write_reg1(unsigned char reg, unsigned char val)
{
	int no;
	unsigned short value;

	__gpio_set_pin(SPEN);
	__gpio_set_pin(SPCK);
	__gpio_clear_pin(SPDA);
	__gpio_clear_pin(SPEN);
	udelay(25);
	value = ((reg << 8) | (val & 0xFF));
	for(no = 0; no < 16; no++) {
		__gpio_clear_pin(SPCK);
		if ((value & 0x8000) == 0x8000)
			__gpio_set_pin(SPDA);
		else
			__gpio_clear_pin(SPDA);
		udelay(25);
		__gpio_set_pin(SPCK);
		value <<= 1; 
		udelay(25);
	 }
	__gpio_set_pin(SPEN);
	udelay(100);
}

static void __spi_write_reg(unsigned char reg, unsigned char val)
{
	__spi_write_reg1((reg<<2|2), val);
	udelay(100);
}

static int rzx50_panel_init(void **out_panel, struct device *dev,
			   void *panel_pdata)
{
	__gpio_as_output(SPEN);
	__gpio_as_output(SPCK);
	__gpio_as_output(SPDA);
	__gpio_as_output(LCD_RET);
	udelay(50);
	__gpio_clear_pin(LCD_RET);
	mdelay(150);
	__gpio_set_pin(LCD_RET);

	return 0;
}

static void rzx50_panel_exit(void *panel)
{
}

static void rzx50_panel_enable(void *panel)
{
	udelay(50);
	__gpio_clear_pin(LCD_RET);
	mdelay(150);
	__gpio_set_pin(LCD_RET);
	mdelay(10);
	__spi_write_reg(0x00, 0x03);
	__spi_write_reg(0x01, 0x40);
	__spi_write_reg(0x02, 0x11);
	__spi_write_reg(0x03, MODE);
	__spi_write_reg(0x04, 0x32);
	__spi_write_reg(0x05, 0x0e);
	__spi_write_reg(0x07, 0x03);
	__spi_write_reg(0x08, 0x08);
	__spi_write_reg(0x09, 0x32);
	__spi_write_reg(0x0A, 0x88);
	__spi_write_reg(0x0B, 0xc6);
	__spi_write_reg(0x0C, 0x20);
	__spi_write_reg(0x0D, 0x20);
}

static void rzx50_panel_disable(void *panel)
{
	__spi_write_reg(0x00, 0x03);
}

/* TODO: Find out the real lcd model name */
struct panel_ops rzx50_panel_ops = {
	.init		= rzx50_panel_init,
	.exit		= rzx50_panel_exit,
	.enable		= rzx50_panel_enable,
	.disable	= rzx50_panel_disable,
};

/* FIXME: this will be gone when panel code is moved to separate .c file */
#define jz4750_lcd_panel_ops rzx50_panel_ops

#endif /* CONFIG_JZ4750_LCD_INNOLUX_PT035TN01_SERIAL */

#endif /* __JZ_PT035TN01_H__ */

/* The MIT License
 *
 * Copyright (c) 2020 Piotr Duba
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "lcd_i2c.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "tim.h"

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void lcd_init(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;
	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* init sequence */
	HAL_Delay(40);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(5);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);
	HAL_Delay(1);
	lcd_write(lcd->addr, INIT_8_BIT_MODE, xpin);

	/* set 4-bit mode */
	lcd_write(lcd->addr, INIT_4_BIT_MODE, xpin);

	/* set cursor mode */
	lcd_write(lcd->addr, UNDERLINE_OFF_BLINK_OFF, xpin);

	/* clear */
	lcd_clear(lcd);

}

void lcd_write(uint8_t addr, uint8_t data, uint8_t xpin)
{
	uint8_t tx_data[4];

	/* split data */
	tx_data[0] = (data & 0xF0) | EN_PIN | xpin;
	tx_data[1] = (data & 0xF0) | xpin;
	tx_data[2] = (data << 4) | EN_PIN | xpin;
	tx_data[3] = (data << 4) | xpin;

	/* send data via i2c */
	HAL_I2C_Master_Transmit(&HI2C_DEF, addr, tx_data, 4, 100);
	delay_us(120);
	//HAL_Delay(1);
}

void lcd_display(struct lcd_disp * lcd)
{
	uint8_t xpin = 0, i = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	//lcd_clear(lcd);

	/* send first line data */
	lcd_write(lcd->addr, FIRST_CHAR_LINE_1, xpin);
	while(lcd->first_line[i])
	{
		lcd_write(lcd->addr, lcd->first_line[i], (xpin | RS_PIN));
		i++;
	}

	/* send second line data */
	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_2, xpin);
	while(lcd->second_line[i])
	{
		lcd_write(lcd->addr, lcd->second_line[i], (xpin | RS_PIN));
		i++;
	}

	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_3, xpin);
	while(lcd->third_line[i])
	{
		lcd_write(lcd->addr, lcd->third_line[i], (xpin | RS_PIN));
		i++;
	}

	i = 0;
	lcd_write(lcd->addr, FIRST_CHAR_LINE_4, xpin);
	while(lcd->fourth_line[i])
	{
		lcd_write(lcd->addr, lcd->fourth_line[i], (xpin | RS_PIN));
		i++;
	}
}

void lcd_clear(struct lcd_disp * lcd)
{
	uint8_t xpin = 0;

	/* set backlight */
	if(lcd->bl)
	{
		xpin = BL_PIN;
	}

	/* clear display */
	lcd_write(lcd->addr, CLEAR_LCD, xpin);
}

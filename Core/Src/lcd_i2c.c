/*
 * lcd_i2c.c
 *
 *  Created on: Feb 8, 2024
 *      Author: danie
 */
#include "lcd_i2c.h"
I2C_HandleTypeDef hi2c1; //extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly


void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
  // rs -> register select: Toggles between Command/Data Register
	// 0 -> Command Mode, 1-> Data Mode
  //            0  1 2  3  4  5  6  7
  // Data Byte: RS X EN BL D4 D5 D6 D7

  uint8_t data = nibble << D4_BIT;
  data |= rs << RS_BIT;
  data |= 1 << BL_BIT; // Include backlight state in data
  data |= 1 << EN_BIT;
  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
  HAL_Delay(1);
  data &= ~(1 << EN_BIT);
  HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}

// set rs to 0 to send command
void lcd_send_cmd(uint8_t cmd) {
  uint8_t upper_nibble = cmd >> 4;
  uint8_t lower_nibble = cmd & 0x0F;
  lcd_write_nibble(upper_nibble, 0);
  lcd_write_nibble(lower_nibble, 0);
  if (cmd == 0x01 || cmd == 0x02) {
    HAL_Delay(2);
  }
}

// set rs to 1 to send data
void lcd_send_data(uint8_t data) {
  uint8_t upper_nibble = data >> 4;
  uint8_t lower_nibble = data & 0x0F;
  lcd_write_nibble(upper_nibble, 1);
  lcd_write_nibble(lower_nibble, 1);
}

void lcd_init() {
  HAL_Delay(50);
  lcd_write_nibble(0x03, 0); // rs = 0 -> command mode
  HAL_Delay(5);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x03, 0);
  HAL_Delay(1);
  lcd_write_nibble(0x02, 0);
  lcd_send_cmd(0x28);  // 001 8/4 2/1 10/7 X X -> b'0010 1000'  enable 4-bit bus mode, 2 lines preview, and the 5x7 dot format
  lcd_send_cmd(0x0C);  // 0000 1DUB -> Bit 0: Cursor blink On/Off, Bit 1: cursor underline, Bit 2: Display On/Off -> 0x0C = 0000 1100
  lcd_send_cmd(0x06);  // character entry set mode? 0000 01 I/D S -> bit 0: display shift On/Off, Bit 1: Address counter inc/dec
  lcd_send_cmd(0x01);  // Clear display
  HAL_Delay(2);
}

void lcd_write_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
    uint8_t address;
    switch (row) {
        case 0:
            address = 0x00;
            break;
        case 1:
            address = 0x40;
            break;
        default:
            address = 0x00;
    }
    address += column;
    lcd_send_cmd(0x80 | address);
}

void lcd_clear(void) {
	lcd_send_cmd(0x01);
    HAL_Delay(2);
}

//void lcd_backlight(uint8_t state) {
//  if (state) {
//    backlight_state = 1;
//  } else {
//    backlight_state = 0;
//  }
//}



/*
 * lcd.h
 *
 *  Created on: Jan 8, 2024
 *      Author: denis
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "main.h"
#include "cmsis_os.h"
void LCD_SendHalfByte(uint8_t info);
void LCD_Send(uint8_t info, uint8_t mode);
void LCD_SetCursor(uint8_t col);
void LCD_Clear();
void LCD_PrintString(uint8_t array[], uint8_t size);
void LCD_PrintDouble(double value);
void LCD_Init();

#endif /* INC_LCD_H_ */

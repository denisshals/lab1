/*
 * lcd.c
 *
 *  Created on: Jan 7, 2024
 *      Author: denis
 */
#include "lcd.h"

void LCD_SendHalfByte(uint8_t info) {
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, info & 1);
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (info >> 1) & 1);
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (info >> 2) & 1);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (info >> 3) & 1);
	vTaskDelay(20);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	vTaskDelay(20);
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
}
void LCD_Send(uint8_t info, uint8_t mode) {
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, mode);
    LCD_SendHalfByte(info >> 4);
    LCD_SendHalfByte(info);
}
void LCD_SetCursor(uint8_t col) {
	uint8_t address = col + (col < 9 ? - 1 : 0x37);
    //uint8_t address = col + (row == 1 ? 0 : 0x40) - 1;
    LCD_Send(0x80 | address, 0);
}
void LCD_Clear() {
    LCD_Send(0b00000001, 0);
    vTaskDelay(20);
}
void LCD_PrintString(uint8_t array[], uint8_t size) {
	for(uint8_t i = 0; i < size; i++) LCD_Send(array[i], 1);
}
void LCD_PrintDouble(double value) {
	value *= 1000000.0;
	uint16_t integerPart = (uint16_t) value;
	uint8_t floatPart = (uint8_t) (value * 100 - integerPart * 100);
	uint8_t array[8];
	array[0] = (uint8_t) (integerPart / 100) + 0x30;
	array[1] = (uint8_t) (integerPart % 100 / 10) + 0x30;
	array[2] = (uint8_t) (integerPart % 10) + 0x30;
	array[3] = 0x2E;
	array[4] = (uint8_t) (floatPart / 10) + 0x30;
	array[5] = (uint8_t) (floatPart % 10) + 0x30;
	array[6] = 0x75;
	array[7] = 0x41;
	LCD_PrintString(array, 8);
}
void LCD_Init() {
	LCD_Send(0b00000010, 0);
    LCD_Send(0b00101000, 0); // Function Set
    LCD_Send(0b00001100, 0); // Display On
    LCD_Clear();
    LCD_Send(0b00000110, 0); // Entry Set
}


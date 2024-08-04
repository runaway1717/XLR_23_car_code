/*
 * Segment.h
 *
 *  Created on: Mar 15, 2023
 *      Author: Pramsu
 */

#ifndef INC_SEGMENT_H_
#define INC_SEGMENT_H_

#include "stm32f4xx_hal.h"

uint8_t encodeDigit(uint8_t digit); // Done
void clear(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
void setSegments(uint8_t segments, uint8_t length, uint8_t pos, uint8_t br, int on, GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin); // Done


/* These are segment specific functions. Do not modify them. They write on the seven segment display. */
void TM1637_Init(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
//void TM1637_Demo(void); // Removed
void TM1637_DisplayDecimal(int v, int displaySeparator);
void TM1637_SetBrightness(char brightness, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
void TM1637_Start(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
void TM1637_Stop(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
void TM1637_ReadResult(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
void TM1637_WriteByte(unsigned char b, GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t); // Done
void TM1637_DelayUsec(unsigned int i); // Done
void TM1637_ClkHigh(GPIO_TypeDef*, uint16_t); // Done
void TM1637_ClkLow(GPIO_TypeDef*, uint16_t); // Done
void TM1637_DataHigh(GPIO_TypeDef*, uint16_t); // Done
void TM1637_DATALow(GPIO_TypeDef*, uint16_t); // Done
void displayDecimal(int value, GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin); //Done

#endif /* INC_SEGMENT_H_ */

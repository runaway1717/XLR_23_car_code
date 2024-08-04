/*
 * segment.c
 *
 *  Created on: Mar 15, 2023
 *      Author: Pramsu
 */

#include "segment.h"
#include "main.h"
#include "stm32f4xx_hal.h"



GPIO_TypeDef* DATA_GPIO_Port;
//GPIO_TypeDef* CLK_GPIO_Port;

uint16_t DATA_Pin;
//uint16_t CLK_Pin;

//#define DATA_GPIO_Port GPIOD
//#define DATA_Pin GPIO_PIN_2

//#define CLK_GPIO_Port GPIOC
//#define CLK_Pin GPIO_PIN_12

uint8_t data[] = { 0xff, 0xff, 0xff, 0xff , 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };

uint8_t encodeDigit(uint8_t digit){
	return digitToSegment[digit & 0x0f];
}

const char segmentMap[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0-7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8-9, A-F
    0x00
};

void TM1637_ClkHigh(GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
    HAL_GPIO_WritePin(Clk_GPIO_Port, Clk_Pin, GPIO_PIN_SET);
}

void TM1637_ClkLow(GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
    HAL_GPIO_WritePin(Clk_GPIO_Port, Clk_Pin, GPIO_PIN_RESET);
}

void TM1637_DataHigh(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin)
{
    HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_SET);
}

void TM1637_DataLow(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin)
{
    HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);
}


void TM1637_Init(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
//	HAL_GPIO_Init(GPIOC, GPIO_Init);
//	HAL_GPIO_Init(GPIOD, GPIO_Init);
    // TM1637_SetBrightness(0x0f);
    TM1637_ClkHigh(Clk_GPIO_Port, Clk_Pin);
    TM1637_DataHigh(DATA_GPIO_Port, DATA_Pin);
    TM1637_ClkLow(Clk_GPIO_Port, Clk_Pin);
    TM1637_DataLow(DATA_GPIO_Port, DATA_Pin);
    for(int i = 0; i < 10; i++) data[i] = encodeDigit(i);
}

void clear(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin){
//    uint8_t data_send[] = {0,0,0,0};
	for(int j = 0; j < 4; j++)
    setSegments(0, j, 0, 0, 0, DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
}

void TM1637_Start(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
    TM1637_ClkHigh(Clk_GPIO_Port, Clk_Pin);
    TM1637_DataHigh(DATA_GPIO_Port, DATA_Pin);
    TM1637_DelayUsec(2);
    TM1637_DataLow(DATA_GPIO_Port, DATA_Pin);
}

void TM1637_Stop(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
    TM1637_ClkLow(Clk_GPIO_Port, Clk_Pin);
    TM1637_DelayUsec(2);
    TM1637_DataLow(DATA_GPIO_Port, DATA_Pin);
    TM1637_DelayUsec(2);
    TM1637_ClkHigh(Clk_GPIO_Port, Clk_Pin);
    TM1637_DelayUsec(2);
    TM1637_DataHigh(DATA_GPIO_Port, DATA_Pin);
}

void TM1637_ReadResult(GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
    TM1637_ClkLow(Clk_GPIO_Port, Clk_Pin);
    TM1637_DelayUsec(5);

    TM1637_ClkHigh(Clk_GPIO_Port, Clk_Pin);
    TM1637_DelayUsec(2);
    TM1637_ClkLow(Clk_GPIO_Port, Clk_Pin);
}

void TM1637_WriteByte(unsigned char b, GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
	for (int i = 0; i < 8; ++i) {
		TM1637_ClkLow(Clk_GPIO_Port, Clk_Pin);
		if (b & 0x01) { // odd -> 1; even -> 0
			TM1637_DataHigh(DATA_GPIO_Port, DATA_Pin);
		}
		else {
			TM1637_DataLow(DATA_GPIO_Port, DATA_Pin);
		}
		TM1637_DelayUsec(3);
		b >>= 1;
		TM1637_ClkHigh(Clk_GPIO_Port, Clk_Pin);
		TM1637_DelayUsec(3);
	}
}

void TM1637_DelayUsec(unsigned int i)
{
    for (; i>0; i--) {
        for (int j = 0; j < 500; ++j) {
					__NOP();
        }
    }
}

void TM1637_SetBrightness(char brightness, GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin)
{
    // Brightness command:
    // 1000 0XXX = display off
    // 1000 1BBB = display on, brightness 0-7
    // X = don't care
    // B = brightness
    TM1637_Start(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_WriteByte(0x87 + brightness, DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_ReadResult(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_Stop(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
}

void setSegments(uint8_t segments, uint8_t length, uint8_t pos, uint8_t br, int on, GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin){

    TM1637_Start(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_WriteByte(0x40, DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_ReadResult(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_Stop(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);

    TM1637_Start(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_WriteByte(0xc0 + (pos  & 0x03), DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
    TM1637_ReadResult(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);

	  TM1637_WriteByte(segments, DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
      TM1637_ReadResult(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);

      TM1637_Stop(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);

//   uint8_t m_brightness =  (br & 0x7) | (on? 0x08 : 0x00);
//	uint8_t m_brightness = (br | 0x0f);
//	// Write COMM3 + brightness
//	TM1637_Start(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
//	TM1637_WriteByte(0x80 + (m_brightness & 0x0f), DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
//	TM1637_ReadResult(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
//    TM1637_Stop(DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);

}

void displayDecimal(int value, GPIO_TypeDef* DATA_GPIO_Port, uint16_t DATA_Pin, GPIO_TypeDef* Clk_GPIO_Port, uint16_t Clk_Pin){

	int data_send[] = {0,0,0,0};

	for(int j = 3; j > -1; j--){
				  data_send[j] = value%10;
				  value = value/10;
			  }

	for(int j = 1; j < 4; j++){
	        setSegments(data[data_send[j]], 1, j, 0x0f, 1, DATA_GPIO_Port, DATA_Pin, Clk_GPIO_Port, Clk_Pin);
	      }

}

/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  * Use				   : Function 1 - ADC_Read(&hadcx, uint32_t y)
  * 								  Parameters - x is ADC number, y is channel.
  * 					 			  Returns    - 0 to 4095 map of ADC Value
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
uint16_t ADC_Read(ADC_HandleTypeDef* hadc, uint32_t ADC_CH);
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

uint16_t ADC_Read(ADC_HandleTypeDef* hadc, uint32_t ADC_CH)
{
	uint32_t val;
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CH;
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(hadc, &sConfig);
	HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 1);
	val = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);
	return val;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


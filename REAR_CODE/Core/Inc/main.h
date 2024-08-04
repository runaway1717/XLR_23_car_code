/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor1_Pin GPIO_PIN_3
#define Motor1_GPIO_Port GPIOF
#define Motor2_Pin GPIO_PIN_4
#define Motor2_GPIO_Port GPIOF
#define IR_OUT_Pin GPIO_PIN_9
#define IR_OUT_GPIO_Port GPIOF
#define Damper1_Pin GPIO_PIN_2
#define Damper1_GPIO_Port GPIOC
#define Damper2_Pin GPIO_PIN_3
#define Damper2_GPIO_Port GPIOC
#define LV_Batt_Pin GPIO_PIN_2
#define LV_Batt_GPIO_Port GPIOA
#define FR1_Pin GPIO_PIN_6
#define FR1_GPIO_Port GPIOA
#define FR1_EXTI_IRQn EXTI9_5_IRQn
#define FR2_Pin GPIO_PIN_7
#define FR2_GPIO_Port GPIOA
#define FR2_EXTI_IRQn EXTI9_5_IRQn
#define CAN_LED_Pin GPIO_PIN_4
#define CAN_LED_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOC
#define LED_5_Pin GPIO_PIN_0
#define LED_5_GPIO_Port GPIOB
#define SPI_CSN_Pin GPIO_PIN_15
#define SPI_CSN_GPIO_Port GPIOE
#define TP2_SCL_Pin GPIO_PIN_6
#define TP2_SCL_GPIO_Port GPIOB
#define TP2_SDA_Pin GPIO_PIN_7
#define TP2_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

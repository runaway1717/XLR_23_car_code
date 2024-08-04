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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I1_BRK_SWITCH_Pin GPIO_PIN_1
#define I1_BRK_SWITCH_GPIO_Port GPIOF
#define I1_FOR_EN_Pin GPIO_PIN_2
#define I1_FOR_EN_GPIO_Port GPIOF
#define I1_REV_EN_Pin GPIO_PIN_3
#define I1_REV_EN_GPIO_Port GPIOF
#define I1_REG_DIS_Pin GPIO_PIN_4
#define I1_REG_DIS_GPIO_Port GPIOF
#define IGNITION_Pin GPIO_PIN_6
#define IGNITION_GPIO_Port GPIOA
#define TSMS_Pin GPIO_PIN_1
#define TSMS_GPIO_Port GPIOD
#define SD_ERR_Pin GPIO_PIN_2
#define SD_ERR_GPIO_Port GPIOD
#define BRK_LIGHT_Pin GPIO_PIN_4
#define BRK_LIGHT_GPIO_Port GPIOD
#define I2_BRK_SWITCH_Pin GPIO_PIN_9
#define I2_BRK_SWITCH_GPIO_Port GPIOG
#define I2_FOR_EN_Pin GPIO_PIN_10
#define I2_FOR_EN_GPIO_Port GPIOG
#define I2_REV_EN_Pin GPIO_PIN_11
#define I2_REV_EN_GPIO_Port GPIOG
#define I2_REG_DIS_Pin GPIO_PIN_12
#define I2_REG_DIS_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

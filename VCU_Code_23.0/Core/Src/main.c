/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
float map(uint16_t value, uint16_t min_val, uint16_t max_val, uint16_t offset);
void  CAN_Start();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t adc_apps1;
uint16_t adc_apps2;
float apps1,apps2,apps_avg,torque_demand;
uint16_t range_adc_apps1[] = {710, 2160};
uint16_t range_adc_apps2[] = {1405, 3870};
uint16_t brk_avg;
uint16_t adc_stear_sens;
uint8_t rtds;
uint8_t tsms;
uint8_t tc;
uint8_t tv;
uint16_t brk_light_thres=430;
uint8_t RxData[8];
uint16_t RxData1[4];
uint8_t RxData2[8];
uint32_t id;
uint8_t rxdata[8];
uint8_t clearfault_left=0,clearfault_right=0;
uint8_t left_inv_transmit[8];
uint8_t right_inv_transmit[8];
uint16_t torque_limit=30;
float temp_app;
uint8_t imp;
uint16_t a=0,b=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
+  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, IGNITION_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, SD_ERR_Pin,GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOD, BRK_LIGHT_Pin, GPIO_PIN_SET);
  CAN_Start();
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  apps1=(((float)(adc_apps1 - range_adc_apps1[0])) / ((float)(range_adc_apps1[1]- range_adc_apps1[0])))*100.00;
//	  apps2=(((float)(adc_apps2 - range_adc_apps2[0])) / ((float)(range_adc_apps2[1]- range_adc_apps2[0])))*100.00;
////	  apps1 = map(adc_apps1, range_adc_apps1[0], range_adc_apps1[1],0);
////	  apps2 = map(adc_apps2, range_adc_apps2[0], range_adc_apps2[1],0);
//	  apps_avg=(apps1+apps2)/2;  // goes from 0 to 100
//	  torque_demand=(apps_avg/100)*torque_limit;
//	  a++;

//	  if(HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin)==GPIO_PIN_SET){
//	  		tsms=1;
//	  	}else{
//	  		tsms=0;
//	  }

//	  if(tsms==0){
//	  		rtds=0;
//	  		clearfault_left=0;
//	  		clearfault_right=0;
//	  }

	  if(brk_avg>brk_light_thres){
		  HAL_GPIO_WritePin(GPIOD, BRK_LIGHT_Pin, GPIO_PIN_SET);
	  }else{
		  HAL_GPIO_WritePin(GPIOD, BRK_LIGHT_Pin, GPIO_PIN_RESET);
	  }
	  HAL_GPIO_WritePin(GPIOF, I1_FOR_EN_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, I1_BRK_SWITCH_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, I2_REV_EN_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOG, I2_BRK_SWITCH_Pin,GPIO_PIN_SET);
	  a++;
	  //HAL_GPIO_WritePin(GPIOF, I2_REV_EN_Pin,GPIO_PIN_SET);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, I1_BRK_SWITCH_Pin|I1_FOR_EN_Pin|I1_REV_EN_Pin|I1_REG_DIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IGNITION_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SD_ERR_Pin|BRK_LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, I2_BRK_SWITCH_Pin|I2_FOR_EN_Pin|I2_REV_EN_Pin|I2_REG_DIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : I1_BRK_SWITCH_Pin I1_FOR_EN_Pin I1_REV_EN_Pin I1_REG_DIS_Pin */
  GPIO_InitStruct.Pin = I1_BRK_SWITCH_Pin|I1_FOR_EN_Pin|I1_REV_EN_Pin|I1_REG_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : IGNITION_Pin PA15 */
  GPIO_InitStruct.Pin = IGNITION_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TSMS_Pin */
  GPIO_InitStruct.Pin = TSMS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TSMS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_ERR_Pin BRK_LIGHT_Pin */
  GPIO_InitStruct.Pin = SD_ERR_Pin|BRK_LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2_BRK_SWITCH_Pin I2_FOR_EN_Pin I2_REV_EN_Pin I2_REG_DIS_Pin */
  GPIO_InitStruct.Pin = I2_BRK_SWITCH_Pin|I2_FOR_EN_Pin|I2_REV_EN_Pin|I2_REG_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_Start(){

	CAN_FilterTypeDef filter1, filter2;

	filter1.FilterBank = 0;

	filter1.FilterActivation = ENABLE;
	filter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;

	filter1.FilterScale = CAN_FILTERSCALE_32BIT;
	filter1.FilterMode = CAN_FILTERMODE_IDMASK;

	filter1.FilterMaskIdHigh = 0x0000;
	filter1.FilterMaskIdLow  = 0x0000;

	filter1.FilterIdLow  = 0x0000;
	filter1.FilterIdHigh = 0x0000;

	filter1.SlaveStartFilterBank = 14;


	filter2.FilterBank = 15;

	filter2.FilterActivation = ENABLE;
	filter2.FilterFIFOAssignment = CAN_FILTER_FIFO1;

	filter2.FilterScale = CAN_FILTERSCALE_32BIT;
	filter2.FilterMode = CAN_FILTERMODE_IDMASK;

	filter2.FilterMaskIdHigh = 0x0000;
	filter2.FilterMaskIdLow  = 0x0000;

	filter2.FilterIdLow  = 0x0000;
	filter2.FilterIdHigh = 0x0000;

	filter2.SlaveStartFilterBank = 14;


	if (HAL_CAN_ConfigFilter(&hcan1, &filter1) != HAL_OK) { // Configuring CAN line according to Filter values
		Error_Handler();
	}
	if (HAL_CAN_ConfigFilter(&hcan2, &filter2)!= HAL_OK){
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK) { // CAN start
		Error_Handler();
	}

	if(HAL_CAN_Start(&hcan2) != HAL_OK){
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK){ // Interrupt activation
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN ) != HAL_OK){ // Interrupt activation
			Error_Handler();
		}
}

void CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data_tx){
	    CAN_TxHeaderTypeDef TxHeader;
		uint32_t Txmailbox = 0x00U;
	    TxHeader.DLC=8;
	    TxHeader.StdId=id; // 11 bits
	    TxHeader.IDE=CAN_ID_STD;
	    TxHeader.RTR=CAN_RTR_DATA;

	    if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0) {
	    	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx, &Txmailbox)!= HAL_OK)
	    		Error_Handler();
	    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef RxHeader;
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader,RxData2);
		if(RxHeader.StdId==0x2){
			adc_apps1=((uint16_t)RxData2[1]<<8)|RxData2[0];
			adc_apps2=((uint16_t)RxData2[3]<<8)|RxData2[2];;
			brk_avg=((uint16_t)RxData2[5]<<8)|RxData2[4];
			adc_stear_sens=((uint16_t)RxData2[7]<<8)|RxData2[6];
			if(brk_avg>brk_light_thres){
					  HAL_GPIO_WritePin(GPIOD, BRK_LIGHT_Pin, GPIO_PIN_SET);
				  }else{
					  HAL_GPIO_WritePin(GPIOD, BRK_LIGHT_Pin, GPIO_PIN_RESET);
				  }
//			if(tsms==1&&rtds==0){
//				left_inv_transmit[0]=0;
//				left_inv_transmit[1]=0;
//				left_inv_transmit[2]=0;
//				left_inv_transmit[3]=0;
//				left_inv_transmit[4]=1;
//				left_inv_transmit[5]=0;
//				left_inv_transmit[6]=((torque_limit)*10)%256;
//				left_inv_transmit[7]=((torque_limit)*10)/256;
//
//				right_inv_transmit[0]=0;
//				right_inv_transmit[1]=0;
//				right_inv_transmit[2]=0;
//				right_inv_transmit[3]=0;
//				right_inv_transmit[4]=0;
//				right_inv_transmit[5]=0;
//				right_inv_transmit[6]=((torque_limit)*10)%256;
//				right_inv_transmit[7]=((torque_limit)*10)/256;
//
//				CAN_Tx(&hcan1,0x0C0,right_inv_transmit);
//				CAN_Tx(&hcan1,0x0F0,left_inv_transmit);
//
//			}else if(tsms==1&&rtds==1){
//				left_inv_transmit[0]=(int)((torque_demand)*10)%256;
//				left_inv_transmit[1]=(int)((torque_demand)*10)/256;;
//				left_inv_transmit[2]=0;
//				left_inv_transmit[3]=0;
//				left_inv_transmit[4]=1;
//				left_inv_transmit[5]=1;
//				left_inv_transmit[6]=((torque_limit)*10)%256;
//				left_inv_transmit[7]=((torque_limit)*10)%256;
//
//				right_inv_transmit[0]=(int)((torque_demand)*10)%256;;
//				right_inv_transmit[1]=(int)((torque_demand)*10)/256;;
//				right_inv_transmit[2]=0;
//				right_inv_transmit[3]=0;
//				right_inv_transmit[4]=0;
//				right_inv_transmit[5]=1;
//				right_inv_transmit[6]=((torque_limit)*10)%256;
//				right_inv_transmit[7]=((torque_limit)*10)%256;
//
//				CAN_Tx(&hcan1,0x0C0,right_inv_transmit);
//				CAN_Tx(&hcan1,0x0F0,left_inv_transmit);
//
//				}
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
		}
//		//HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader,RxData1);
//		if(RxHeader.StdId==0x1){
//			tsms=RxData2[2];
//			rtds=RxData2[3];
//			tc=RxData2[4];
//			tv=RxData2[5];
//			imp=RxData2[1];
//			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
//		}
//		if(RxHeader.StdId==0x0C1){
//			if(RxData2[4]==0){
//				clearfault_left=1;
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
//			}
//		}
//		if(RxHeader.StdId==0x0F1){
//			if(RxData2[4]==0){
//				clearfault_right=1;
//			}
//		}
//		if(RxHeader.StdId==0x0C0){
//			rxdata[0]=RxData1[0];
//			rxdata[1]=RxData1[1];
//			rxdata[2]=RxData1[2];
//			rxdata[3]=RxData1[3];
//			rxdata[4]=RxData1[4];
//			rxdata[5]=RxData1[5];
//			rxdata[6]=RxData1[6];
//			rxdata[7]=RxData1[7];
//
//		}

}
float map(uint16_t value, uint16_t min_val, uint16_t max_val, uint16_t offset){
	 temp_app = (((float)(value - min_val)) / ((float)(max_val- min_val)))*100.00;
		if (temp_app < 0.00){
			return 0.00;
		}
		else if (temp_app > 100.00){
			return 100.00;
		}
		else {
			return temp_app;
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

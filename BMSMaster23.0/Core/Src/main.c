/* USER CODE BEGIN Header */
/**
 **************************
 * @file           : main.c
 * @brief          : Main program body
 **************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 **************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  OK = 0,
  Overvoltage = 1,
  Undervoltage = 2,
  Overtemperature = 3,
  Overcurrent = 4
} Error_State_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VOLT_COUNT 12
#define TEMP_COUNT 6

#define STACK_COUNT 8
#define MESSAGE_COUNT 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
typedef struct {
	CAN_HandleTypeDef* hcan;
    uint32_t id;
    uint16_t data[8];
    uint8_t DLC;

} CanMessage;


CanMessage canMessageQueue[50]; //here you define the size of CAN message you want to send sequentially
uint8_t queueHead = 0;
uint8_t queueTail = 0;


uint32_t CAN_Slave_no = 1;
uint32_t CAN_Slave_ID;
uint8_t request_to_slave = 1;
uint8_t TxData4[8];
uint16_t Txdata1[4];
uint8_t Txdata2[8];
uint8_t charging_state = 0; // to store various charging states
uint16_t crnt = 0;

// uint32_t CAN_slave_no;
// uint32_t CAN_slave_ID;

uint16_t TxData1[4];
uint16_t RxData[3];
uint8_t RxData1[8];

uint8_t Temp_Reg[STACK_COUNT][TEMP_COUNT] = {0};
uint16_t Volt_Reg[STACK_COUNT][VOLT_COUNT] = {0};
float Cell_Voltage[96] = {0};
uint8_t Cell_Temperature[48] = {0};
uint16_t Volt_Temp[4] = {0};
uint16_t Current;
int16_t v_out;
int i, chg_mode = 0;
float volt_total;
uint16_t Volt_max, Volt_min;
uint8_t Temp_max, Temp_min, Cur_max_temp = 0, Prev_max_temp = 0, Temp_flag = 0;

uint8_t TxData[8]; // for charger data
uint8_t Ch_fd[8];
uint16_t voltage_fd, current_fd = 0, temp_flag = 0, curr_act;
uint16_t tsms;
//uint32_t tickstart, curr_tick;
uint16_t err_flag_cntr = 0;

float x, x1;
float current, prev_current, curr_current;
float current_value = 0;

Error_State_TypeDef Error_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

void CANStart();
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef hcan, uint32_t id, uint32_t DLC, uint8_t *data);
void CANTransmit(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t DLC, uint8_t *data);
void CAN1Transmit(void);
void CheckErrors(void);
float current_sensor();
void Charger_CANTransmit(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t DLC, uint8_t *data);
uint16_t ADC_Read(uint32_t ADC_CH);
void eq_CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint16_t* data,uint8_t len);
void CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data_tx);
void dq_CAN_Tx(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///Declare CAN_RxHeaderTypeDef for holding details of the complete Rx message/
CAN_RxHeaderTypeDef rxHeader;
CAN_RxHeaderTypeDef rxHeaderForChargerCan;
///Declare CAN_TxHeaderTypeDef for holding details of the complete Tx message/
CAN_TxHeaderTypeDef txHeader;

///Array to hold the received data/
uint16_t rxData[4];
///Array to hold the transmitted data/
uint8_t txData[8];
uint8_t rxDataFromChargerCan[8];

//Adding an array to keep track of the questioning and answering on the CAN line/
uint32_t canHealthMonitoringArray[8];
///Variable to keep the threshold of the number of CAN messages that can be missed before indicating an error/
uint32_t canHealthMonitoringThreshold = 2;
///Variable to act as a flag to tell the latch circuit to activate itself/
uint8_t slaveCanErrorFlag;

///Variable to hold the mail box details used to transmit data/
uint32_t txMailBox = 0x00U;
uint8_t testVar1 = 5;











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
  HAL_Init();

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
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  CANStart();


  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim10);


//  tickstart = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  curr_tick = HAL_GetTick();
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 249;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, comm_led_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_Latch_GPIO_Port, BMS_Latch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : comm_led_Pin PG5 */
  GPIO_InitStruct.Pin = comm_led_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : tsms_Pin */
  GPIO_InitStruct.Pin = tsms_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(tsms_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMS_Latch_Pin */
  GPIO_InitStruct.Pin = BMS_Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMS_Latch_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void CANStart()
{
  CAN_FilterTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

  // Allow two IDs per entry
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterIdHigh = 0x0000;
  filter.FilterBank = 0;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
  {
    Error_Handler();
  }
  filter.FilterBank = 1;
  if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef hcan, uint32_t id, uint32_t DLC, uint8_t *data)
{
  uint32_t mailbox;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = id;
  txHeader.DLC = DLC;

  return HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &mailbox);
}

// void CANTransmit(uint32_t id,uint32_t IDE,uint32_t RTR,uint32_t DLC,uint8_t *data)
//{
//	uint32_t mailbox = 0x00U;
//	CAN_TxHeaderTypeDef Header;
//	Header.StdId= id;
//	Header.IDE= IDE;
//	Header.RTR= RTR;
//	Header.DLC= DLC;
//
//	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
//	  HAL_CAN_AddTxMessage(&hcan1, &Header, data , &mailbox);
//	}
//	if((hcan1.Instance->TSR & CAN_TSR_TXOK0) && (hcan1.Instance->TSR & CAN_TSR_RQCP0))
//		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_10);
//	else
//		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_9);
//
//	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)>0) {
//	  HAL_CAN_AddTxMessage(&hcan2, &Header, data , &mailbox);
//	}
//	if((hcan2.Instance->TSR & CAN_TSR_TXOK0) && (hcan2.Instance->TSR & CAN_TSR_RQCP0))
//		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_10);
//	else
//		HAL_GPIO_TogglePin (GPIOD, GPIO_PIN_9);
//
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) // data collection from slaves in each 50 ms
  {
    // HAL_GPIO_TogglePin(indication1_led_GPIO_Port, indication1_led_Pin);
    TxData4[0] = 5;
    TxData4[1] = 5;
    TxData4[2] = 5;
    TxData4[3] = 5;
    TxData4[4] = 5;
    TxData4[5] = 5;
    TxData4[6] = 5;
    TxData4[7] = 5;

    // CAN_Transmit(hcan2, 100, 8,TxData4);

    if (CAN_Slave_no < 9)
    {
      CAN_Slave_ID = CAN_Slave_no * 100;
      if (CAN_Transmit(hcan2, CAN_Slave_ID, 1, &request_to_slave) == HAL_OK)
      {
        HAL_GPIO_TogglePin(comm_led_GPIO_Port, comm_led_Pin);
      }
//      /Incrementing the value in the can health monitoring array as we ask the question/
      canHealthMonitoringArray[CAN_Slave_no - 1]++;
      if (canHealthMonitoringArray[CAN_Slave_no - 1] > canHealthMonitoringThreshold)
      {
//        /This implies that we have encountered an error and we need to tell the BMS latch circuit/
        slaveCanErrorFlag = SET;
      }
      CAN_Slave_no++;
    }
    else
    {
      CAN_Slave_no = 1;
    }
  }

  //	if(htim->Instance == TIM3)
  //	{
  //		/Perform the charger related tasks/
  //		voltage_fd=Ch_fd[0]*256+Ch_fd[1];
  //		current_fd=Ch_fd[2]*256+Ch_fd[3];
  //		tsms=ADC_Read(ADC_CHANNEL_15);
  //		if(tsms>4000)
  //		{
  //			if(voltage_fd<3984&&Volt_max<4120)
  //			{
  //				uint16_t volt = 3984;
  //				uint16_t cur = 110;
  //				TxData[0]=volt >> 8;
  //				TxData[1]=volt & ((1<<8) - 1);
  //				TxData[2]=cur >> 8;
  //				TxData[3]=cur & ((1<<8) - 1);
  //				TxData[4]=0;
  //				Charger_CANTransmit(0x1806E5F4,1,0,8,(uint8_t *) TxData);
  //			}
  //			if(voltage_fd<3984&&Volt_max>4120)
  //			{
  //				uint16_t volt = 3984;
  //				uint16_t cur = 55;
  //				TxData[0]=volt >> 8;
  //				TxData[1]=volt & ((1<<8) - 1);
  //				TxData[2]=cur >> 8;
  //				TxData[3]=cur & ((1<<8) - 1);
  //				TxData[4]=0;
  //				Charger_CANTransmit(0x1806E5F4,1,0,8,(uint8_t *) TxData);
  //			}
  //			if(voltage_fd<3984&&Volt_max>4180)
  //			{						//?
  //				uint16_t volt = 3984;
  //				uint16_t cur = 0;
  //				TxData[0]=volt >> 8;
  //				TxData[1]=volt & ((1<<8) - 1);
  //				TxData[2]=cur >> 8;
  //				TxData[3]=cur & ((1<<8) - 1);
  //				TxData[4]=1;
  //				Charger_CANTransmit(0x1806E5F4,1,0,8,(uint8_t *) TxData);
  //			}
  //			if(voltage_fd>3984)
  //			{
  //				uint16_t volt = 3984;
  //				uint16_t cur = 0;
  //				TxData[0]=volt >> 8;
  //				TxData[1]=volt & ((1<<8) - 1);
  //				TxData[2]=cur >> 8;
  //				TxData[3]=cur & ((1<<8) - 1);
  //				TxData[4]=1;
  //				Charger_CANTransmit(0x1806E5F4,1,0,8,(uint8_t *) TxData);
  ////				HAL_Delay(400);
  //				HAL_GPIO_WritePin(BMS_Latch_GPIO_Port, BMS_Latch_Pin, RESET);
  //			}
  //		}
  //		else
  //		{
  //			TxData[4]=1;
  //			Charger_CANTransmit(0x1806E5F4,1,0,8,(uint8_t *) TxData);
  //		}
  //	}

  if (htim->Instance == TIM3)
  {
//    /Perform the charger related tasks/
    voltage_fd = Ch_fd[0] * 256 + Ch_fd[1];
    current_fd = Ch_fd[2] * 256 + Ch_fd[3];
    curr_act = current_fd / 10;

//    tsms = ADC_Read(ADC_CHANNEL_15);

    if(HAL_GPIO_ReadPin(tsms_GPIO_Port, tsms_Pin) == 1)
//    	tsms
//    if (tsms > 4000)
    {
    	tsms =1;
//      if (voltage_fd < 3984 && Volt_max < 4000)
    	if (voltage_fd < 4000 && Volt_max < 4000)
      {
//        uint16_t volt = 3984;
    	uint16_t volt = 4000;
        uint16_t cur = 110;
        crnt = cur;
        TxData[0] = volt >> 8;
        TxData[1] = volt & ((1 << 8) - 1);
        TxData[2] = cur >> 8;
        TxData[3] = cur & ((1 << 8) - 1);
        TxData[4] = 0;
        Charger_CANTransmit(0x1806E5F4, 1, 0, 8, (uint8_t *)TxData);

//        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
      }
//      else if (voltage_fd < 3984 && Volt_max >= 4000 && Volt_max < 4120)
    	else if (voltage_fd < 4000 && Volt_max >= 4000 && Volt_max < 4120)
      {
        if (crnt == 0)
          crnt = 55;

//        uint16_t volt = 3984;
        uint16_t volt = 4000;
        uint16_t cur = crnt;
        TxData[0] = volt >> 8;
        TxData[1] = volt & ((1 << 8) - 1);
        TxData[2] = cur >> 8;
        TxData[3] = cur & ((1 << 8) - 1);
        TxData[4] = 0;
        Charger_CANTransmit(0x1806E5F4, 1, 0, 8, (uint8_t *)TxData);

//        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
      }
//      else if (voltage_fd < 3984 && Volt_max >= 4120)
    	else if (voltage_fd < 4000 && Volt_max >= 4120)
      {
//        uint16_t volt = 3984;
    	uint16_t volt = 4000;
//        uint16_t cur = 22;
    	uint16_t cur = 55;
        crnt = cur;
        TxData[0] = volt >> 8;
        TxData[1] = volt & ((1 << 8) - 1);
        TxData[2] = cur >> 8;
        TxData[3] = cur & ((1 << 8) - 1);
        TxData[4] = 0;
        Charger_CANTransmit(0x1806E5F4, 1, 0, 8, (uint8_t *)TxData);

//        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
      }
//      if (voltage_fd < 3984 && Volt_max > 4200)
    	if (voltage_fd < 4000 && Volt_max > 4198)
      { //?
//        uint16_t volt = 3984;
        uint16_t volt = 4000;
        uint16_t cur = 0;
        crnt = cur;
        TxData[0] = volt >> 8;
        TxData[1] = volt & ((1 << 8) - 1);
        TxData[2] = cur >> 8;
        TxData[3] = cur & ((1 << 8) - 1);
        TxData[4] = 1;
        Charger_CANTransmit(0x1806E5F4, 1, 0, 8, (uint8_t *)TxData);
      }
//      if (voltage_fd > 3984)
    	if (voltage_fd > 4000)
      {
//        uint16_t volt = 3984;
    	uint16_t volt = 4000;
        uint16_t cur = 0;
        crnt = cur;
        TxData[0] = volt >> 8;
        TxData[1] = volt & ((1 << 8) - 1);
        TxData[2] = cur >> 8;
        TxData[3] = cur & ((1 << 8) - 1);
        TxData[4] = 1;
        Charger_CANTransmit(0x1806E5F4, 1, 0, 8, (uint8_t *)TxData);
        //				HAL_Delay(400);
        HAL_GPIO_WritePin(BMS_Latch_GPIO_Port, BMS_Latch_Pin, RESET);
      }
    }
    else
    {
      tsms = 0;
      TxData[4] = 1;
      Charger_CANTransmit(0x1806E5F4, 1, 0, 8, (uint8_t *)TxData);
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, RESET);
    }
  }

  if (htim->Instance == TIM4) // check error, measure current and operate BMS latch in each 400 ms
  {
    /*Write the part of code where we reset the slave can error flag*/
    if ((canHealthMonitoringArray[0] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[1] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[2] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[3] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[4] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[5] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[6] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[7] <= canHealthMonitoringThreshold) && (canHealthMonitoringArray[8] <= canHealthMonitoringThreshold))
    {
//      /This imlpies that all the indices in the array have indicated that the slave CAN is working correctly/
      slaveCanErrorFlag = RESET;
    }
    // HAL_GPIO_TogglePin(indication3_led_GPIO_Port, indication3_led_Pin);
    //  measure current
    CheckErrors();
    current_value = current_sensor();
    if (current_value >= 220.00 || current_value <= -50.00)
    {
//      Error_State = Overcurrent;
    }
    // error check
    // CheckErrors();
    // BMS_latch operation
    if ((Error_State == Overvoltage) | (Error_State == Undervoltage) | (Error_State == Overtemperature) | (Error_State == Overcurrent) | (slaveCanErrorFlag == SET))
    {
    	++err_flag_cntr;
    	if(err_flag_cntr > 40)

    	{
    		HAL_GPIO_WritePin(BMS_Latch_GPIO_Port, BMS_Latch_Pin, RESET); // error then RESET
      //				HAL_GPIO_TogglePin(comm_led_GPIO_Port, comm_led_Pin);
    	}
    }
    else
    {
      err_flag_cntr = 0;
      HAL_GPIO_WritePin(BMS_Latch_GPIO_Port, BMS_Latch_Pin, SET);
    }

    for (uint8_t i = 0; i < STACK_COUNT; i++)
    {
      for (uint8_t j = 0; j < VOLT_COUNT; j++)
      {
        Cell_Voltage[VOLT_COUNT * i + j] = (float)Volt_Reg[i][j] / 1000;
      }
      for (uint8_t k = 0; k < TEMP_COUNT; k++)
      {
        Cell_Temperature[TEMP_COUNT * i + k] = Temp_Reg[i][k];
      }
    }
  }
  if(htim == &htim10)
  {
	if(tsms==1)
		CAN1Transmit();
  }


}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  testVar1 = 10;
//  /Checking for CAN messages/
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, (uint8_t *)Volt_Temp);
  if (hcan->Instance == CAN1)
  {
//    /This implies that the charger CAN has responded/
    if (rxHeader.ExtId == 0x18FF50E5)
    {
      uint8_t *RxData1 = (uint8_t *)Volt_Temp;
      Ch_fd[0] = RxData1[0];
      Ch_fd[1] = RxData1[1];
      Ch_fd[2] = RxData1[2];
      Ch_fd[3] = RxData1[3];
      Ch_fd[4] = RxData1[4];
      Ch_fd[5] = RxData1[5];
      Ch_fd[6] = RxData1[6];
      Ch_fd[7] = RxData1[7];
    }
    if (rxHeader.StdId == 0x13)
    {
      RxData1[3] = Volt_Temp[3];
      RxData1[2] = Volt_Temp[2];
      RxData1[1] = Volt_Temp[1];
      RxData1[0] = Volt_Temp[0];
    }

    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
  }

  else if (hcan->Instance == CAN2)
  {
    /*This implies that the battery slave CAN has responded*/
    uint8_t slave_no = rxHeader.StdId / 100;
    uint8_t slave_data_id = rxHeader.StdId % 100;
//    /Resetting the value in the array at a given index to indicate that the particular slave has responded/
    canHealthMonitoringArray[slave_no - 1] = RESET;
    if (slave_data_id == 1)
    {
      //		HAL_GPIO_TogglePin(comm_led_GPIO_Port, comm_led_Pin);
      for (int i = 0; i < 4; i++)
      {
        Volt_Reg[slave_no - 1][i] = Volt_Temp[i];
      }

    }

    if (slave_data_id == 5)
    {
      //		HAL_GPIO_TogglePin(comm_led_GPIO_Port, comm_led_Pin);
      for (int i = 0; i < 4; i++)
      {
        Volt_Reg[slave_no - 1][i + 4] = Volt_Temp[i];
      }
    }

    if (slave_data_id == 9)
    {
      //		HAL_GPIO_TogglePin(comm_led_GPIO_Port, comm_led_Pin);
      for (int i = 0; i < 4; i++)
      {
        Volt_Reg[slave_no - 1][i + 8] = Volt_Temp[i];
      }
    }

    if (slave_data_id == 21)
    {
      // HAL_GPIO_TogglePin(comm_led_GPIO_Port, comm_led_Pin);
      for (int i = 0, j = 0; i < 5; i += 2, j++)
      {
        Temp_Reg[slave_no - 1][i] = Volt_Temp[j] >> 8;
        Temp_Reg[slave_no - 1][i + 1] = Volt_Temp[j] & 0xff;
      }
    }
  }
}

void CheckErrors(void)
{
  Volt_max = 0;
  Volt_min = 5000;
  volt_total = 0;
  Temp_max = 0;
  Temp_min = 100;

  Error_State = OK;

  for (int i = 0; i < STACK_COUNT; i++)
  {
    for (int j = 0; j < VOLT_COUNT; j++)
    {
      if (Volt_Reg[i][j] > Volt_max)
        Volt_max = Volt_Reg[i][j];

      if (Volt_Reg[i][j] < Volt_min)
        Volt_min = Volt_Reg[i][j];

      volt_total += Volt_Reg[i][j]; // Calculating overall voltage
    }
  }

  volt_total = volt_total / 1000.0; // converting milli-Volts to Volts

  for (int i = 0; i < STACK_COUNT; i++)
  {
    //		Temp_avg[i] = 0;

    for (int j = 0; j < TEMP_COUNT; j++)
    {
      if (Temp_Reg[i][j] > Temp_max)
        Temp_max = Temp_Reg[i][j];

      if (Temp_Reg[i][j] < Temp_min)
        Temp_min = Temp_Reg[i][j];

      //			Temp_avg[i] += Temp_Reg[i][j];
    }

    //		Temp_avg[i] = Temp_avg[i]/TEMP_COUNT;
  }

  if (Volt_max > 4200)
    Error_State = Overvoltage;

  else if (Volt_min < 3200)
    Error_State = Undervoltage;

  //	Prev_max_temp = Cur_max_temp;
  //	Cur_max_temp = Temp_max;
  //
  //	if(Temp_max > 59)
  //	{
  //		if (Cur_max_temp - Prev_max_temp <= 2 && Cur_max_temp - Prev_max_temp >= -2
  //				&& Temp_flag == 0)
  //		{
  //			Temp_flag++;
  //
  //			if(Temp_flag > 3)
  //				Error_State = Overtemperature;
  //		}
  //	}

  //	for(int k = 0; k < STACK_COUNT; ++k)
  //	{


//  if(Temp_max > 55 && Temp_max < 62)
//  {
//	  temp_flag++;
//
//  }
//  else
//	  temp_flag = 0;
  if (chg_mode == 0 && Temp_max > 205 && Temp_max < 215) // in case of connector removal
  	  Error_State = Overtemperature;

  else if (chg_mode == 0 && Temp_max < 54)
	  temp_flag = 0;
  else if (chg_mode == 0 && Temp_max > 54 && Temp_max < 59)
	  ++temp_flag;
  else if (chg_mode == 0 && Temp_max > 59)
  {
	  if(temp_flag > 10000)
		  Error_State = Overtemperature;
  }

  else if (chg_mode == 1 && Temp_max > 44) // in case of charging
	  Error_State = Overtemperature;
  //	}

  if (Temp_max > 216)
	  Error_State = OK;
}

// float current_sensor()
//{
//	HAL_ADC_Start(&hadc3);
//	HAL_ADC_PollForConversion(&hadc3, 10);
//	int16_t v_out = HAL_ADC_GetValue(&hadc3);
//	float x = ((v_out/4095.00) * 5.00) - 2.5;
//     float current = ((x/1.25)*400);
//	return current;
// }

float current_sensor()
{
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 14);
  // int16_t v_out = HAL_ADC_GetValue(&hadc3);
  v_out = ADC_Read(ADC_CHANNEL_8);

  x = (v_out / 4096.00) * 3.3;
  x1 = x*2 - 2.5;
  current = ((x1 / 1.25) * 200)+5;

  prev_current = curr_current;
  curr_current = current;

  if (current < 8 && current > -2)
    current = 0;

  if (curr_current - prev_current > 7)
    current = 0;

  return current;
}

void Charger_CANTransmit(uint32_t id, uint32_t IDE, uint32_t RTR, uint32_t DLC, uint8_t *data)
{
  uint32_t mailbox;
  CAN_TxHeaderTypeDef Header;
  Header.ExtId = id;
  Header.IDE = CAN_ID_EXT;
  Header.RTR = CAN_RTR_DATA;
  Header.DLC = DLC;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
  {
    HAL_CAN_AddTxMessage(&hcan1, &Header, data, &mailbox);
  }
}

uint16_t ADC_Read(uint32_t ADC_CH)
{
  uint32_t val;
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CH;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
  HAL_ADC_Start(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 1);
  val = HAL_ADC_GetValue(&hadc3);
  HAL_ADC_Stop(&hadc3);
  return val;
}
void CAN1Transmit(){
	for(int i=0;i<8;i++){
		for(int j=0;j<4;j++){
			Txdata1[j]=Volt_Reg[i][j];
		}
		eq_CAN_Tx(&hcan1,(i+1)*100,Txdata1,4);
		for(int j=4;j<8;j++){
			Txdata1[j-4]=Volt_Reg[i][j];
		}
		eq_CAN_Tx(&hcan1,(i+1)*100+1,Txdata1,4);
		for(int j=8;j<12;j++){
			Txdata1[j-8]=Volt_Reg[i][j];
		}
		eq_CAN_Tx(&hcan1,(i+1)*100+2,Txdata1,4);
	}
	for(int i=0;i<8;i++){
		for(int j=0;j<8;j++){
			Txdata2[j]=Temp_Reg[i][j];
		}
		eq_CAN_Tx(&hcan1,(i+1)*100+3,Txdata2,8);
	}
	while(1)
		{
		   if ((hcan1.Instance->TSR & CAN_TSR_TME0) || (hcan1.Instance->TSR & CAN_TSR_TME1) || (hcan1.Instance->TSR & CAN_TSR_TME2))  // checking empty CAN Tx mailboxes for can2 replace hcan1 with hcan2
		   {
			dq_CAN_Tx();
			if (queueHead == queueTail) {
					// Queue is empty, nothing to dequeue
					break; // once queue empty no need for infinite queue
		   }
		}
	}
}
void eq_CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint16_t* data,uint8_t len) {
    // Check if queue is full
    if ((queueHead + 1) % 10 == queueTail) {
        // Queue is full, discard new message
        return;
    }

    // Enqueue message
    canMessageQueue[queueHead].id = id;
    for (int i = 0; i < len; i++) {
        canMessageQueue[queueHead].data[i] = data[i];
    }
    canMessageQueue[queueHead].hcan = hcan;
    queueHead = (queueHead + 1) % 10;
}

void dq_CAN_Tx(void) {
    // Check if queue is empty
    if (queueHead == queueTail) {
        // Queue is empty, nothing to dequeue
        return;
    }

    // Transmit message at the head of the queue
    CanMessage* message = &canMessageQueue[queueTail];

    CAN_Tx(message -> hcan,message->id,message->data);
    // Update queue tail
    queueTail = (queueTail + 1) % 10;
}

void CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data_tx){
	    CAN_TxHeaderTypeDef TxHeader;
		uint32_t Txmailbox = 0x00U;
		TxHeader.DLC=8;
	    TxHeader.StdId=id; // 11 bits
	    TxHeader.IDE=CAN_ID_STD;
	    TxHeader.RTR=CAN_RTR_DATA;

		if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0){


			if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx, &Txmailbox)!= HAL_OK){
				Error_Handler();
			}
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

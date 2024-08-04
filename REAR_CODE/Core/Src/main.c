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
typedef struct {
	CAN_HandleTypeDef* hcan;
    uint32_t id;
    uint16_t data[4];
    uint8_t DLC;

} CanMessage;


CanMessage canMessageQueue[10]; //here you define the size of CAN message you want to send sequentially
uint8_t queueHead = 0;
uint8_t queueTail = 0;

void eq_CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint16_t* data,uint8_t DLC);
void dq_CAN_Tx();
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */
uint32_t ir_sens;
int m1;
float volt,temp_ir,damp_right,damp_left,flowrate,flowr,flowr1;
uint32_t pres,pres2,pres3,pres4, temp,temp2,temp3,temp4,stat;
uint16_t Tim1Prescaler,Period,Period2,Tim2Prescaler;
uint8_t RxData[8];
int c,a,b,d,x,count,count1;
int c=1;int b1=0;int c1=0;int tim_on=0;
uint32_t rcr_value,ccr_value;
uint16_t damp_sens[2];
uint16_t ptl[8],ptr[8];
uint16_t sens[4];
uint32_t ide;
uint16_t batt;
float Tdisplay,Pdisplay,Tdisplay2,Pdisplay2,Tdisplay3,Pdisplay3,Tdisplay4,Pdisplay4;
uint8_t data1[4],data2[4],data3[4],data4[4];
int f1,f2,f3;
CAN_RxHeaderTypeDef RxHeader;

uint32_t t1,t2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void irsens(void);
static void Tim1_cust(void);
static void Tim2_cust(void);
void lv_batt();
void dampers();
void func_time(int c);
void diffunc();
void CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data_tx);
void CAN_Start();
void flowrate1();
void flowrate2();
void pres_temp();
void CAN_transmit();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Txdata[8];
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
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM13_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

Tim2_cust();
Tim1_cust();
CAN_Start();
//  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
//                 Error_Handler();

   // start pwm generation
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim13);
  fanpwm();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  sConfig.Channel = ADC_CHANNEL_5;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
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
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 5;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
s
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 15999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 15999;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 9;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAN_LED_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_5_Pin|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|SPI_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : FR1_Pin FR2_Pin */
  GPIO_InitStruct.Pin = FR1_Pin|FR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_LED_Pin LED4_Pin */
  GPIO_InitStruct.Pin = CAN_LED_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_5_Pin PB14 PB15 */
  GPIO_InitStruct.Pin = LED_5_Pin|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 SPI_CSN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11|SPI_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==FR1_Pin){
		count++;
	}
	if(GPIO_Pin==FR2_Pin){
			count1++;
		}
}

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


static void Tim1_cust(void)
{
	/* set the Timer prescaler to get 1kHz as counter clock */
	 Tim1Prescaler= (uint16_t) (SystemCoreClock / 1000) - 1;
	/* configure the repetition counter */
      TIM1->RCR = ((uint32_t) 9) -1;
 	 rcr_value=TIM1->RCR;
 	/* Initialize the PWM period to get 150 ms as frequency from 1kHz */
	 Period = 110;
		/* configure pulse width */
		 TIM1->CCR1 = 100;
	/* configure the Timer prescaler */
	 TIM1->PSC = Tim1Prescaler;
	/* configure the period */
	 TIM1->ARR = Period-1;

	/* Select the Clock Division to 1*/
	/* Reset clock Division bit field */
	 TIM1->CR1 &= ~ TIM_CR1_CKD;
	/* Select DIV1 as clock division*/
	 TIM1->CR1 |= TIM_CLOCKDIVISION_DIV1;
	/* Select the Up-counting for TIM1 counter */
	/* Reset mode selection bit fields*/
	 TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	/* select Up-counting mode */
	 TIM1->CR1 |= TIM_COUNTERMODE_UP;
	/* SET PWM1 mode */
	/* Reset the Output Compare Mode Bits */
	 TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
	 TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;
	 /* Select the output compare mode 1*/
	  TIM1->CCMR1 |= TIM_OCMODE_PWM1;
	 /************** One pulse mode configuration ************/
	  /* One Pulse Mode selection */
	  TIM1->CR1 |= TIM_CR1_OPM;
	 /********************************************************/
	 /******* Slave mode configuration: Trigger mode *********/
	 /* Select the TIM_TS_ITR1 signal as Input trigger for the TIM */
	  TIM1->SMCR &= ~TIM_SMCR_TS;
	  TIM1->SMCR |= TIM_TS_ITR1;
	 /* Select the Slave Mode */
	  TIM1->SMCR &= ~TIM_SMCR_SMS;
	  TIM1->SMCR |= TIM_SLAVEMODE_TRIGGER;
	 /******************************************************************/
	 /* Enable the output compare 1 Preload */
	  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
	 /* Set the UG bit to enable UEV */
	  TIM1->EGR |= TIM_EGR_UG;
	 /* Enable the TIM1 Main Output */
	  TIM1->BDTR |= TIM_BDTR_MOE;
	 /* Select active low as output polarity level */
	 /* Reset the Output Polarity level */
	  TIM1->CCER &= ~TIM_CCER_CC1P;
	 /* Set the Low output */
	  TIM1->CCER |= TIM_OCPOLARITY_LOW;
	 /* Enable CC1 output on High level */
	  TIM1->CCER |= TIM_CCER_CC1E;
	 /* Enable the TIM Counter */
	  TIM1->CR1 |= TIM_CR1_CEN;



}

static void Tim2_cust(void)
{
	/* set the Timer prescaler to get 1kHz as counter clock */
	 Tim2Prescaler= (uint16_t) ((SystemCoreClock ) / 1000) - 1;
	/* Initialize the PWM period to get 1 sec as frequency from 1MHz */
	 Period2 = 1000;
	/* configure the period */
	 TIM2->ARR = Period2-1;
	/* configure the Timer prescaler */
	 TIM2->PSC = Tim2Prescaler;
	 /* Select the Clock Divison to 1*/
	 /* Reset clock Division bit field */
	  TIM2->CR1 &= ~ TIM_CR1_CKD;
	 /* Select DIV1 as clock division*/
	  TIM2->CR1 |= TIM_CLOCKDIVISION_DIV1;
	 /* Select the Up-counting for TIM1 counter */
	 /* Reset mode selection bit fields */
	  TIM2->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	 /* select Up-counting mode */
	  TIM2->CR1 |= TIM_COUNTERMODE_UP;
	 /****** Master mode configuration: Trigger update mode *******/
	 /* Trigger of TIM2 Update into TIM1 Slave */
	  TIM1->CR2 &= ~ TIM_CR2_MMS;
	  TIM2->CR2 |= TIM_TRGO_UPDATE;
	 /*************************************************************/
	 /* Enable the TIM Counter */
	  TIM2->CR1 |= TIM_CR1_CEN;

}


void func_time(int c)
{if(c==1)
   flowrate1();
 if(c==2)
  flowrate2();

 if(c>2)
 	 diffunc();
 }

void diffunc(void)
{
//	irsens();
	dampers();
	pres_temp();
	lv_batt();
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_SET);

}


void pres_temp(void)
{
	      HAL_I2C_Master_Receive(&hi2c1, (uint16_t) (0x36<<1)+1, data1, 4, 10);
		  HAL_I2C_Master_Receive(&hi2c1, (uint16_t) (0x28<<1)+1, data2, 4, 10);
	      HAL_I2C_Master_Receive(&hi2c2, (uint16_t) (0x36<<1)+1, data3, 4, 10);
		  HAL_I2C_Master_Receive(&hi2c2, (uint16_t) (0x28<<1)+1, data4, 4, 10);

		  pres=(data1[0]<<8) | data1[1];
		  temp = (data1[2]<<3) | (data1[3]>>5);

		  pres2=(data2[0]<<8) | data2[1];-
		  temp2 = (data2[2]<<3) | (data2[3]>>5);

		  pres3=(data3[0]<<8) | data3[1];
		  temp3 = (data3[2]<<3) | (data3[3]>>5);

		  pres4=(data4[0]<<8) | data4[1];
+ 		  temp4 = (data4[2]<<3) | (data4[3]>>5);

		  stat = data1[0]>>6;  // first two bits denote status of i2c data,refer datasheet
          // Right Rear
		  Tdisplay=((float)temp*200.0/2048.0)-50.0; //Mapping done according to data sheet
		  pres = (((1 << 14) - 1) & (pres >> (1 - 1))); // masked out the first two bits denoting status
		  Pdisplay=(((float)pres-1000.0)*7.0/14000.0); //Mapping done according to data sheet

		  // Right Front
		 Tdisplay2=((float)temp2*200.0/2048.0)-50.0; //Mapping done according to data sheet
		 pres2 = (((1 << 14) - 1) & (pres2 >> (1 - 1))); // masked out the first two bits denoting status
		 Pdisplay2=(((float)pres2-1000.0)*7.0/14000.0)-0.01; //Mapping done according to data sheet
        //Left Rear
		 Tdisplay3=((float)temp3*200.0/2048.0)-50.0; //Mapping done according to data sheet
     	  pres3 = (((1 << 14) - 1) & (pres3 >> (1 - 1))); // masked out the first two bits denoting status
		  Pdisplay3=(((float)pres3-1000.0)*7.0/14000.0); //Mapping done according to data sheet

		  // Left Front
		  Tdisplay4=((float)temp4*200.0/2048.0)-50.0; //Mapping done according to data sheet
 		  pres4 = (((1 << 14) - 1) & (pres4 >> (1 - 1))); // masked out the first two bits denoting status
 		  Pdisplay4=(((float)pres4-1000.0)*7.0/14000.0); //Mapping done according to data sheet

 		  ptl[0]=(uint16_t)(Tdisplay3*10);//Left Front
 		  ptl[1]=(uint16_t)(Pdisplay3*10);
 		  ptl[2]=(uint16_t)(Tdisplay4*10);//Left Rear
 		  ptl[3]=(uint16_t)(Pdisplay4*10);

 		  ptr[0]=(uint16_t)(Tdisplay2*10);// Right Front
 		  ptr[1]=(uint16_t)(Pdisplay2*10);
 		  ptr[2]=(uint16_t)(Tdisplay*10);//Right  Rear
 		  ptr[3]=(uint16_t)(Pdisplay*10);


}

void dampers(void)
{
	damp_sens[0] = ADC_Read(&hadc1,ADC_CHANNEL_12);
	damp_sens[1] = ADC_Read(&hadc1,ADC_CHANNEL_13);
	damp_right=damp_sens[0]*3.30/4096.0;
	damp_left=damp_sens[1]*3.30/4096.0;
    sens[0]=damp_sens[0];
    sens[1]=damp_sens[1];

}

void irsens(void)
{
	ir_sens=ADC_Read(&hadc1,ADC_CHANNEL_15);
	volt=(ir_sens)*3/(4096.0)+ 0.060;
	temp_ir=volt/3.0*450-70;
	sens[2]=ir_sens;


}

void flowrate1(void)
{
    flowr=count/5.50;
	count=0;

	sens[2]=(uint8_t) count;


}
void flowrate2(void)
{
    flowr1=count1/5.50;
	count1=0;

	sens[3]=(uint8_t) count1;



}

void lv_batt(void)
{
	batt=ADC_Read(&hadc2,ADC_CHANNEL_2);
	sens[2]=batt;

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)

{
	if(htim == &htim1)
	{   rcr_value=TIM1->CNT;
      if(c==10)
    	  c=1;
      func_time(c);
	  c++;}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim == &htim13 && tim_on==1 )
//		{
//			 CAN_transmit();
//		}
//}
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

		if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0){

			ide= TxHeader.StdId;
			if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx, &Txmailbox)!= HAL_OK){
				Error_Handler();
			}
		}


}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//	CAN_RxHeaderTypeDef RxHeader;

	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader,RxData);
	if(RxHeader.StdId==0x04 ){
		CAN_transmit();
		tim_on=1;
	}
}
void CAN_transmit(void)
{

	eq_CAN_Tx(&hcan1,0x025, ptl,4);
	eq_CAN_Tx(&hcan1,0x026, ptr,4);
	eq_CAN_Tx(&hcan1,0x027, sens,4);
	while(1)
	    {
	       if ((hcan1.Instance->TSR & CAN_TSR_TME0) || (hcan1.Instance->TSR & CAN_TSR_TME1) || (hcan1.Instance->TSR & CAN_TSR_TME2))  // checking empty CAN Tx mailboxes for can2 replace hcan1 with hcan2
	       {
	        dq_CAN_Tx();
	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
	        if (queueHead == queueTail) {
	                // Queue is empty, nothing to dequeue
	                break; // once queue empty no need for infinite queue
	       }
	    }
	}
	tim_on=0;
//	HAL_TIM_Base_Stop_IT(&htim13);
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

void dq_CAN_Tx(void)
{
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

void fanpwm(void)
{
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_2);
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

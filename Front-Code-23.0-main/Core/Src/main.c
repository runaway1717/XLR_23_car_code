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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
typedef struct {
	CAN_HandleTypeDef* hcan;
    uint32_t id;
    uint8_t data[8];
    uint8_t DLC;

} CanMessage;


CanMessage canMessageQueue[10]; //here you define the size of CAN message you want to send sequentially
uint8_t queueHead = 0;
uint8_t queueTail = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanOne);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcanTwo);
float map(uint16_t value, uint16_t min_val, uint16_t max_val, uint16_t offset);
void implausiblityCheck();
void softBSPD();
void CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data_tx);
void CAN_Start();
void CAN_Transmit_Messages();
void eq_CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data,uint8_t DLC);
void dq_CAN_Tx();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ide;
uint32_t temp=0;


uint8_t rtds_temp;
uint8_t tsms = 0;// hard code



uint8_t rtds = 0;
int CNT=0;
int CNT_impl=0;
uint8_t rtds_temp_var=0;
uint8_t clearfault_temp_var=0;


uint8_t clearfault_left=0;//hard code
uint8_t clearfault_right=0;
int c1=0;


uint8_t soft_bspd=0;

uint8_t implausiblity = 0;
uint8_t ip_temp_var=0;
uint8_t temp_var_soft_bspd=0;

uint16_t adc_apps1 = 0, adc_apps2 = 0;
uint16_t range_adc_apps2[] = {1405, 3870};
uint16_t range_adc_apps1[] = {430, 1900};
//uint16_t range_adc_apps2[] = {1405, 3870};
//uint16_t apps_avg;
float soft_bspd_threshold_apps=25.0;;
float app1, app2,apps_avg,torque_demand;

uint16_t adc_brk1=0, adc_brk2=0 , FrontBrakePressure=0 , RearBrakePressure=0;
uint16_t brk_avg;
uint16_t soft_bspd_threshold_brk=1650;

uint16_t adc_steer_sens=0;
uint16_t adc_ir_sens=0;
uint16_t adc_damper_left=0;
uint16_t adc_damper_right=0;

float torque_commanded,torque_commanded_left,torque_commanded_right;
uint16_t Txdata_imp1[4],Txdata_imp2[4];
uint8_t Txdata_state[8];
uint8_t Txdata_inv_rst[8];
short Txdata_control[4];
uint8_t inv_transmit[8];

uint8_t tc=0;
uint8_t tv=0;
uint16_t torque_limit=40;
uint8_t left_inv_transmit[8];
uint8_t right_inv_transmit[8];

uint8_t a=0,b=0,a1=0;
float test=-69.1;

float kp;
float ki;
float steering_degree,steering_rad;
float op=0.0, oi=0;
float o=0.0;
float xvel, yvel;
float compfactor;
float wref=0,zgyro;
float error=0,error_i=0;
long ta=0;
long tb=0;
float k=-0.001; // understeering coeff
float k2=0.0;
float wref2=0.0;

int ishan=0;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

 // commenting it to remove can
  CAN_Start();
  HAL_TIM_Base_Start_IT(&htim5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//xvel=4;
	adc_apps1 = ADC_Read(&hadc1, ADC_CHANNEL_8);
	adc_apps2 = ADC_Read(&hadc1, ADC_CHANNEL_9);
	adc_brk1 = ADC_Read(&hadc1,ADC_CHANNEL_7);
	adc_brk2 = ADC_Read(&hadc1,ADC_CHANNEL_6);//front




	adc_damper_left = ADC_Read(&hadc3, ADC_CHANNEL_9);// left damper
	adc_damper_right = ADC_Read(&hadc3, ADC_CHANNEL_14);//right damper
	adc_steer_sens = ADC_Read(&hadc1,ADC_CHANNEL_5);
	adc_ir_sens= ADC_Read(&hadc3,ADC_CHANNEL_15);


	app1 = map(adc_apps1, range_adc_apps1[0], range_adc_apps1[1],100);
	app2 = map(adc_apps2, range_adc_apps2[0], range_adc_apps2[1],100);
	apps_avg = (app1+app2)/2;
	torque_demand=(apps_avg/100)*torque_limit;
	brk_avg = (adc_brk1+adc_brk2)/2;
	FrontBrakePressure = 1 + 0.0225*(adc_brk2 - 372.27);
	RearBrakePressure = 1 + 0.0225*(adc_brk1 - 372.27);
	softBSPD();
	implausiblityCheck();
	steering_degree= ((adc_steer_sens-1888.0)/1144.0)*34.0; // to be transmitted
//	steering_rad= ((adc_steer_sens-2055.0)/1144.0)*34.0*(3.1415/180.0);

//	k2= (xvel*steering_rad-1.6*zgyro)/(1.6*xvel*xvel)-0.006;
	wref2= ((xvel*steering_degree*(0.01745))/(1.6+k2*xvel*xvel));


	//compfactor=1.1197-(0.1202/xvel);
    //compfactor=1;
   // if(compfactor<=1)
	//{
//		compfactor=1;
//	}
//	wref = ((xvel*steering_rad)/(1.6+k*xvel*xvel))*compfactor; // 1.6 is the wheelbase
	tb=ta;
	ta=HAL_GetTick();
	error= wref2- zgyro;
	if (xvel<2){
		xvel=2;
	}
//	//xvel should be transmitted before this
	kp= (556.085-(21.544*xvel))*0.005; // 5.14 at xvel=2
	ki=(1521.88+(75047.39/xvel));   //
	op=kp*error;
	error_i = error_i + error*(ta-tb)/1000;
	oi =ki*error_i ;
	if (zgyro>-0.015 && zgyro<0.015){
		o=0.0;
	}
////	if (oi>7){
////		oi=7;
////	}
////	else if(oi<-7){
////		oi=-7;
////	}
	o =op;
//	//o=5.0;
	if (o>10){
		o=10;
	}
	else if(o<-10) {
		o=-10;
	}
	if (xvel<=6){
		o=0;
	}




//	comment out testing
	if(HAL_GPIO_ReadPin(TSMS_GPIO_Port, TSMS_Pin)==GPIO_PIN_SET){
			tsms=1;
		}else{
			tsms=0;
		}
	if(rtds_temp==0){
		if(HAL_GPIO_ReadPin(GPIOB, RTDS_INPUT_Pin)==GPIO_PIN_SET && (tsms==1) && (clearfault_left==1) && (clearfault_right==1)&&(rtds==0)&&(adc_brk1>500)&&(adc_brk2>500)){
			c1++;
			if(CNT==0){
				HAL_TIM_Base_Start_IT(&htim6);
			}else if(CNT!=0){
				HAL_TIM_Base_Start_IT(&htim6);
				HAL_GPIO_TogglePin(GPIOB, BUZZ_OUT_Pin);
			}
			rtds_temp=1;
		}
	}
//	if(tv==0){
//		if(HAL_GPIO_ReadPin(TV_Enable_GPIO_Port, TV_Enable_Pin)==GPIO_PIN_SET ){
//			tv=1;
//		}
//	}
//	else if(tv==1){
//		if(HAL_GPIO_ReadPin(TV_Enable_GPIO_Port, TV_Enable_Pin)==GPIO_PIN_SET ){
//			tv=0;
//		}
//	}

	if(tv==1){
		HAL_GPIO_WritePin(GPIOG, TV_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG, TC_LED_Pin, GPIO_PIN_SET);

	}else{
		HAL_GPIO_WritePin(GPIOG, TV_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, TC_LED_Pin, GPIO_PIN_RESET);
	}


	if(tsms==0){
		rtds=0;
		rtds_temp=0;
		clearfault_left=0;
		clearfault_right=0;
	}
//	rtds=1;
	Txdata_imp1[0]=(uint16_t)apps_avg;
	Txdata_imp1[1]=FrontBrakePressure;
	Txdata_imp1[2]=RearBrakePressure;
	Txdata_imp1[3]=(short)(steering_degree*10);

	Txdata_imp2[0]=adc_damper_right;
	Txdata_imp2[1]=adc_damper_left;
	Txdata_imp2[3]=adc_ir_sens;

	Txdata_state[0]=soft_bspd;
	Txdata_state[1]=implausiblity;
	Txdata_state[2]=tsms;
	Txdata_state[3]=rtds;
	Txdata_state[4]=tc;
	Txdata_state[5]=tv;

	Txdata_control[0]=(short)(op*100);
	Txdata_control[1]=(short)(oi*100);
	Txdata_control[2]=(short)(error*100);
	Txdata_control[3]=(short)(error_i*100);
//	Txdata_control[1]=oi;

	if(tv==0){
		torque_commanded_left=torque_commanded;
		torque_commanded_right=torque_commanded;
	}else if(tv==1){
		torque_commanded_left=torque_commanded+o*0.5;
		torque_commanded_right=torque_commanded-o*0.5;
	}
	if (torque_commanded_left>(float)torque_limit){
		torque_commanded_left=(float)torque_limit;
	}
	else if (torque_commanded_left<0.00){
		torque_commanded_left=0.00;
	}
	if (torque_commanded_right>(float)torque_limit){
		torque_commanded_right=(float)torque_limit;
	}
	else if (torque_commanded_right<0.00){
		torque_commanded_right=0.00;
	}
	a++;
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
  sConfig.Channel = ADC_CHANNEL_9;
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
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
																			// Using for CANTx transmission frequency, Delay of 30msec
  /* USER CODE END TIM5_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 15999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 49;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
																// Using for RTDS, Delay of 2sec
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 40000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
																// Using for implausibility function, Delay of 100msec
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 39;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 40000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CAN_LED_Pin|LED_Pin|LEDF8_Pin|LEDF9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIO1_Pin|CLK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DIO2_Pin|CLK2_Pin|TC_LED_Pin|TV_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, INV_RST_LED_Pin|BUZZ_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_LED_Pin LED_Pin LEDF8_Pin LEDF9_Pin */
  GPIO_InitStruct.Pin = CAN_LED_Pin|LED_Pin|LEDF8_Pin|LEDF9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin CLK1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|CLK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO2_Pin CLK2_Pin TC_LED_Pin TV_LED_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin|CLK2_Pin|TC_LED_Pin|TV_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : TV_EXTI_Pin TV_BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = TV_EXTI_Pin|TV_BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : INV_RST_EXTI3_Pin */
  GPIO_InitStruct.Pin = INV_RST_EXTI3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(INV_RST_EXTI3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INV_RST_LED_Pin */
  GPIO_InitStruct.Pin = INV_RST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INV_RST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RTDS_INPUT_Pin TSMS_Pin */
  GPIO_InitStruct.Pin = RTDS_INPUT_Pin|TSMS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZ_OUT_Pin */
  GPIO_InitStruct.Pin = BUZZ_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_OUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

		if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0){
			HAL_GPIO_TogglePin(GPIOF, CAN_LED_Pin);
			if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx, &Txmailbox)!= HAL_OK){
				Error_Handler();
			}
		}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//	CAN_RxHeaderTypeDef RxHeader;
//	uint8_t RxData[8];
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader,RxData);
	if(RxHeader.StdId==0x139){
		//HAL_GPIO_TogglePin(GPIOF, LEDF9_Pin);
		xvel=((float)(((uint16_t)RxData[1]<<8)|RxData[0]))*0.01;
	}
	if(RxHeader.StdId==0x122){
		zgyro=(float)(((uint16_t)RxData[5]<<8)|RxData[4])*0.001;
	}
}
float map(uint16_t value, uint16_t min_val, uint16_t max_val, uint16_t offset){
	float temp = (((float)(value - (min_val+offset))) / ((float)((max_val-offset) - (min_val+offset)))) * 100.00;
		if (temp < 0.00){
			return 0.00;
		}
		else if (temp > 100.00){
			return 100.00;
		}
		else {
			return temp;
		}
	}

void implausiblityCheck(){
	if(app1-app2>10 || app2-app1>10){
		if(CNT_impl==0){
			HAL_TIM_Base_Start_IT(&htim7);
		}else if(CNT_impl!=1){
			HAL_TIM_Base_Start_IT(&htim7);
		}
	}
}

void softBSPD(){
	if(adc_brk1>soft_bspd_threshold_brk||adc_brk2>soft_bspd_threshold_brk){
		if(apps_avg>soft_bspd_threshold_apps){
			temp_var_soft_bspd=1;
		}
	}
	if(temp_var_soft_bspd==0){
		torque_commanded=torque_demand;
	}else if(temp_var_soft_bspd==1){
		torque_commanded=0;
		soft_bspd=1;
		HAL_GPIO_TogglePin(GPIOF, LEDF9_Pin);
	}
	if((temp_var_soft_bspd==1) && (apps_avg<5)){
		torque_commanded=torque_demand;
		soft_bspd=0;
		temp_var_soft_bspd=0;
		HAL_GPIO_TogglePin(GPIOF, LEDF9_Pin);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == TV_BUTTON_EXTI13_Pin){
		if(HAL_GPIO_ReadPin(GPIOG, TV_BUTTON_EXTI13_Pin )&& tsms==1){
			clearfault_temp_var=1;
		}
	}
//	if(GPIO_Pin==RTDS_EXTI5_Pin){
//		c1++;
//		if(HAL_GPIO_ReadPin(RTDS_EXTI5_GPIO_Port, RTDS_EXTI5_Pin)&& (tsms==1) && (clearfault_left==1) && (clearfault_right==1)&&(rtds==0)&&(adc_brk1>500)&&(adc_brk2>500)){
//			if(CNT==0){
//				HAL_TIM_Base_Start_IT(&htim6);
//			}else if(CNT!=0){
//				HAL_TIM_Base_Start_IT(&htim6);
//				HAL_GPIO_TogglePin(GPIOB, BUZZ_OUT_Pin);
//			}
//		}
//	}
//	if(GPIO_Pin==TV_BUTTON_EXTI13_Pin){
//		if(HAL_GPIO_ReadPin(GPIOG, TV_BUTTON_EXTI13_Pin)){
//			tv=1;
//		}
//	}
	if(GPIO_Pin==TV_EXTI_Pin){
		if(HAL_GPIO_ReadPin(GPIOG, TV_EXTI_Pin)){
			if(tv==0){
				tv=1;
			}else if(tv==1){
				tv=0;
			}
			c1++;
//			HAL_GPIO_TogglePin(INV_RST_LED_GPIO_Port, INV_RST_LED_Pin);
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim6){
		if(CNT==0){
			if(rtds_temp_var==0){
				HAL_GPIO_TogglePin(GPIOB, BUZZ_OUT_Pin);
				rtds_temp_var=1;
			}else if(rtds_temp_var==1){
				HAL_GPIO_TogglePin(GPIOB, BUZZ_OUT_Pin);
				HAL_TIM_Base_Stop_IT(&htim6);
				rtds=1;
				rtds_temp_var=0;
			}
			CNT=1;
		}else if(CNT!=0){
			HAL_GPIO_TogglePin(GPIOB, BUZZ_OUT_Pin);
			HAL_TIM_Base_Stop_IT(&htim6);
			rtds=1;
		}
	}
	if(htim==&htim7){
		if(CNT_impl==0){
			if(ip_temp_var==0){
				ip_temp_var=1;
			}else if(ip_temp_var==1){
				HAL_TIM_Base_Stop_IT(&htim7);
				if(app1-app2>10 || app2-app1>10){
					implausiblity=1;
					rtds=0;
				}
			}
			CNT_impl=1;
		}else if(CNT!=0){
			HAL_TIM_Base_Stop_IT(&htim7);
			if(app1-app2>10 || app2-app1>10){
				implausiblity=1;
				rtds=0;
			}
		}
	}
	if(htim==&htim5){
		CAN_Transmit_Messages();
	}
}
void CAN_Transmit_Messages(){

	if(clearfault_temp_var==1){
			Txdata_inv_rst[0]=20;
			Txdata_inv_rst[1]=0;
			Txdata_inv_rst[2]=1;
			Txdata_inv_rst[3]=0;
			Txdata_inv_rst[4]=0;
			Txdata_inv_rst[5]=0;
//			CAN_Tx(&hcan1,0x0C1,Txdata_inv_rst);
//			CAN_Tx(&hcan1,0x0F1,Txdata_inv_rst);
			eq_CAN_Tx(&hcan1,0x0C1,Txdata_inv_rst,6);
			eq_CAN_Tx(&hcan1,0x0F1,Txdata_inv_rst,6);
			clearfault_temp_var=0;
			clearfault_left=1;
			clearfault_right=1;
	}

	if(tsms==1&&rtds==0&&clearfault_left==1&&clearfault_right==1){
//	if(tsms==1&&clearfault_left==1&&clearfault_right==1&&ishan!=3){
			left_inv_transmit[0]=0;
			left_inv_transmit[1]=0;
			left_inv_transmit[2]=0;
			left_inv_transmit[3]=0;
			left_inv_transmit[4]=1;
			left_inv_transmit[5]=0;
			left_inv_transmit[6]=((torque_limit)*10)%256;
			left_inv_transmit[7]=((torque_limit)*10)/256;

			right_inv_transmit[0]=0;
			right_inv_transmit[1]=0;
			right_inv_transmit[2]=0;
			right_inv_transmit[3]=0;
			right_inv_transmit[4]=0;
			right_inv_transmit[5]=0;
			right_inv_transmit[6]=((torque_limit)*10)%256;
			right_inv_transmit[7]=((torque_limit)*10)/256;
//			CAN_Tx(&hcan1,0x0F0,left_inv_transmit);
			eq_CAN_Tx(&hcan1,0x0F0,left_inv_transmit,8);
			a++;

//			CAN_Tx(&hcan1,0x0C0,right_inv_transmit);
			eq_CAN_Tx(&hcan1,0x0C0,right_inv_transmit,8);
			b++;
	}else if(tsms==1&&rtds==1){
			left_inv_transmit[0]=(int)((torque_commanded_left)*10)%256;
			left_inv_transmit[1]=(int)((torque_commanded_left)*10)/256;;
			left_inv_transmit[2]=0;
			left_inv_transmit[3]=0;
			left_inv_transmit[4]=1;
			left_inv_transmit[5]=1;
			left_inv_transmit[6]=((torque_limit)*10)%256;
			left_inv_transmit[7]=((torque_limit)*10)/256;

			right_inv_transmit[0]=(int)((torque_commanded_right)*10)%256;;
			right_inv_transmit[1]=(int)((torque_commanded_right)*10)/256;;
			right_inv_transmit[2]=0;
			right_inv_transmit[3]=0;
			right_inv_transmit[4]=0;
			right_inv_transmit[5]=1;
			right_inv_transmit[6]=((torque_limit)*10)%256;
			right_inv_transmit[7]=((torque_limit)*10)/256;
//			CAN_Tx(&hcan1,0x0F0,left_inv_transmit);
			eq_CAN_Tx(&hcan1,0x0F0,left_inv_transmit,8);
//			CAN_Tx(&hcan1,0x0C0,right_inv_transmit);
			eq_CAN_Tx(&hcan1,0x0C0,right_inv_transmit,8);
	}
	eq_CAN_Tx(&hcan1,0x1,Txdata_imp1,8);
	eq_CAN_Tx(&hcan1,0x2,Txdata_imp2,6);
	eq_CAN_Tx(&hcan1,0x3,Txdata_state,8);
	eq_CAN_Tx(&hcan1,0x4,Txdata_control,8);
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
void eq_CAN_Tx(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data,uint8_t len) {
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

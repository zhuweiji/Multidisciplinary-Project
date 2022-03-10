/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include "stdbool.h"
#include "pid.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ShowTask */
osThreadId_t ShowTaskHandle;
const osThreadAttr_t ShowTask_attributes = {
  .name = "ShowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void motors(void *argument);
void encoder_task(void *argument);
void show(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[4];
uint8_t indexer = 0; // to store array index and determine number of incoming chars
uint8_t data_rx = 0; // to receive incoming uart char
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_UART_Receive_IT(&huart3,(uint8_t *) &data_rx, 1);
//  HAL_UART_Receive_DMA (&huart3, UART1_rxBuffer, 12);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motors, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of ShowTask */
  ShowTaskHandle = osThreadNew(show, NULL, &ShowTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 320;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
bool receivingUART = false;
bool commandReady = false;
uint8_t count = 0;
uint8_t txCount = 0;
uint32_t testVal = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	UNUSED(huart);
  count += 1;
	uint8_t endChar = '\r';
	if (data_rx == endChar && ! receivingUART){
		data_rx = 0;
		indexer = 0;
		memset(aRxBuffer,0, 4);
		receivingUART = true;
	} else if (receivingUART){
		if (data_rx != endChar){
			aRxBuffer[indexer] = data_rx;
			indexer++;
		} else {
			receivingUART = false;
			commandReady = true;
			data_rx = 0;
		}
	}
	/**
	 * re receiving signal
	 */
	HAL_UART_Receive_IT(&huart3,(uint8_t *) &data_rx, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
  txCount +=1 ;
}

// motors function

float arrTofloat(uint8_t arr[4], int s, int e){
	float res =0;
	for (int j=0;j<=e-s;j++){
//		myInt1 = bytes[0] + (bytes[1] << 8) + (bytes[2] << 16) + (bytes[3] << 24);
	 	res += pow(10,j)*((int) arr[e-j]-48);
	}
	return res;
}

void stop_rear_wheels()
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
}

void set_wheel_direction(bool isForward){
	if (isForward){
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	}
}

bool inLab = false;
uint32_t motorAPwm = 5840; // 5820
uint32_t motorBPwm = 5510; // 5510;
uint32_t motorAPwmLow = 1000;
uint32_t motorBPwmLow;
uint32_t servoRight = 135; // measure tan 1.6/3 - set to 140 => motor will not run -> think 135 is max
uint32_t servoLeft = 53; // old 44 // measure tan: 1.7/3
uint32_t servoMid = 73;
float DegConstLeft;
float DegConstRight;

/** measure distance **/
bool isMeasureDis = false;
bool overFlow = false;
uint32_t diffA = 0;
uint32_t diffB = 0;
uint32_t cnt1A = 0;
uint32_t cnt1B = 0;
uint32_t cnt2A = 0;
uint32_t cnt2B = 0;
int diffSpeed = 0;

bool isDownA = false;
bool isDownB = false;

uint32_t findDiffTime (bool isCountDown, uint32_t cnt1, uint32_t cnt2){
	uint32_t diff;
	if(isCountDown){
		if(cnt2 <= cnt1){
			overFlow = false;
			diff = cnt1 - cnt2;
		} else {
			overFlow = true;
			diff = (65535 - cnt2) + cnt1;
		}
	}
	else{
		if(cnt2 >= cnt1){
			overFlow = false;
			diff = cnt2 - cnt1;
		} else {
			overFlow = true;
			diff = (65535 - cnt1) + cnt2;
		}
	}
	return diff;
}

float measure (int cnt1_A, int cnt1_B){
	float disA, disB;
	int cnt2_A, cnt2_B, diff_A, diff_B;

	isDownA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
	isDownB = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	cnt2_A = __HAL_TIM_GET_COUNTER(&htim2);
	cnt2_B = __HAL_TIM_GET_COUNTER(&htim3);

	cnt2A = cnt2_A;
	cnt2B = cnt2_B;
	diff_A = findDiffTime(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2), cnt1_A, __HAL_TIM_GET_COUNTER(&htim2));
	diff_B = findDiffTime(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3), cnt1_B, __HAL_TIM_GET_COUNTER(&htim3));

	diffA = diff_A;
	diffB = diff_B;
	disA = diff_A*M_PI/(330*4)*5.65; // real is 6.5
	disB = diff_B*M_PI/(330*4)*5.65; // real is 6.5
  return (disA + disB)/2;
}

float measureDiffVelo (int cnt1_A, int cnt1_B, int oldTick){
	float veloA, veloB;
	int cnt2_A, cnt2_B, diff_A, diff_B;
	isDownA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
	isDownB = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	cnt2_A = __HAL_TIM_GET_COUNTER(&htim2);
	cnt2_B = __HAL_TIM_GET_COUNTER(&htim3);

	diff_A = findDiffTime(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2), cnt1_A, __HAL_TIM_GET_COUNTER(&htim2));
	diff_B = findDiffTime(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3), cnt1_B, __HAL_TIM_GET_COUNTER(&htim3));

	int tick = HAL_GetTick() - oldTick;
	veloA = diff_A*M_PI/(330*4)*5.65/tick*1000; // real is 6.5
	veloB = diff_B*M_PI/(330*4)*5.65/tick*1000; // real is 6.5
  return veloB-veloA;
}

/* distance should be in cm*/
void move_straight(bool isForward, float distance)
{
//	uint32_t distance_ticks = ((distance/100)/(0.065*3.1416)) / (speedConstant/330*36) *60 * 1000;
	//(1*1.24/(0.065*3.1416)) / (1180/330*36) *60 * 1000
	set_wheel_direction(isForward);
//	uint8_t hello_A[20];
	isMeasureDis = true; // trigger measure distance
	float measuredDis = 0;
	float diffVelo = 0; // veloB-veloA;
	int cnt1_A, cnt1_B, cnt1_velo_A, cnt1_velo_B;
	int oldTick;
//	htim1.Instance->CCR4 = 66;
	HAL_Delay(200);
	htim1.Instance->CCR4 = 74; // see how
	HAL_Delay(200);
	uint32_t usePwmA, usePwmB;
//	if (distance > 50){
////		usePwmA = motorAPwm;
////		usePwmB = motorBPwm;
//		usePwmA = motorAPwmLow;
//		usePwmB = motorBPwmLow;
//	} else {
//		usePwmA = motorAPwmLow;
//		usePwmB = motorBPwmLow;
//	}

//	usePwmA = 3000;  // forward outside lab, If we need to change this, we should keep these values for turning. If we change this again, turning is affected.
//	usePwmB = 3565;  // forward outside lab, If we need to change this, we should keep these values for turning. If we change this again, turning is affected.
//
//	if (!isForward)
//	{
//		usePwmA = 3000;
//		usePwmB = 4150;
//	}

	if (!inLab) {
		usePwmA = 2500;
		usePwmB = 2100;

		if (!isForward) {
			usePwmA = 2100;
			usePwmB = 2480; //2650
		}
	} else {
		usePwmA = 1000;
		usePwmB = 1000; //1250

		if (!isForward) {
			usePwmA = 1000;
			usePwmB = 1325; //1275;
		}
	}

//	LED_SHOW = 1;
	HAL_Delay(200); // delay to the encoder work properly when change direction
	htim1.Instance->CCR4 = servoMid;
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, usePwmA);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);

	cnt1_A = __HAL_TIM_GET_COUNTER(&htim2);
  cnt1_B = __HAL_TIM_GET_COUNTER(&htim3);
  cnt1_velo_A = cnt1_A;
  cnt1_velo_B = cnt1_B;
  oldTick = HAL_GetTick();
	do {
		HAL_Delay(5);
		measuredDis = measure(cnt1_A, cnt1_B);
//		if (isForward){
//			diffVelo = measureDiffVelo(cnt1_velo_A, cnt1_velo_B, oldTick);
////			uint32_t addValue = (uint32_t) (fabs(diffVelo/0.2));
////			if (addValue == 0){
////				addValue += 1;
////			}
//			if (diffVelo >= 0.05){
//				usePwmB -= 30;
//				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
////			} else if (diffVelo <= -0.05) {
////				usePwmB += 135;
//////				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
////			}
//	//		testVal = addValue;
//	//		diffSpeed = diffVelo;
//		}

//		if (!isForward){
//			diffVelo = measureDiffVelo(cnt1_velo_A, cnt1_velo_B, oldTick);
////			uint32_t addValue = (uint32_t) (fabs(diffVelo/0.2));
////			if (addValue == 0){
////				addValue += 1;
////			}
//			if (diffVelo >= 0.05){
//				usePwmB -= 30;
//				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
//			} else if (diffVelo <= -0.05) {
//				usePwmB += 555;
//				__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
//			}
//	//		testVal = addValue;
////			diffSpeed = diffVelo;
////		}
////		 debug
//	//	testVal = addValue;
//		diffSpeed = diffVelo;

//		 update for speed
//		cnt1_velo_A = __HAL_TIM_GET_COUNTER(&htim2);
//	  cnt1_velo_B = __HAL_TIM_GET_COUNTER(&htim3);
//		oldTick = HAL_GetTick();
	} while (distance > measuredDis);
//	LED_SHOW = 0;
	isMeasureDis = false; // stop measure
	stop_rear_wheels();
}

float measureDiffCount (int cnt1, bool isRight, int oldTick){
 float veloA, veloB;
 int cnt2, diff;
 bool isDown;
 if (isRight){
  isDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
  cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
 } else {
  isDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
  cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
 }
 diff = findDiffTime(isDown, cnt1, cnt2);
 int tick = HAL_GetTick() - oldTick;
 return (float)diff/(float)tick*1000; // unit it count/s
}

void move_straight_PID_2_Wheels(bool isForward, float distance){
 set_wheel_direction(isForward);
// htim1.Instance->CCR4 = 80;
 HAL_Delay(100); // delay to the encoder work properly when change direction
 htim1.Instance->CCR4 = servoMid;

 double offset = 0;
 double error = 0;
 double offsetR = 0;
 double errorR = 0;
 double initError;
 double desiredCountPerSecond = 4000;
 // from copy source - they set it to 4000 - 4000 fw and 3000 -3000  bw
 uint16_t initPwm = 3000;
 uint16_t initPwmA = 3000;
 uint16_t initPwmB = 3000;
 if (! isForward){
  initPwmA = 3000;
  initPwmB = 3000;
 }
 uint16_t minPwm = 2400;
 uint16_t maxPwm = 3600;

 uint16_t pwmValA = initPwmA;
 uint16_t pwmValB = initPwmB;
 uint32_t tick, period, count;

 long leftcount = 0;
 long rightcount = 0;
 long initLeftCount = 0;
 long initRightCount = 0;
 long countInDistanceL = 0;
 long countInDistanceR = 0;
 float currentcount = 0;

 float countsPerRev = 1320; //cntA value per wheel revolution (1320)
 float wheelDiam = 6.05; // init 6.43 - copy
 float wheelCirc = M_PI * wheelDiam;

 float numRev = distance/wheelCirc;
 float targetcount = numRev * countsPerRev;

///////////////////PID CONFIGURATION///////////////////////////////////////////////////////
 double Kp, Ki, Kd, KpL, KiL, KdL;
 if (isForward){
  Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
  Ki = 1;
  Kd = 0.09; // 0.025 look ok but damp quite slow
  KpL = 0.75;
  KiL = 5;
  KdL = 0.09;
 } else {
  Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
  Ki = 1;
  Kd = 0.09; // 0.025 look ok but damp quite slow
  KpL = 0.75;
  KiL = 5;
  KdL = 0.09;
 }
 if (! inLab){
	 if (isForward){
	   Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
	   Ki = 1;
	   Kd = 0.025; // 0.025 look ok but damp quite slow
	   KpL = 0.75;
	   KiL = 9;
	   KdL = 0.025;
	  } else {
	   Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
	   Ki = 1;
	   Kd = 0.025; // 0.025 look ok but damp quite slow
	   KpL = 0.75;
	   KiL = 45;
	   KdL = 0.05;
	  }
 }
 PID_TypeDef pidControl, pidControlR;


 /// can tune Kp, Ki, Kd to the comment value - think it is the source value - not get why set set point to 0
 // cur 2 wheel: 0.15 0.75 0.02 - not work 1.2, 1, 0.405 // 9:32pm 0, 30, 0.001

 // straight forward work: 0.05, 10, 0.01 - but due to luck
 PID(&pidControl, &error, &offset, 0, KpL, KiL, KdL, _PID_P_ON_E, _PID_CD_DIRECT);//150,0,1.4, and 8,0.01,1
 PID_SetMode(&pidControl, _PID_MODE_AUTOMATIC);
 PID_SetSampleTime(&pidControl, 10);
 PID_SetOutputLimits(&pidControl, (int)2400 - initPwmB, (int) 3600 - initPwmB); //600

 PID2(&pidControlR, &errorR, &offsetR, 0, Kp, Ki, Kd, _PID_CD_DIRECT);//150,0,1.4, and 8,0.01,1
 PID_SetMode(&pidControlR, _PID_MODE_AUTOMATIC);
 PID_SetSampleTime(&pidControlR, 10);
 PID_SetOutputLimits(&pidControlR, (int)2400 - initPwmA, (int) 3600 - initPwmA); //600

 rightcount = __HAL_TIM_GET_COUNTER(&htim2);
 leftcount = __HAL_TIM_GET_COUNTER(&htim3);
 initLeftCount = leftcount;
 initRightCount = rightcount;
 isMeasureDis = true;
 currentcount = (leftcount+rightcount)/2;

 tick = HAL_GetTick();

 period = tick;
 count = 0;

 bool modLeft, modRight;
 modLeft = true;
 modRight = true;
 do {
   HAL_Delay(20);

    errorR = measureDiffCount(rightcount, true, tick) - desiredCountPerSecond;
    error = measureDiffCount(leftcount, false, tick) - desiredCountPerSecond;
    tick = HAL_GetTick();
    PID_Compute(&pidControl);
    if (modRight){
     diffA = errorR;
     PID_Compute(&pidControlR);
     pwmValA = initPwmA + offsetR;
     cnt1A = pwmValA;
    }

    if (modLeft){
     diffB = error;
     PID_Compute(&pidControl);
     pwmValB = initPwmB + offset;
     cnt1B = pwmValB;
    }

    if (modRight ){
     __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
    }
    if (modLeft){
     __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
    }

    rightcount = __HAL_TIM_GET_COUNTER(&htim2);
    leftcount = __HAL_TIM_GET_COUNTER(&htim3);

    countInDistanceL = findDiffTime(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3), initLeftCount,  leftcount);
    countInDistanceR = findDiffTime(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2), initRightCount, rightcount);

    currentcount = (
      countInDistanceL + countInDistanceR
    )/2;
    testVal = currentcount;

  } while ( currentcount < targetcount);
 stop_rear_wheels();
 isMeasureDis = false;
}

void move_straight_PID(bool isForward, float distance){
	set_wheel_direction(isForward);
	HAL_Delay(200); // delay to the encoder work properly when change direction
	htim1.Instance->CCR4 = servoMid;

	double offset = 0;
	double error = 0;
	// from copy source - they set it to 4000 - 4000 fw and 3000 -3000  bw
	uint16_t pwmValA = 3000;
	uint16_t pwmValB = 3000;

	long leftcount = 0;
	long rightcount = 0;
	long currentcount = 0;

	float countsPerRev = 1320; //cntA value per wheel revolution (1320)
	float wheelDiam = 6.1;
	float wheelCirc = M_PI * wheelDiam;

	float numRev = distance/wheelCirc;
	float targetcount = numRev * countsPerRev;

///////////////////PID CONFIGURATION///////////////////////////////////////////////////////
	PID_TypeDef pidControlDiff;

	/// can tune Kp, Ki, Kd to the comment value - think it is the source value - not get why set set point to 0
	PID(&pidControlDiff, &error, &offset, 0, 7, 10, 0.05, _PID_P_ON_E, _PID_CD_DIRECT);//150,0,1.4, and 8,0.01,1
	PID_SetMode(&pidControlDiff, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&pidControlDiff, 10);
	PID_SetOutputLimits(&pidControlDiff, -400, 400); //600
//////////////////////////////////////////////////////////////////

	//reset counter values
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);

	 if(isForward){ //if forward use leftcount
		 currentcount = leftcount;
	 }
	 else { //if backward use rightcount
		 currentcount = rightcount;
	 }

	do {
			HAL_Delay(30);
			leftcount = __HAL_TIM_GET_COUNTER(&htim2);
			rightcount = __HAL_TIM_GET_COUNTER(&htim3);

//////////////////////////PID PART//////////////////////////////////////////////////
			/** I think the rightcount = 65535 - right count may not necessary **/
			if(isForward){
				 if(rightcount != 0){
					 rightcount = 65535 - rightcount;
				 }
				 error = leftcount - rightcount;
			}
			else{
				 if(leftcount != 0){
					 leftcount = 65535 - leftcount;
				 }
				 error = leftcount - rightcount;
			}

			 //pid computation
			 PID_Compute(&pidControlDiff);
//////////////////////////////////////////////////////////////////////////////

			 //update pwmValue
			 pwmValA = pwmValA + offset;
			 pwmValB = pwmValB - offset;
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);

			 if(isForward){ //if forward use leftcount
				 currentcount = leftcount;
			 }
			 else{ //if backward use rightcount
				 currentcount = rightcount;
			 }
		} while ( currentcount < targetcount);
	stop_rear_wheels();
}

/* distance should be in cm ALT FUNCTION FOR THREE POINT*/
void move_straight_three_point(bool isForward, float distance)
{
//	uint32_t distance_ticks = ((distance/100)/(0.065*3.1416)) / (speedConstant/330*36) *60 * 1000;
	//(1*1.24/(0.065*3.1416)) / (1180/330*36) *60 * 1000
	set_wheel_direction(isForward);
//	uint8_t hello_A[20];
	isMeasureDis = true; // trigger measure distance
	float measuredDis = 0;
	float diffVelo = 0; // veloB-veloA;
	int cnt1_A, cnt1_B, cnt1_velo_A, cnt1_velo_B;
	int oldTick;
	htim1.Instance->CCR4 = 74; // see how
	uint32_t usePwmA, usePwmB;
//	if (distance > 50){
////		usePwmA = motorAPwm;
////		usePwmB = motorBPwm;
//		usePwmA = motorAPwmLow;
//		usePwmB = motorBPwmLow;
//	} else {
//		usePwmA = motorAPwmLow;
//		usePwmB = motorBPwmLow;
//	}

	if (inLab) {
		usePwmA = 1150;
		usePwmB = 1000;
	} else {
		usePwmA = 1150;
		usePwmB = 1000;
	}

	HAL_Delay(500); // used to be 200; delay to the encoder work properly when change direction
	htim1.Instance->CCR4 = servoMid;
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, usePwmA);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);

	cnt1_A = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_B = __HAL_TIM_GET_COUNTER(&htim3);
	cnt1_velo_A = cnt1_A;
	cnt1_velo_B = cnt1_B;

  oldTick = HAL_GetTick();
	do {
		HAL_Delay(5);
		measuredDis = measure(cnt1_A, cnt1_B);
	} while (distance > measuredDis);
//	LED_SHOW = 0;
	isMeasureDis = false; // stop measure
	stop_rear_wheels();
}

void move_straight_10(){
	htim1.Instance->CCR4 = servoMid;
	osDelay(500);
	uint32_t pwmVal = 5505;
	uint32_t pwmVal_2 = 5820;

	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal_2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);
	osDelay(125); // set this to change the distance
	// osDelay(120) is 10cm
	stop_rear_wheels();
	osDelay(500);
}

void turn_left()
{
	htim1.Instance->CCR4 = servoMid;
	osDelay(500);
	htim1.Instance->CCR4 = servoLeft;
	uint32_t pwmVal = 0;
	uint32_t pwmVal_2 = 6000;

	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal_2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);
	osDelay(630); // set this to change the angle of turn
	stop_rear_wheels();
//  htim1.Instance->CCR4 = servoMid;
//	osDelay(500);
}

void turn_right()
{
	htim1.Instance->CCR4 = servoMid;
	osDelay(500);
	htim1.Instance->CCR4 = servoRight;
	uint32_t pwmVal = 6000;
	uint32_t pwmVal_2 = 0;

	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal_2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);
	osDelay(660); // set this to change the angle of turn
	stop_rear_wheels();
//	htim1.Instance->CCR4 = servoMid;
//	osDelay(500);
}

/**
 * For turning
 */
const float tan_of_wheel_deg_right = 1.75/4.1; // 4.1 work outside lab
const float tan_of_wheel_deg_left = 1.75/4; // 31.06 deg 0.8/1.3 0.8/1.6 1.75/3
float calTurnDis (int deg, bool isRight){
	float tan;
	if (isRight){
		tan = tan_of_wheel_deg_right;
	} else {
		tan = tan_of_wheel_deg_left;
	}
	float turnRadius = 14.5 / tan;
	return turnRadius * deg * M_PI/180;
}

int testDeg = 0;
void turn_deg( int deg, bool isRight, bool isForward){
	testDeg = 0;
	float degMul;
	if (isRight){
		degMul = DegConstRight;
	} else {
		degMul = DegConstLeft;
	}
	set_wheel_direction(isForward);
	uint32_t servo;

	float measuredDis = 0;
	float diffVelo = 0; // veloB-veloA; - cm/s
	uint32_t usePwmB = motorBPwmLow;
	int cnt1_A, cnt1_B, cnt1_velo_A, cnt1_velo_B;
	int oldTick;

	if (isRight){
		servo = servoRight;
	} else {
		servo = servoLeft;
	}
	HAL_Delay(200);
	htim1.Instance->CCR4 = servo;
	HAL_Delay(200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, motorAPwmLow); //1000
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, usePwmB); //1020

	float distance = calTurnDis(deg * degMul, isRight);
	cnt1_A = __HAL_TIM_GET_COUNTER(&htim2);
	cnt1_B = __HAL_TIM_GET_COUNTER(&htim3);
	cnt1A = cnt1_A;
	cnt1B = cnt1_B;

	// for speed control
	cnt1_velo_A = cnt1_A;
	cnt1_velo_B = cnt1_B;
	oldTick = HAL_GetTick();
	do {
		HAL_Delay(5);
		//diffVelo = measureDiffVelo(cnt1_velo_A, cnt1_velo_B, oldTick);
		measuredDis = measure(cnt1_A, cnt1_B);
//		if (diffVelo >= 0.2 || diffVelo <= -0.2){
//			usePwmB += (int)  ceil(diffVelo/2);
//			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
//		}
//		uint32_t addValue = (uint32_t) (fabs(diffVelo/0.2));
//		if (addValue == 0){
//			addValue += 1;
//		}
//		if (diffVelo >= 0.05){
//			usePwmB -= addValue;
//			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
//		} else if (diffVelo <= -0.05) {
//			usePwmB += addValue;
//			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
//		}
//		testVal = addValue;

		// update for speed
//		cnt1_velo_A = __HAL_TIM_GET_COUNTER(&htim2);
//		cnt1_velo_B = __HAL_TIM_GET_COUNTER(&htim3);
//		oldTick = HAL_GetTick();
		} while (distance > measuredDis);
	stop_rear_wheels();
  htim1.Instance->CCR4 = servoMid;
}


/**
 * 3 point turn
 */
const float R = 63.5/2;
float d = R/2;
void three_points_turn_90deg(bool isRight){

	float deg = 30;

	if (!isRight)
	{
		if (inLab){
			deg = 30 * 1.20; //lab floor 1.16
		} else {
			deg = 30 * 1.098; //1.198, 1.075
		}
	} else {
		if (inLab){
			deg = 30 * 0.96; //lab floor
		} else {
			deg = 30 * 0.9; //1.235
		}
	}

	move_straight_three_point(false, 2);
	turn_deg(deg, isRight, true);
	move_straight_three_point(false, d);
	turn_deg(deg, isRight, true);
	move_straight_three_point(false, d);
	turn_deg(deg, isRight, true);
	move_straight_three_point(false, d);
	move_straight_three_point(false, 2);
}
// end motors function
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	uint8_t ch = 'A';
	  for(;;)
	  {
//		HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
//		if(ch<'Z')
//			ch++;
//		else ch = 'A';
	    osDelay(5000);
	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motors */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
// this var should be del later;
bool haveSendSignalBack = false;
/* USER CODE END Header_motors */
void motors(void *argument)
{
  /* USER CODE BEGIN motors */
	//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	//	for(;;) {
	//		htim1.Instance->CCR4 = 110; //82
	//		osDelay(2000);
	//		htim1.Instance->CCR4 = 74; //72
	//		osDelay(2000);
	//		htim1.Instance->CCR4 = 44; //62
	//		osDelay(2000);
	//		htim1.Instance->CCR4 = 74; //62
	//		osDelay(2000);
	//	}
	uint16_t pwmVal;
	uint16_t pwmVal_2;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	/**
	 * note: if set encoder in different channel, may cause in first for loop, encoder not wake up yet and result in move straight no work
	 */

	/**
	 * init the variable base on lab condition
	 */
	if (inLab) {
//		motorBPwmLow = 1030;
		motorBPwmLow = 1020;
		DegConstLeft = 0.97;
		DegConstRight = 0.97;
	} else {
//		motorBPwmLow = 1030;
		motorBPwmLow = 1000; //2300
		motorAPwmLow = 1000;
		DegConstLeft = 0.97;
		DegConstRight = 1;
	}

	char direction = 'p'; // 'l' for left, 'r' for right, 'f' for forward
	uint8_t completeChar = 'C';
	uint8_t defaultChar = 'z';
	uint8_t endChar = '\r';

	bool isMoved = false;
	float straightDistance = 0;
	int deg = 0;
	bool doCommand = false;

	osDelay(100);
//	aRxBuffer[0] = 'L';
	bool haveTest = false;
	for(;;)
	  {
			if (commandReady && haveTest){
				haveSendSignalBack = false;
				doCommand = true;
				switch (aRxBuffer[0]){
					case 'F':
						straightDistance = arrTofloat(aRxBuffer,1,indexer-1);
//						if (straightDistance < (float) 50){
//							int moveTime = (int) straightDistance/10;
//							for (int i=0; i < moveTime; i ++){
//								move_straight_10();
//							}
//						} else {
//							move_straight(true, straightDistance);
//						}
						move_straight(true, straightDistance);
						straightDistance = 0;
						break;
					case 'B':
						straightDistance = arrTofloat(aRxBuffer,1,indexer-1);
						move_straight(false, straightDistance);
						straightDistance = 0;
						break;
					case 'L':
						if (indexer-1 > 1){
							deg = (int) arrTofloat(aRxBuffer,1,indexer-1);
							turn_deg(deg, false, true);
							deg = 0;
						} else {
							three_points_turn_90deg(false);
						}
						break;
					case 'R':
						if (indexer-1 > 1){
							deg = (int) arrTofloat(aRxBuffer,1,indexer-1);
							turn_deg(deg, true, true);
							deg =0;
						} else {
							three_points_turn_90deg(true);
						}
						break;
					case 'A':
						// left reverse
						if (indexer-1 > 1){
							deg = (int) arrTofloat(aRxBuffer,1,indexer-1);
							turn_deg(deg, true, false);
							deg =0;
						}
						break;
					case 'D':
						// right reverse
						if (indexer-1 > 1){
							deg = (int) arrTofloat(aRxBuffer,1,indexer-1);
							turn_deg(deg, false, false);
							deg =0;
						}
						break;
					default:
						doCommand = false;
						break;
				}
				if (doCommand){
					if (HAL_UART_Transmit_IT(&huart3, (uint8_t *) &completeChar, 1) == HAL_OK){
						haveSendSignalBack = true;
					};
				}
				commandReady = false;
			}
/* for test */
			if (! haveTest){
//				move_straight(true, 200);

//				move_straight_three_point(true,5);
//				move_straight_three_point(false,5);
//				three_points_turn_90deg(true);
//				turn_deg(90 * DegConstRight, true, true);
				three_points_turn_90deg(false);

//				move_straight_PID_2_Wheels(false, 50);
//				move_straight(false, 100);

				haveTest = true;
			}
	  	osDelay(1000);
	  }
  /* USER CODE END motors */
}

/* USER CODE BEGIN Header_encoder_task */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_task */
void encoder_task(void *argument)
{
  /* USER CODE BEGIN encoder_task */
  /* Infinite loop */
  for (;;){
  	osDelay(10000);
  }
  /* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  uint8_t hello[20] = "Hello World!\0";
  /* Infinite loop */
  for(;;)
  {
//			sprintf(hello, "Cmd ready: %d", commandReady);
//			OLED_ShowString(10, 20, hello);

//			sprintf(hello, "Rec UART: %d", receivingUART);
//			OLED_ShowString(10, 30, hello);
//
//
//			sprintf(hello, "Count cbtx: %d", txCount);
//			OLED_ShowString(10, 40, hello);
//
//			sprintf(hello, "Send back: %d", haveSendSignalBack);
//			OLED_ShowString(10, 50, hello);

			sprintf(hello, "dis: %d", testVal);
			OLED_ShowString(10, 10, hello);

			/**debug**/

//  				sprintf(hello, "Deg: %d", testDeg);
//  				OLED_ShowString(10, 20, hello);

//				sprintf(hello, "isDown: %d %d", isDownA, isDownB);
//				OLED_ShowString(10, 10, hello);
			sprintf(hello, "DiffA: %d", diffA);
			OLED_ShowString(10, 20, hello);

			sprintf(hello, "DiffB: %d", diffB);
			OLED_ShowString(10, 30, hello);
//
			sprintf(hello, "Diffspeed: %d", cnt1A);
			OLED_ShowString(10, 40, hello);
//
			sprintf(hello, "Measuring: %d", cnt1B);
			OLED_ShowString(10, 50, hello);
			OLED_Refresh_Gram();
    osDelay(100);
  }
  /* USER CODE END show */
}

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


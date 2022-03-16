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
#include "ICM20948.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
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
/* Definitions for MoveObs */
osThreadId_t MoveObsHandle;
const osThreadAttr_t MoveObs_attributes = {
  .name = "MoveObs",
  .stack_size = 512 * 4,
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

// gyro value for PID
float targetAngle = 0;
float curAngle = 0;
uint8_t readGyroZData[2];
int16_t gyroZ;
float angleNow = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void motors(void *argument);
void move_obs_task(void *argument);
void show(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[4];
uint8_t indexer = 0; // to store array index and determine number of incoming chars
uint8_t data_rx = 0; // to receive incoming uart char
float targetAngle;
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_2000DPS);

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

  /* creation of MoveObs */
  MoveObsHandle = osThreadNew(move_obs_task, NULL, &MoveObs_attributes);

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
  sConfig.Channel = ADC_CHANNEL_11;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : US_TRIG_Pin */
  GPIO_InitStruct.Pin = US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_TRIG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
bool receivingUART = false;
bool commandReady = false;
uint8_t count = 0;
uint8_t txCount = 0;
int testVal = 0;

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

uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint8_t Is_First_Captured = 0;
float obsDist_US = 0;
uint8_t icCBCount = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	icCBCount += 1;
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1 (TRI: TIM4_CH2)
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			obsDist_US = (IC_Val2 > IC_Val1 ? (IC_Val2 - IC_Val1) : (65535 - IC_Val1 + IC_Val2)) * 0.034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
		}
	}
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

void move_straight_PID(bool isForward, float distance){
 set_wheel_direction(isForward);
 htim1.Instance->CCR4 = 90;
 HAL_Delay(500); // delay to the encoder work properly when change direction
 htim1.Instance->CCR4 = servoMid;
 HAL_Delay(500);

 double offset = 0; // output Pid
 double input = 0; // input Pid
 // from copy source - they set it to 4000 - 4000 fw and 3000 -3000  bw
 uint16_t initPwmA = 3000;
 uint16_t initPwmB = 3000;
 if (! isForward){
  initPwmA = 3000;
  initPwmB = 3000;
 }

 uint16_t pwmValA = initPwmA;
 uint16_t pwmValB = initPwmB;

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
 int dirCoef = 1;
 double Kp, Ki, Kd, KpL, KiL, KdL;
 if (isForward){
//  Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
//  Ki = 1;
//  Kd = 0.09; // 0.025 look ok but damp quite slow
  Kp = 0; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
    Ki = 0;
    Kd = 0;
 } else {
  Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
  Ki = 1;
  Kd = 0.09; // 0.025 look ok but damp quite slow
  dirCoef = -1;
 }
 if (! inLab){
	 Kp = 1.5;
	 Ki = 0;
	 Kd = 0;
 }
 PID_TypeDef pidControl, pidControlR;


 /// can tune Kp, Ki, Kd to the comment value - think it is the source value - not get why set set point to 0
 // cur 2 wheel: 0.15 0.75 0.02 - not work 1.2, 1, 0.405 // 9:32pm 0, 30, 0.001

 // straight forward work: 0.05, 10, 0.01 - but due to luck
// PID(&pidControl, &input, &offset, 0, Kp, Ki, Kd, _PID_P_ON_E, _PID_CD_DIRECT);//150,0,1.4, and 8,0.01,1
// PID_SetMode(&pidControl, _PID_MODE_AUTOMATIC);
// PID_SetSampleTime(&pidControl, 10);
// PID_SetOutputLimits(&pidControl, (int)2300 - initPwmB, (int) 3700 - initPwmB); //600


 rightcount = __HAL_TIM_GET_COUNTER(&htim2);
 leftcount = __HAL_TIM_GET_COUNTER(&htim3);
 initLeftCount = leftcount;
 initRightCount = rightcount;
 isMeasureDis = true;
 count = 0;

 bool modLeft, modRight;
 modLeft = true;
 modRight = true;

 // reset angle
 gyroZ = 0;
 curAngle = 0;
 do {
    HAL_Delay(10);
   	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
   	// since self test always sê gyroZ from -20 to 0 in the stable state
//   	curAngle += ((gyroZ >= -20 && gyroZ <= 10) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
//   	curAngle += ((gyroZ >= -35 && gyroZ <= 10) ? 0 : gyroZ); //inside lab
// negative moves it to the left, positive moves it to the right
   	curAngle += ((gyroZ >= -35 && gyroZ <= 25) ? 0 : gyroZ); //outside lab
   	input = -curAngle;

    PID_Compute(&pidControl);

    PID(&pidControl, &input, &offset, 0, Kp, Ki, Kd, _PID_P_ON_E, _PID_CD_DIRECT);//150,0,1.4, and 8,0.01,1
    PID_SetMode(&pidControl, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&pidControl, 10);
    PID_SetOutputLimits(&pidControl, (int)2300 - initPwmB, (int) 3700 - initPwmB); //600


    if (modRight){
     pwmValA = initPwmA - offset * dirCoef;
     cnt1A = pwmValA;
    }

    if (modLeft){
     pwmValB = initPwmB + offset * dirCoef;
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

  } while ( currentcount < targetcount);

 stop_rear_wheels();
 isMeasureDis = false;
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

void turn_left(float * targetAngle)
{
	htim1.Instance->CCR4 = servoMid;
	osDelay(500);
	htim1.Instance->CCR4 = servoLeft;
	uint32_t pwmVal = 0;
	uint32_t pwmVal_2 = 3000;

	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);

	set_wheel_direction(true);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal_2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);

	angleNow = 0;
	gyroZ = 0;
	int last_curTask_tick = HAL_GetTick();

	bool isAngle = false;


	while (!isAngle){
		if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
			__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
			angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

			if (angleNow >= 83) {
				htim1.Instance->CCR4 = servoMid;
	//			stop_rear_wheels();
				break;
			}
		last_curTask_tick = HAL_GetTick();
		}
	}

// ORIGINAL
//	do {
//	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
//		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
////		  angleNow += GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
////		  angleNow += ((gyroZ >= -1 && gyroZ <= 1) ? 0 : gyroZ); //outside lab
//		   	angleNow += ((gyroZ >= -35 && gyroZ <= 25) ? 0 : gyroZ); //outside lab
//
//		  if (abs(angleNow - *targetAngle) < 0.01) break;
//		  last_curTask_tick = HAL_GetTick();
//	  }
//	} while(1);

//	osDelay(630); // set this to change the angle of turn
//	stop_rear_wheels();

//  htim1.Instance->CCR4 = servoMid;
//	osDelay(500);
}

void turn_right(float * targetAngle)
{
	uint8_t hello[20] = "";
	sprintf(hello, "start", angleNow);
	OLED_ShowString(10, 10, hello);
	htim1.Instance->CCR4 = servoMid;
	HAL_Delay(500);
	//osDelay(500);
	htim1.Instance->CCR4 = servoRight;
	uint32_t pwmVal = 3000;
	uint32_t pwmVal_2 = 0;
	sprintf(hello, "here1", angleNow);
	OLED_ShowString(10, 10, hello);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	sprintf(hello, "fckmdp", angleNow);
	OLED_ShowString(10, 10, hello);
	set_wheel_direction(true);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmVal_2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmVal);
	sprintf(hello, "fckkg", angleNow);
	OLED_ShowString(10, 10, hello);
	angleNow = 0;
	gyroZ = 0;
	int last_curTask_tick = HAL_GetTick();

	bool isAngle = false;
	sprintf(hello, "before", angleNow);
	OLED_ShowString(10, 10, hello);
	while (!isAngle){
		if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
			__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
			angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

//		  angleNow += ((gyroZ >= -1 && gyroZ <= 1) ? 0 : gyroZ); //outside lab
//  	angleNow += ((gyroZ >= -35 && gyroZ <= 25) ? 0 : gyroZ); //outside lab

			if (angleNow <= -85) {
				htim1.Instance->CCR4 = servoMid;
				//stop_rear_wheels();
				break;
			}
		last_curTask_tick = HAL_GetTick();
		}
	}
	sprintf(hello, "finish", angleNow);
	OLED_ShowString(10, 10, hello);
//	do {
//	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
//		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
////		  angleNow += GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
//		  angleNow += ((gyroZ >= -1 && gyroZ <= 1) ? 0 : gyroZ); //outside lab
//		  if ((*targetAngle - angleNow) < 0.01) break;
//		  last_curTask_tick = HAL_GetTick();
//	  }
//	} while(1);
//	osDelay(660); // set this to change the angle of turn
//	stop_rear_wheels();

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

	float angleNow = 0; gyroZ = 0;
	int last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
		  angleNow += GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
//		  angleNow += ((gyroZ >= -35 && gyroZ <= 25) ? 0 : gyroZ); //outside lab
		  if (abs(angleNow - deg) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(1);

//	__SET_MOTOR_DUTY(&htim8, 0, 0);
//	__RESET_SERVO_TURN(&htim1);
//	do {
//		HAL_Delay(5);
//		//diffVelo = measureDiffVelo(cnt1_velo_A, cnt1_velo_B, oldTick);
//		measuredDis = measure(cnt1_A, cnt1_B);
////		if (diffVelo >= 0.2 || diffVelo <= -0.2){
////			usePwmB += (int)  ceil(diffVelo/2);
////			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
////		}
////		uint32_t addValue = (uint32_t) (fabs(diffVelo/0.2));
////		if (addValue == 0){
////			addValue += 1;
////		}
////		if (diffVelo >= 0.05){
////			usePwmB -= addValue;
////			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
////		} else if (diffVelo <= -0.05) {
////			usePwmB += addValue;
////			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, usePwmB);
////		}
////		testVal = addValue;
//
//		// update for speed
////		cnt1_velo_A = __HAL_TIM_GET_COUNTER(&htim2);
////		cnt1_velo_B = __HAL_TIM_GET_COUNTER(&htim3);
////		oldTick = HAL_GetTick();
//		} while (distance > measuredDis);
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

////////////////////////ADC IR //////////////////////////////////////
uint16_t obsTick_IR = 0;
float obsDist_IR = 0;
float debugObsDist_IR = 0;
float GLOBAL_IR_COUNTER = 0;

void readIR(int irNum){
	uint8_t hello[20] = "";
	sprintf(hello, "IR counter %f", GLOBAL_IR_COUNTER);
	OLED_ShowString(10, 60, hello);

	GLOBAL_IR_COUNTER ++;

	uint16_t dataPoint = 0; uint32_t IR_data_raw_acc = 0;
	isMeasureDis = true;
	if (irNum == 1){
		__ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		HAL_ADC_Stop(&hadc1);
	} else if (irNum == 2) {
		__ADC_Read_Dist(&hadc2, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		HAL_ADC_Stop(&hadc2);
	}
}

void move_straight_PID_IR(bool isForward, long* distanceCount){
 set_wheel_direction(isForward);
 htim1.Instance->CCR4 = 90;
 HAL_Delay(500); // delay to the encoder work properly when change direction
 htim1.Instance->CCR4 = servoMid;
 HAL_Delay(500);

 double offset = 0; // output Pid
 double input = 0; // input Pid
 // from copy source - they set it to 4000 - 4000 fw and 3000 -3000  bw
 uint16_t initPwm = 3000;

 uint16_t pwmValA = initPwm;
 uint16_t pwmValB = initPwm;

 long leftcount = 0;
 long rightcount = 0;
 long initLeftCount = 0;
 long initRightCount = 0;
 long countInDistanceL = 0;
 long countInDistanceR = 0;
 float currentcount = 0;

///////////////////PID CONFIGURATION///////////////////////////////////////////////////////
 int dirCoef = 1;
 double Kp, Ki, Kd, KpL, KiL, KdL;
 if (isForward){
  Kp = 0; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
	Ki = 0;
	Kd = 0;
 } else {
  Kp = 0.5; // look ok 0.01 0 0 but still depends on the battery - work with 3100 and 2900 //second 0.01 0.5 0
  Ki = 1;
  Kd = 0.09; // 0.025 look ok but damp quite slow
  dirCoef = -1;
 }
 if (! inLab){
	 Kp = 1.5;
	 Ki = 0;
	 Kd = 0;
 }
 PID_TypeDef pidControl, pidControlR;


 rightcount = __HAL_TIM_GET_COUNTER(&htim2);
 leftcount = __HAL_TIM_GET_COUNTER(&htim3);
 initLeftCount = leftcount;
 initRightCount = rightcount;
 isMeasureDis = true;
 count = 0;

 bool modLeft, modRight;
 modLeft = true;
 modRight = true;

 // reset angle
 gyroZ = 0;
 curAngle = 0;

 readIR(1);
 float initIRDis = obsDist_IR;
 debugObsDist_IR = obsDist_IR;
 HAL_Delay(100);
 isMeasureDis = true;
 do {
    HAL_Delay(10);
    readIR(1);
   	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
   	// since self test always sê gyroZ from -20 to 0 in the stable state
//   	curAngle += ((gyroZ >= -20 && gyroZ <= 10) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
//   	curAngle += ((gyroZ >= -35 && gyroZ <= 10) ? 0 : gyroZ); //inside lab
// negative moves it to the left, positive moves it to the right
   	curAngle += ((gyroZ >= -35 && gyroZ <= 25) ? 0 : gyroZ); //outside lab
   	input = -curAngle;

    PID_Compute(&pidControl);

    PID(&pidControl, &input, &offset, 0, Kp, Ki, Kd, _PID_P_ON_E, _PID_CD_DIRECT);//150,0,1.4, and 8,0.01,1
    PID_SetMode(&pidControl, _PID_MODE_AUTOMATIC);
    PID_SetSampleTime(&pidControl, 10);
    PID_SetOutputLimits(&pidControl, (int)2300 - initPwm, (int) 3700 - initPwm); //600


    if (modRight){
     pwmValA = initPwm - offset * dirCoef;
     cnt1A = pwmValA;
    }

    if (modLeft){
     pwmValB = initPwm + offset * dirCoef;
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

    *distanceCount = (
      countInDistanceL + countInDistanceR
    )/2;

  } while (fabs(obsDist_IR - initIRDis) <= 100);
 stop_rear_wheels();
 isMeasureDis = false;
}

////////////////////////ADC IR END //////////////////////////////////////

//// MOVE OBSTACLE /////

void robot_move_dis_obs() {
//	curAngle = 0; gyroZ = 0;
//	obsDist_US = 0;
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	__delay_us(&htim4, 1000); // wait for 10us
	HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);


//	osDelay(200); // give timer interrupt chance to update obsDist_US value
//	float dis = obsDist_US - 30;
//	targetAngle = 90;
//
//	move_straight_PID(true, dis);

	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
}

void week_9_v1() {

	uint8_t hello[20] = "";
	sprintf(hello, "MDP SUGOII", 0);
	OLED_ShowString(10, 20, hello);
//
	HAL_Delay(2000);

//	readIR(1);
//	HAL_Delay(2000);
////	sprintf(hello, "IR1 %f", fabs(obsDist_IR));
////	OLED_ShowString(10, 20, hello);
//	readIR(1);
//	HAL_Delay(2000);
//
//	readIR(1);
//	HAL_Delay(2000);
//
//	readIR(1);
//	HAL_Delay(2000);
//
//	readIR(1);
//	HAL_Delay(2000);
//
//	readIR(1);
//	HAL_Delay(2000);

	readIR(1);
	while (obsDist_IR > 15)
	{
		readIR(1);

		sprintf(hello, "IR1 %f", fabs(obsDist_IR));
		OLED_ShowString(10, 20, hello);
		OLED_Refresh_Gram();

		move_straight_PID(true, 10);

		obsDist_IR = 0;
		readIR(1);
	}



//	 move_straight_PID(true, 150);
//	 HAL_Delay(100);
//
//	 targetAngle = 90;
//	 turn_right(&targetAngle);
//	 stop_rear_wheels();
//	 HAL_Delay(100);
//
//	 move_straight_PID(true, 75);
//	 HAL_Delay(100);
//
//	 targetAngle = 90;
//	 turn_left(&targetAngle);
//	 stop_rear_wheels();
//	 HAL_Delay(100);
//
//	 move_straight_PID(true, 10);
//	 HAL_Delay(100);
//
//	 targetAngle = 90;
//	 turn_left(&targetAngle);
//	 stop_rear_wheels();
//	 HAL_Delay(100);
//
//	 move_straight_PID(true, 135);
//	 HAL_Delay(100);
//
//	 targetAngle = 90;
//	 turn_left(&targetAngle);
//	 stop_rear_wheels();
//	 HAL_Delay(100);
//
//	 move_straight_PID(true, 10);
//	 HAL_Delay(100);
//
//	 targetAngle = 90;
//	 turn_left(&targetAngle);
//	 stop_rear_wheels();
//	 HAL_Delay(100);
//
//	 move_straight_PID(true, 30);
//	 HAL_Delay(100);
//
//	 targetAngle = 90;
//	 turn_right(&targetAngle);
//	 stop_rear_wheels();
//	 HAL_Delay(100);
//
//	 move_straight_PID(true, 80);
//	 HAL_Delay(130);

	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
}
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
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
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
						move_straight_PID(true, straightDistance);
						straightDistance = 0;
						break;
					case 'B':
						straightDistance = arrTofloat(aRxBuffer,1,indexer-1);
						move_straight_PID(false, straightDistance);
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
//				move_straight(false, 100);

//				move_straight_three_point(true,5);
//				move_straight_three_point(false,5);

//				three_points_turn_90deg(true);
//				three_points_turn_90deg(false);

//				turn_deg(90, true, true);

//				targetAngle = 90;
//				turn_left(&targetAngle);


//				for (int i=0; i < 4 ; i++){
//					move_straight_PID(false, 50);
//				}

//				move_straight_PID(true, 200);
//				move_straight_PID(false, 200);

//				targetAngle = 90;
//				turn_left(&targetAngle);
//				turn_right(&targetAngle);

				week_9_v1();

				haveTest = true;

			}
	  	osDelay(1000);
	  }
  /* USER CODE END motors */
}

/* USER CODE BEGIN Header_move_obs_task */
/**
* @brief Function implementing the MoveObs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_move_obs_task */
void move_obs_task(void *argument)
{
  /* USER CODE BEGIN move_obs_task */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	bool haveDone = false;

	uint8_t curStep = -1;
	float length1 = 0;
	float length2 = 0;

	long distanceCount = 0;
  /* Infinite loop */
  for(;;)
  {
  	switch (curStep){
  	 case 0:
  		 robot_move_dis_obs();
  		 curStep += 1;
  		 break;
  	 case 1:
  		 // turn right 90
  		curStep += 1;
  		break;
  	 case 2:
  		 ///// move until pass the obstacle
  		// if not found IR - haven't meet obstacle yet
  		// move until found
  		// => move util not found

  		// if found IR - have meet obstacle
  	  // move until not foud

  		 // save this value - lenght 1 -
  		curStep += 1;
  		break;
  	 case 3:
  		 // turn left 180 - to the other side
  		 curStep += 1;
  		 break;
  	 case 4:
  		 // move util pass the obstacle, then turn to other side again

  		 if (length2 >= length1){
  			 // the case that have meet obstacle after fist 1st deg turn

  			 // turn left 180
  		 } else {
  			 // the case that have not meet obstacle after fist 1st deg turn

  			 // turn left 90
  			 // turn right 90 - if diff it to small - may remove this and go straight
  		 }
  		 curStep += 1;
  	 case 5:
  		 // back to the initial pos
  		 robot_move_dis_obs();
  		 // move the length
  		 curStep += 1;
  	 default:
  		 break;
  	}

  	if (curStep == -1){
  		isMeasureDis = true;
  		move_straight_PID_IR(true, &distanceCount);
  		curStep -= 1;
  		haveDone = false;
  	}
    osDelay(1000);
  }
  /* USER CODE END move_obs_task */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the ShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
  /* Infinite loop */
  for(;;)
  {
//	 OLED_Refresh_Gram();

    osDelay(1);
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


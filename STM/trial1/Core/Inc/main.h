/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define IR_CONST_A 25644.81557
#define IR_CONST_B 260.4233354
#define IR_SAMPLE 50 // init 100

#define __Gyro_Read_Z(_I2C, readGyroData, gyroZ) ({ \
	HAL_I2C_Mem_Read(_I2C,ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, readGyroData, 2, 0xFFFF); \
	gyroZ = readGyroData[0] << 8 | readGyroData[1]; \
})

#define __ADC_Read_Dist(_ADC, dataPoint, IR_data_raw_acc, obsDist, obsTick) ({ \
	HAL_ADC_Start(_ADC); \
	HAL_ADC_PollForConversion(_ADC,20); \
	while (dataPoint < IR_SAMPLE - 1){ \
		IR_data_raw_acc += HAL_ADC_GetValue(_ADC); \
		dataPoint = (dataPoint + 1) % IR_SAMPLE; \
	} \
	if (dataPoint == IR_SAMPLE - 1) { \
		obsDist = IR_CONST_A / (IR_data_raw_acc / dataPoint - IR_CONST_B); \
		obsTick = IR_data_raw_acc / dataPoint; \
		IR_data_raw_acc = 0; \
	} \
})

#define __delay_us(_TIMER4, time) ({ \
	__HAL_TIM_SET_COUNTER(_TIMER4, 0); \
	while (__HAL_TIM_GET_COUNTER(_TIMER4) < time); \
})

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC
#define US_TRIG_Pin GPIO_PIN_4
#define US_TRIG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

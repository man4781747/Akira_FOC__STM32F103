/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define Ang_ADC_Pin GPIO_PIN_3
#define Ang_ADC_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_5
#define nFAULT_GPIO_Port GPIOA
#define Current_W_Pin GPIO_PIN_7
#define Current_W_GPIO_Port GPIOA
#define Current_U_Pin GPIO_PIN_0
#define Current_U_GPIO_Port GPIOB
#define Current_V_Pin GPIO_PIN_1
#define Current_V_GPIO_Port GPIOB
#define SLEEP_Pin GPIO_PIN_12
#define SLEEP_GPIO_Port GPIOB
#define SLEW_Pin GPIO_PIN_13
#define SLEW_GPIO_Port GPIOB
#define GAIN_Pin GPIO_PIN_14
#define GAIN_GPIO_Port GPIOB
#define PWM_ACTIVE_Pin GPIO_PIN_15
#define PWM_ACTIVE_GPIO_Port GPIOB
#define PWM_V_H_Pin GPIO_PIN_8
#define PWM_V_H_GPIO_Port GPIOA
#define PWM_U_H_Pin GPIO_PIN_9
#define PWM_U_H_GPIO_Port GPIOA
#define PWM_W_H_Pin GPIO_PIN_10
#define PWM_W_H_GPIO_Port GPIOA
#define EEPROM_RW__Pin GPIO_PIN_15
#define EEPROM_RW__GPIO_Port GPIOA
#define LED_Status_Pin GPIO_PIN_3
#define LED_Status_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
enum DeviceMode {
  DeviceMode_Stop,
  DeviceMode_SpeedMode,
  DeviceMode_IqMode,
  DeviceMode_PositionMode,
  DeviceMode_SetAngShift
};
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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
#include "stm32h7xx_hal.h"

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
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOE
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOE
#define BMI088_ACCEL_DR_Pin GPIO_PIN_14
#define BMI088_ACCEL_DR_GPIO_Port GPIOC
#define BMI088_GYRO_DR_Pin GPIO_PIN_15
#define BMI088_GYRO_DR_GPIO_Port GPIOC
#define OSD_CS_Pin GPIO_PIN_12
#define OSD_CS_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOD
#define BMI270_CS_Pin GPIO_PIN_15
#define BMI270_CS_GPIO_Port GPIOA
#define SPL06_DR_Pin GPIO_PIN_0
#define SPL06_DR_GPIO_Port GPIOD
#define SPL06_DR_EXTI_IRQn EXTI0_IRQn
#define BMI088_ACCEL_CS_Pin GPIO_PIN_4
#define BMI088_ACCEL_CS_GPIO_Port GPIOD
#define BMI088_GYRO_CS_Pin GPIO_PIN_5
#define BMI088_GYRO_CS_GPIO_Port GPIOD
#define BMI270_DR_Pin GPIO_PIN_7
#define BMI270_DR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define moto2_1_Pin GPIO_PIN_13
#define moto2_1_GPIO_Port GPIOC
#define moto2_2_Pin GPIO_PIN_14
#define moto2_2_GPIO_Port GPIOC
#define Light_switch_Pin GPIO_PIN_15
#define Light_switch_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define key_Pin GPIO_PIN_5
#define key_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define infrared5_Pin GPIO_PIN_12
#define infrared5_GPIO_Port GPIOB
#define infrared4_Pin GPIO_PIN_13
#define infrared4_GPIO_Port GPIOB
#define infrared3_Pin GPIO_PIN_14
#define infrared3_GPIO_Port GPIOB
#define infrared2_Pin GPIO_PIN_15
#define infrared2_GPIO_Port GPIOB
#define infrared1_Pin GPIO_PIN_8
#define infrared1_GPIO_Port GPIOA
#define buzzer_Pin GPIO_PIN_12
#define buzzer_GPIO_Port GPIOA
#define moto1_2_Pin GPIO_PIN_15
#define moto1_2_GPIO_Port GPIOA
#define moto1_1_Pin GPIO_PIN_3
#define moto1_1_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_4
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

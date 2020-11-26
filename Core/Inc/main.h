/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_5
#define CS_GPIO_Port GPIOC
#define DC_Pin GPIO_PIN_1
#define DC_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_7
#define RESET_GPIO_Port GPIOE
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define LED_Z_Pin GPIO_PIN_13
#define LED_Z_GPIO_Port GPIOD
#define LED_Y_Pin GPIO_PIN_14
#define LED_Y_GPIO_Port GPIOD
#define LED_X_Pin GPIO_PIN_15
#define LED_X_GPIO_Port GPIOD
#define Motor_Z_Direction_Pin GPIO_PIN_8
#define Motor_Z_Direction_GPIO_Port GPIOA
#define Motor_Y_Direction_Pin GPIO_PIN_9
#define Motor_Y_Direction_GPIO_Port GPIOA
#define Motor_X_Direction_Pin GPIO_PIN_10
#define Motor_X_Direction_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

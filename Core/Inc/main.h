/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_nucleo_144.h"
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
void Success_Handler(void);
void stopMotor(int);
void controlSpeed(float);
void controlMotor(int, uint32_t, char);
void updateReset(uint32_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor4_IN2_Pin GPIO_PIN_2
#define Motor4_IN2_GPIO_Port GPIOE
#define Motor13_IN1_Pin GPIO_PIN_3
#define Motor13_IN1_GPIO_Port GPIOE
#define Motor14_IN1_Pin GPIO_PIN_5
#define Motor14_IN1_GPIO_Port GPIOE
#define Motor14_IN2_Pin GPIO_PIN_6
#define Motor14_IN2_GPIO_Port GPIOE
#define Motor5_IN2_Pin GPIO_PIN_0
#define Motor5_IN2_GPIO_Port GPIOF
#define Motor5_IN1_Pin GPIO_PIN_1
#define Motor5_IN1_GPIO_Port GPIOF
#define Motor2_IN1_Pin GPIO_PIN_4
#define Motor2_IN1_GPIO_Port GPIOF
#define Motor13_IN2_Pin GPIO_PIN_8
#define Motor13_IN2_GPIO_Port GPIOF
#define Motor1_IN2_Pin GPIO_PIN_2
#define Motor1_IN2_GPIO_Port GPIOC
#define Motor10_IN1_Pin GPIO_PIN_5
#define Motor10_IN1_GPIO_Port GPIOA
#define Motor10_IN2_Pin GPIO_PIN_6
#define Motor10_IN2_GPIO_Port GPIOA
#define Motor1_IN1_Pin GPIO_PIN_1
#define Motor1_IN1_GPIO_Port GPIOB
#define Motor1_SW_Pin GPIO_PIN_13
#define Motor1_SW_GPIO_Port GPIOF
#define Motor3_SW_Pin GPIO_PIN_14
#define Motor3_SW_GPIO_Port GPIOF
#define Motor5_SW_Pin GPIO_PIN_15
#define Motor5_SW_GPIO_Port GPIOF
#define Motor9_SW_Pin GPIO_PIN_7
#define Motor9_SW_GPIO_Port GPIOE
#define Motor8_SW_Pin GPIO_PIN_8
#define Motor8_SW_GPIO_Port GPIOE
#define Motor10_SW_Pin GPIO_PIN_10
#define Motor10_SW_GPIO_Port GPIOE
#define Motor2_SW_Pin GPIO_PIN_11
#define Motor2_SW_GPIO_Port GPIOE
#define Motor11_SW_Pin GPIO_PIN_12
#define Motor11_SW_GPIO_Port GPIOE
#define Motor4_SW_Pin GPIO_PIN_13
#define Motor4_SW_GPIO_Port GPIOE
#define Motor12_SW_Pin GPIO_PIN_14
#define Motor12_SW_GPIO_Port GPIOE
#define Motor13_SW_Pin GPIO_PIN_15
#define Motor13_SW_GPIO_Port GPIOE
#define Motor14_SW_Pin GPIO_PIN_10
#define Motor14_SW_GPIO_Port GPIOB
#define Motor6_IN2_Pin GPIO_PIN_15
#define Motor6_IN2_GPIO_Port GPIOB
#define Motor4_IN1_Pin GPIO_PIN_11
#define Motor4_IN1_GPIO_Port GPIOD
#define Motor3_IN2_Pin GPIO_PIN_12
#define Motor3_IN2_GPIO_Port GPIOD
#define Motor3_IN1_Pin GPIO_PIN_13
#define Motor3_IN1_GPIO_Port GPIOD
#define Motor6_IN1_Pin GPIO_PIN_6
#define Motor6_IN1_GPIO_Port GPIOC
#define Motor8_IN2_Pin GPIO_PIN_7
#define Motor8_IN2_GPIO_Port GPIOC
#define Motor8_IN1_Pin GPIO_PIN_15
#define Motor8_IN1_GPIO_Port GPIOA
#define Motor7_IN1_Pin GPIO_PIN_0
#define Motor7_IN1_GPIO_Port GPIOD
#define Motor7_IN2_Pin GPIO_PIN_1
#define Motor7_IN2_GPIO_Port GPIOD
#define Motor12_IN2_Pin GPIO_PIN_4
#define Motor12_IN2_GPIO_Port GPIOD
#define Motor12_IN1_Pin GPIO_PIN_5
#define Motor12_IN1_GPIO_Port GPIOD
#define Motor11_IN2_Pin GPIO_PIN_6
#define Motor11_IN2_GPIO_Port GPIOD
#define Motor11_IN1_Pin GPIO_PIN_7
#define Motor11_IN1_GPIO_Port GPIOD
#define Motor7_SW_Pin GPIO_PIN_9
#define Motor7_SW_GPIO_Port GPIOG
#define Motor6_SW_Pin GPIO_PIN_14
#define Motor6_SW_GPIO_Port GPIOG
#define Motor2_IN2_Pin GPIO_PIN_6
#define Motor2_IN2_GPIO_Port GPIOB
#define Motor9_IN1_Pin GPIO_PIN_8
#define Motor9_IN1_GPIO_Port GPIOB
#define Motor9_IN2_Pin GPIO_PIN_9
#define Motor9_IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MOTOR_COUNT 14
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

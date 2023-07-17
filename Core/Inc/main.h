/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define ButtonA_Pin GPIO_PIN_0
#define ButtonA_GPIO_Port GPIOA
#define ButtonA_EXTI_IRQn EXTI0_IRQn
#define PWM_MotorA_Pin GPIO_PIN_1
#define PWM_MotorA_GPIO_Port GPIOA
#define PWM_MotorB_Pin GPIO_PIN_2
#define PWM_MotorB_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_3
#define Buzzer_GPIO_Port GPIOA
#define EncoderB_Pin GPIO_PIN_4
#define EncoderB_GPIO_Port GPIOA
#define EncoderA_Pin GPIO_PIN_5
#define EncoderA_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_6
#define IN4_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_7
#define IN3_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_0
#define IN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_1
#define IN1_GPIO_Port GPIOB
#define ButtonB_Pin GPIO_PIN_12
#define ButtonB_GPIO_Port GPIOB
#define InfraRedA_Pin GPIO_PIN_13
#define InfraRedA_GPIO_Port GPIOB
#define InfraRedB_Pin GPIO_PIN_14
#define InfraRedB_GPIO_Port GPIOB
#define TriggerE_Pin GPIO_PIN_15
#define TriggerE_GPIO_Port GPIOB
#define UltraE_Pin GPIO_PIN_8
#define UltraE_GPIO_Port GPIOA
#define UltraC_Pin GPIO_PIN_9
#define UltraC_GPIO_Port GPIOA
#define UltraD_Pin GPIO_PIN_10
#define UltraD_GPIO_Port GPIOA
#define TriggerC_Pin GPIO_PIN_11
#define TriggerC_GPIO_Port GPIOA
#define TriggerD_Pin GPIO_PIN_12
#define TriggerD_GPIO_Port GPIOA
#define LedE_Pin GPIO_PIN_5
#define LedE_GPIO_Port GPIOB
#define LedC_Pin GPIO_PIN_6
#define LedC_GPIO_Port GPIOB
#define LedD_Pin GPIO_PIN_7
#define LedD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

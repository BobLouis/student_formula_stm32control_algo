/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define ready_SW_Pin GPIO_PIN_2
#define ready_SW_GPIO_Port GPIOE
#define ready_SW_EXTI_IRQn EXTI2_IRQn
#define reset_SW_Pin GPIO_PIN_3
#define reset_SW_GPIO_Port GPIOE
#define reset_SW_EXTI_IRQn EXTI3_IRQn
#define precharge_SW_Pin GPIO_PIN_4
#define precharge_SW_GPIO_Port GPIOE
#define precharge_SW_EXTI_IRQn EXTI4_IRQn
#define clear_fault_SW_Pin GPIO_PIN_5
#define clear_fault_SW_GPIO_Port GPIOE
#define clear_fault_SW_EXTI_IRQn EXTI9_5_IRQn
#define current_L_Pin GPIO_PIN_0
#define current_L_GPIO_Port GPIOC
#define current_R_Pin GPIO_PIN_1
#define current_R_GPIO_Port GPIOC
#define steer_Pin GPIO_PIN_0
#define steer_GPIO_Port GPIOA
#define APPSR_Pin GPIO_PIN_2
#define APPSR_GPIO_Port GPIOA
#define APPSL_Pin GPIO_PIN_3
#define APPSL_GPIO_Port GPIOA
#define BPPS_Pin GPIO_PIN_4
#define BPPS_GPIO_Port GPIOA
#define pedals_LED_Pin GPIO_PIN_5
#define pedals_LED_GPIO_Port GPIOA
#define readyToDrive_LED_Pin GPIO_PIN_6
#define readyToDrive_LED_GPIO_Port GPIOA
#define fault_LED_Pin GPIO_PIN_7
#define fault_LED_GPIO_Port GPIOA
#define velocity_Pin GPIO_PIN_4
#define velocity_GPIO_Port GPIOC
#define precharge_LED_Pin GPIO_PIN_5
#define precharge_LED_GPIO_Port GPIOC
#define CAN_fault_LED_Pin GPIO_PIN_0
#define CAN_fault_LED_GPIO_Port GPIOB
#define PWM_FR_Pin GPIO_PIN_1
#define PWM_FR_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_9
#define Buzzer_GPIO_Port GPIOE
#define RST_Pin GPIO_PIN_14
#define RST_GPIO_Port GPIOE
#define CS1_Pin GPIO_PIN_15
#define CS1_GPIO_Port GPIOE
#define PWM_RL_Pin GPIO_PIN_13
#define PWM_RL_GPIO_Port GPIOD
#define PWM_RR_Pin GPIO_PIN_15
#define PWM_RR_GPIO_Port GPIOD
#define PWM_FL_Pin GPIO_PIN_5
#define PWM_FL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define PI 3.14159f
#define DEG2RAD(x) ((x)*PI/180.0f)
#define RAD2DEG(x) ((x)*180.0f/PI)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
char * float_to_string(float value, char* buffer, int16_t precision);
void soft_reset();
void usb_send(char *resp);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor1_PWM0_Pin GPIO_PIN_0
#define Motor1_PWM0_GPIO_Port GPIOC
#define Motor1_PWM1_Pin GPIO_PIN_1
#define Motor1_PWM1_GPIO_Port GPIOC
#define Motor2_PWM0_Pin GPIO_PIN_2
#define Motor2_PWM0_GPIO_Port GPIOC
#define Motor2_PWM1_Pin GPIO_PIN_3
#define Motor2_PWM1_GPIO_Port GPIOC
#define PWM_1_Pin GPIO_PIN_0
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_1
#define PWM_2_GPIO_Port GPIOA
#define Motor1_Pos_Pin GPIO_PIN_4
#define Motor1_Pos_GPIO_Port GPIOA
#define Motor2_Pos_Pin GPIO_PIN_5
#define Motor2_Pos_GPIO_Port GPIOA
#define Motor3_Pos_Pin GPIO_PIN_6
#define Motor3_Pos_GPIO_Port GPIOA
#define Motor4_Pos_Pin GPIO_PIN_7
#define Motor4_Pos_GPIO_Port GPIOA
#define Motor3_Curr_Pin GPIO_PIN_4
#define Motor3_Curr_GPIO_Port GPIOC
#define Motor4_Curr_Pin GPIO_PIN_5
#define Motor4_Curr_GPIO_Port GPIOC
#define Motor1_Curr_Pin GPIO_PIN_0
#define Motor1_Curr_GPIO_Port GPIOB
#define Motor2_Curr_Pin GPIO_PIN_1
#define Motor2_Curr_GPIO_Port GPIOB
#define EC11_Push_Pin GPIO_PIN_2
#define EC11_Push_GPIO_Port GPIOB
#define EC11_Push_EXTI_IRQn EXTI2_IRQn
#define EC11_A_Pin GPIO_PIN_12
#define EC11_A_GPIO_Port GPIOB
#define EC11_A_EXTI_IRQn EXTI15_10_IRQn
#define EC11_B_Pin GPIO_PIN_13
#define EC11_B_GPIO_Port GPIOB
#define EC11_B_EXTI_IRQn EXTI15_10_IRQn
#define Button_Pin GPIO_PIN_6
#define Button_GPIO_Port GPIOC
#define Work_LED_Pin GPIO_PIN_7
#define Work_LED_GPIO_Port GPIOC
#define Motor4_PWM0_Pin GPIO_PIN_8
#define Motor4_PWM0_GPIO_Port GPIOC
#define Motor4_PWM1_Pin GPIO_PIN_9
#define Motor4_PWM1_GPIO_Port GPIOC
#define Encoder1_C1_Pin GPIO_PIN_8
#define Encoder1_C1_GPIO_Port GPIOA
#define Encoder1_C2_Pin GPIO_PIN_9
#define Encoder1_C2_GPIO_Port GPIOA
#define Encoder2_C1_Pin GPIO_PIN_15
#define Encoder2_C1_GPIO_Port GPIOA
#define Motor3_PWM0_Pin GPIO_PIN_10
#define Motor3_PWM0_GPIO_Port GPIOC
#define Motor3_PWM1_Pin GPIO_PIN_11
#define Motor3_PWM1_GPIO_Port GPIOC
#define Motor_En_Pin GPIO_PIN_2
#define Motor_En_GPIO_Port GPIOD
#define Encoder2_C2_Pin GPIO_PIN_3
#define Encoder2_C2_GPIO_Port GPIOB
#define Encoder3_C1_Pin GPIO_PIN_4
#define Encoder3_C1_GPIO_Port GPIOB
#define Encoder3_C2_Pin GPIO_PIN_5
#define Encoder3_C2_GPIO_Port GPIOB
#define Encoder4_C1_Pin GPIO_PIN_6
#define Encoder4_C1_GPIO_Port GPIOB
#define Encoder4_C2_Pin GPIO_PIN_7
#define Encoder4_C2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

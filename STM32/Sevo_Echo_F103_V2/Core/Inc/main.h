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
char * float_to_string(float value, char* buffer, int16_t precision);
void soft_reset();
void usb_send(char *resp);
uint32_t millis();
uint64_t get_us(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Power_Proj_Pin GPIO_PIN_13
#define Power_Proj_GPIO_Port GPIOC
#define ADC_Servo2_Pin GPIO_PIN_4
#define ADC_Servo2_GPIO_Port GPIOA
#define ADC_Servo3_Pin GPIO_PIN_5
#define ADC_Servo3_GPIO_Port GPIOA
#define ADC_Servo4_Pin GPIO_PIN_6
#define ADC_Servo4_GPIO_Port GPIOA
#define ADC_Servo5_Pin GPIO_PIN_7
#define ADC_Servo5_GPIO_Port GPIOA
#define ADC_Servo0_Pin GPIO_PIN_0
#define ADC_Servo0_GPIO_Port GPIOB
#define ADC_Servo1_Pin GPIO_PIN_1
#define ADC_Servo1_GPIO_Port GPIOB
#define Echo_2_Pin GPIO_PIN_2
#define Echo_2_GPIO_Port GPIOB
#define Echo_2_EXTI_IRQn EXTI2_IRQn
#define Echo_0_Pin GPIO_PIN_10
#define Echo_0_GPIO_Port GPIOB
#define Echo_0_EXTI_IRQn EXTI15_10_IRQn
#define Echo_1_Pin GPIO_PIN_11
#define Echo_1_GPIO_Port GPIOB
#define Echo_1_EXTI_IRQn EXTI15_10_IRQn
#define Echo_Trig_Pin GPIO_PIN_14
#define Echo_Trig_GPIO_Port GPIOB
#define Echo_3_Pin GPIO_PIN_3
#define Echo_3_GPIO_Port GPIOB
#define Echo_3_EXTI_IRQn EXTI3_IRQn
#define Echo_4_Pin GPIO_PIN_4
#define Echo_4_GPIO_Port GPIOB
#define Echo_4_EXTI_IRQn EXTI4_IRQn
#define Echo_5_Pin GPIO_PIN_5
#define Echo_5_GPIO_Port GPIOB
#define Echo_5_EXTI_IRQn EXTI9_5_IRQn
#define Echo_6_Pin GPIO_PIN_6
#define Echo_6_GPIO_Port GPIOB
#define Echo_6_EXTI_IRQn EXTI9_5_IRQn
#define Echo_7_Pin GPIO_PIN_7
#define Echo_7_GPIO_Port GPIOB
#define Echo_7_EXTI_IRQn EXTI9_5_IRQn
#define Echo_8_Pin GPIO_PIN_8
#define Echo_8_GPIO_Port GPIOB
#define Echo_8_EXTI_IRQn EXTI9_5_IRQn
#define Echo_9_Pin GPIO_PIN_9
#define Echo_9_GPIO_Port GPIOB
#define Echo_9_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

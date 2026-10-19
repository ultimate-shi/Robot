/*
 * motor_encoder.h
 *
 *  Created on: Jan 27, 2026
 *      Author: 12817
 */

#ifndef INC_MOTOR_ENCODER_H_
#define INC_MOTOR_ENCODER_H_

#include "stm32f4xx_hal.h"

#define MOTOR_ENCODER_MAX_CHS	4
#define MOTOR_ENCODER_INC_MIDDLE	0x40000000L	//增量编码器使用int32_t的中间值，避免上下溢出问题
#define MOTOR_ENCODER_ABS_MIDDLE 	0x00008000L	//绝对编码器只使用中间值

typedef struct motor_encoder_s {
	uint16_t magic;
	TIM_HandleTypeDef *htim;
	int32_t high_count;
	int is_halfword_timer;
} motor_encoder_t;

void motor_encoder_init();
void motor_encoder_set(motor_encoder_t * mep, int32_t value);
int32_t motor_encoder_get(motor_encoder_t *mep);
int32_t motor_encoder_get_dir(motor_encoder_t *mep, int* dir);
motor_encoder_t * motor_encoder_add(TIM_HandleTypeDef *htim);
void motor_period_elapsed_callback(TIM_HandleTypeDef *htim);
#endif /* INC_MOTOR_ENCODER_H_ */

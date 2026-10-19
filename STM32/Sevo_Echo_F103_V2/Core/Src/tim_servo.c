/*
 * servo.c
 *
 *  Created on: Sep 7, 2025
 *      Author: Luke 1281772125@qq.com
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "tim_servo.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

#define SERVO_CYCLE_LOW_US	485.0f
#define SERVO_CYCLE_HIGH_US	2515.0f

ServoTIM _servo_timers[SERVO_NUMS];					//舵机硬件Timer

//用于保护舵机的控制数据
#define SERVO_UPDATE_INTERVAL   100
typedef struct servo_status_s {
    uint16_t pwm_us;
    uint64_t update_ms;
    //SemaphoreHandle_t mutex;
} servo_status_t;
static servo_status_t _servo_status[SERVO_NUMS];

int16_t set_servo_timer(uint8_t ch, TIM_HandleTypeDef * tim, uint32_t tim_ch) {
	if(ch >= SERVO_NUMS) {
		printf("setup_servo_timer(%u): Out of range(%d)!\n", ch, SERVO_NUMS);
		return -1;
	}
	_servo_timers[ch].tim = tim;
	_servo_timers[ch].ch = tim_ch;
	return 0;
}

// 设置舵机的PWM, us = 500 ~ 2500
void set_servo_pwm(uint16_t ch, uint16_t us) {
	const ServoTIM *st = &_servo_timers[ch];
	if(_servo_status[ch].pwm_us == 0 && us > 0) //若PWM处于关闭状态，则打开
	{
		HAL_TIM_PWM_Start(st->tim, st->ch);
	}
    else if(_servo_status[ch].pwm_us > 0 && us == 0) //若PWM处于打开状态，则关闭
    {
    	HAL_TIM_PWM_Stop(st->tim, st->ch);
    }
	__HAL_TIM_SET_COMPARE(st->tim, st->ch, us);
    //Save status
    _servo_status[ch].update_ms = millis();
    _servo_status[ch].pwm_us = us;
}

//读取舵机PWM的微秒值
uint16_t get_servo_pwm(uint16_t ch) {
	const ServoTIM *st = &_servo_timers[ch];
	return __HAL_TIM_GET_COMPARE(st->tim, st->ch);
}

//初始化舵机
int16_t init_servo()
{
    //绑定硬件Timer
	set_servo_timer(0, &htim1, TIM_CHANNEL_1);	
	set_servo_timer(1, &htim1, TIM_CHANNEL_2);	
    set_servo_timer(2, &htim2, TIM_CHANNEL_1);
    set_servo_timer(3, &htim2, TIM_CHANNEL_2);
    set_servo_timer(4, &htim2, TIM_CHANNEL_3);
    set_servo_timer(5, &htim2, TIM_CHANNEL_4);
	
    //复位舵机
    for(int16_t i=0; i<SERVO_NUMS; i++) {
    	_servo_status[i].pwm_us = 0;
        _servo_status[i].update_ms = millis();
        set_servo_pwm(i, _servo_status[i].pwm_us);
    }
    return 0;
}

//上位机至少要维持10Hz的更新率，否则舵机将会被关闭
void process_servo(uint32_t now_ms) {
    for(int16_t i=0; i<SERVO_NUMS; i++) {
        if(get_servo_pwm(i) > 0 
        && (now_ms - _servo_status[i].update_ms) >= SERVO_UPDATE_INTERVAL) {
            set_servo_pwm(i, 0);
        }
    }
}
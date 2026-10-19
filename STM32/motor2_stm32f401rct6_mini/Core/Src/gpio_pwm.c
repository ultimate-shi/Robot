/*
 * gpio_pwm.c
 *
 *  Created on: Jan 24, 2026
 *      Author: Luke 1281772125@qq.com
 */
#include "gpio_pwm.h"

#define GPIO_PWM_MAGIC	0x1359

gpio_pwm_t _gpio_pwms[SYS_PWM_CHANNELS];
uint16_t _gpio_pwm_cycle_ms = 0;
uint16_t _gpio_pwm_on = 0;

void gpio_pwm_init() {
	for(int i=0; i<SYS_PWM_CHANNELS; i++) {
		_gpio_pwms[i].magic = GPIO_PWM_MAGIC;
		_gpio_pwms[i].gpiox = NULL;
		_gpio_pwms[i].pin = 0;
		_gpio_pwms[i].hi_duty_ms = 0;
		_gpio_pwms[i].cycle_duty_ms = 0;
		_gpio_pwms[i].cycle_done = 0;
	}
	_gpio_pwm_on = 0;
}

gpio_pwm_t * gpio_pwm_add(GPIO_TypeDef *gpiox, uint16_t pin) {
	gpio_pwm_t * gpp;
	for(int i=0; i<SYS_PWM_CHANNELS; i++) {
		gpp = &_gpio_pwms[i];
		if(gpp->magic == GPIO_PWM_MAGIC && gpp->gpiox == NULL) {
			__disable_irq();
			_gpio_pwms[i].gpiox = gpiox;
			_gpio_pwms[i].pin = pin;
			_gpio_pwms[i].hi_duty_ms = 0;
			_gpio_pwms[i].cycle_duty_ms = 0;
			_gpio_pwms[i].cycle_done = 0;
			__enable_irq();
			return gpp;
		}
		if(gpp->magic == GPIO_PWM_MAGIC && gpp->gpiox == gpiox && gpp->pin == pin)
			return gpp;
	}
	return NULL;
}

void gpio_pwm_set(gpio_pwm_t * gpp, uint16_t duty) {
	if(gpp == NULL)
		return;
	if(duty > 100)
		duty = 100;
	duty /= 100/SYS_PWM_CYCLE_MS;
	if(gpp->magic == GPIO_PWM_MAGIC) {
		gpp->hi_duty_ms = duty;
		return ;
	}
}

void gpio_pwm_enable() {
	_gpio_pwm_on = 1;
}

void gpio_pwm_disable() {
	_gpio_pwm_on = 0;
	osDelay(110); //等待完成最后一个周期
	//把所有引脚拉低复位
	for(int i=0; i<SYS_PWM_CHANNELS; i++) {
		gpio_pwm_t *gpp = &_gpio_pwms[i];
		if(gpp->magic == GPIO_PWM_MAGIC && gpp->gpiox != NULL)
			HAL_GPIO_WritePin(gpp->gpiox, gpp->pin, GPIO_PIN_RESET);
	}
}

inline void gpio_pwm_1ms_tick() {
	  if(_gpio_pwm_on) {
		  if(_gpio_pwm_cycle_ms == 0) { //从PWM周期开始开启所有通道
			  for(int i=0; i<SYS_PWM_CHANNELS; i++) {
				  gpio_pwm_t *gpp = &_gpio_pwms[i];
				  if(gpp->gpiox == NULL)
					  break;
				  gpp->cycle_duty_ms = gpp->hi_duty_ms;
				  gpp->cycle_done = 0;
				  if(gpp->cycle_duty_ms == 0) {
					  HAL_GPIO_WritePin(gpp->gpiox, gpp->pin, GPIO_PIN_RESET);
					  gpp->cycle_done = 1;
					  continue;
				  }
				  HAL_GPIO_WritePin(gpp->gpiox, gpp->pin, GPIO_PIN_SET);
				  if(gpp->cycle_duty_ms >= SYS_PWM_CYCLE_MS)
					  gpp->cycle_done = 1;
			  }
		  }
		  else {
			  for(int i=0; i<SYS_PWM_CHANNELS; i++) {
				  gpio_pwm_t *gpp = &_gpio_pwms[i];
				  if(gpp->gpiox == NULL)
					  break;
				  if((!gpp->cycle_done) && (_gpio_pwm_cycle_ms >= gpp->hi_duty_ms) ) {
					  HAL_GPIO_WritePin(gpp->gpiox, gpp->pin, GPIO_PIN_RESET);
					  gpp->cycle_done = 1;
				  }
			  }
		  }
		  _gpio_pwm_cycle_ms++;
		  if(_gpio_pwm_cycle_ms >= SYS_PWM_CYCLE_MS) {
			  _gpio_pwm_cycle_ms = 0;
		  }
	  }
}

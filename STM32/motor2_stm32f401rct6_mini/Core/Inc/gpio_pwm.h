/*
 * gpio_pwm.h
 *
 *  Created on: Jan 24, 2026
 *      Author: 12817
 */

#ifndef INC_GPIO_PWM_H_
#define INC_GPIO_PWM_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

typedef struct gpio_pwm_s {
	uint16_t magic;
	GPIO_TypeDef * gpiox;
	uint16_t pin;
	uint16_t hi_duty_ms;	// 0 ~ 100ms
	uint16_t cycle_duty_ms;
	uint16_t cycle_done;
} gpio_pwm_t;


#define SYS_PWM_CHANNELS	10
#define SYS_PWM_CYCLE_MS	20

void gpio_pwm_init();
gpio_pwm_t * gpio_pwm_add(GPIO_TypeDef *gpiox, uint16_t pin);
void gpio_pwm_set(gpio_pwm_t * gpp, uint16_t duty);
void gpio_pwm_enable();
void gpio_pwm_disable();
void gpio_pwm_1ms_tick();
#endif /* INC_GPIO_PWM_H_ */

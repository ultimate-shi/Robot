/*
 * adc.h
 *
 *  Created on: Jan 27, 2026
 *      Author: 12817
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define ADC_MIN_VALUE	0
#define ADC_MAX_VALUE	4095

#ifdef _ADC_WTD_
typedef void (*adc_wtd_callback_t)(int,int,void *);
typedef struct adc_watchdog_s {
	uint16_t min;
	uint16_t max;
	QueueHandle_t queue;
} adc_watchdog_t;
void adc_set_range(int ch, uint16_t min, uint16_t max, QueueHandle_t queue);
#endif

void adc_init(ADC_HandleTypeDef* hadc);
uint16_t adc_is_ready();
uint16_t adc_read(uint16_t ch);
uint16_t adc_read_mv(uint16_t ch);
void adc_conv_callback(ADC_HandleTypeDef *hadc);

uint16_t adc_get_voltage(uint16_t adc_value);
uint16_t adc_voltage_to_adc(uint16_t voltage_mv);
#endif /* INC_ADC_H_ */

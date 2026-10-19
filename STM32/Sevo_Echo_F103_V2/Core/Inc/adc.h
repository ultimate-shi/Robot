/*
 * adc.h
 *
 *  Created on: Jan 27, 2026
 *      Author: 12817
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f1xx_hal.h"

#define ADC_MIN_VALUE	0
#define ADC_MAX_VALUE	4095

void adc_init(ADC_HandleTypeDef* hadc);
uint16_t adc_is_ready();
uint16_t adc_read(uint16_t ch);
uint16_t adc_read_mv(uint16_t ch);
void adc_conv_callback(ADC_HandleTypeDef *hadc);

uint16_t adc_get_voltage(uint16_t adc_value);
uint16_t adc_voltage_to_adc(uint16_t voltage_mv);
#endif /* INC_ADC_H_ */

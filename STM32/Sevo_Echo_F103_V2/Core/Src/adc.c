/*
 * adc.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Luke 1281772125@qq.com
 */

#include "adc.h"

#define ADC_AVG_COUNT	256
#define ADC_CH_MAX		7
/*
 对应舵机0的AS5600 ADC改为PCB2026/03版本的Echo0
 对应舵机1的AS5600 ADC改为PCB2026/03版本的Echo1
 ！注意！使用ADC模式时，AS5600模块的PGO引脚电阻R4必须断开
*/

#define VREFINT_CAL_V        1200    // Vrefint 标称校准电压，单位：mV（1.2V = 1200mV）
#define ADC_FULL_SCALE       4095    // 12位 ADC 满量程值（右对齐）

static ADC_HandleTypeDef* _adc_hadc;
static volatile uint16_t _adc_count;
static volatile uint16_t _adc_ready = 0;

static volatile uint32_t _adc_sum[ADC_CH_MAX];
static volatile uint16_t _adc_val[ADC_CH_MAX];
static volatile uint16_t _adc_avg[ADC_CH_MAX];

static uint16_t _adc_ch_nums = 0;

static void adc_clear_val() {
	for(int i=0; i<_adc_ch_nums; i++) {
		_adc_sum[i] = 0;
		_adc_avg[i] = 0;
	}
	_adc_ready = 0;
	__disable_irq();
	_adc_count = 0;
	__enable_irq();
}

void adc_init(ADC_HandleTypeDef* hadc) {
	_adc_hadc = hadc;
	_adc_ch_nums = ADC_CH_MAX;
	adc_clear_val();
	HAL_ADCEx_Calibration_Start(hadc); 
	HAL_ADC_Start_DMA(hadc, (uint32_t*)_adc_val, ADC_CH_MAX);
}

uint16_t adc_is_ready() {
	return _adc_ready;
}

uint16_t adc_read_mv(uint16_t ch) {
	if(ch < _adc_ch_nums)
		return adc_get_voltage(_adc_avg[ch]);
	return 0;
}

uint16_t adc_read(uint16_t ch) {
	if(ch < _adc_ch_nums)
		return _adc_avg[ch];
	return 0;
}

void adc_conv_callback(ADC_HandleTypeDef *hadc) {
	if(hadc != _adc_hadc)
		return;

	_adc_count++;
	for(int i=0; i<_adc_ch_nums; i++)
		_adc_sum[i] += _adc_val[i];
	if(_adc_count < ADC_AVG_COUNT) {
		HAL_ADC_Start_DMA(hadc, (uint32_t*)_adc_val, _adc_ch_nums);
		return;
	}
	HAL_ADC_Stop_DMA(hadc);
	for(int i=0; i<_adc_ch_nums; i++) {
		_adc_avg[i] = _adc_sum[i] / _adc_count;
		_adc_sum[i] = 0;
	}
	_adc_count = 0;
	_adc_ready = 1;
	HAL_ADC_Start_DMA(hadc, (uint32_t*)_adc_val, _adc_ch_nums);
}

uint16_t adc_get_voltage(uint16_t adc_value)
{
    // 1. 获取你读取到的 Vrefint 实时原始值
    uint16_t D_vref = _adc_avg[ADC_CH_MAX-1]; 

    // 2. 健壮性检查
    if (D_vref == 0) return 0;

    /* 
     * 3. 直接计算（利用等比关系）
     * 公式：Vin = (adc_value / D_vref) * 1200mV
     * 先乘 1200 是为了保持精度且避免浮点运算
     */
    uint32_t actual_voltage_mv = (uint32_t)adc_value * 1200 / D_vref;

    return (uint16_t)actual_voltage_mv;
}


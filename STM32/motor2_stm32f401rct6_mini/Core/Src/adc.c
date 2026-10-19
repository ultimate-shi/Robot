/*
 * adc.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Luke 1281772125@qq.com
 */

#include "adc.h"

#define ADC_AVG_COUNT	256
#define ADC_CH_MAX		9

#define VREFINT_CAL_V        1200    // Vrefint 标称校准电压，单位：mV（1.2V = 1200mV）
#define ADC_FULL_SCALE       4095    // 12位 ADC 满量程值（右对齐）

static ADC_HandleTypeDef* _adc_hadc;
static volatile uint16_t _adc_count;
static volatile uint16_t _adc_ready = 0;

static volatile uint32_t _adc_sum[ADC_CH_MAX];
static volatile uint16_t _adc_val[ADC_CH_MAX];
static volatile uint16_t _adc_avg[ADC_CH_MAX];
#ifdef _ADC_WTD_
adc_watchdog_t _adc_wtd[ADC_CH_MAX];
#endif
static uint16_t _adc_ch_nums = 0;

static void adc_clear_val() {
	for(int i=0; i<_adc_ch_nums; i++) {
		_adc_sum[i] = 0;
		_adc_avg[i] = 0;
#ifdef _ADC_WTD_
		_adc_wtd[i].queue = NULL;
		_adc_wtd[i].min = 0;
		_adc_wtd[i].max = 4095;
#endif
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
	//HAL_ADCEx_Calibration_Start(hadc); //Not found in STM32F401xxx
	HAL_ADC_Start_DMA(hadc, (uint32_t*)_adc_val, ADC_CH_MAX);
}

#ifdef _ADC_WTD_
void adc_set_range(int ch, uint16_t min, uint16_t max, QueueHandle_t queue) {
	if(ch >= ADC_CH_MAX)
		return;
	adc_watchdog_t *awtd = &_adc_wtd[ch];
	awtd->min = min;
	awtd->max = max;
	awtd->queue = queue;
}
#endif

uint16_t adc_is_ready() {
	return _adc_ready;
}

uint16_t adc_read_mv(uint16_t ch) {

	if(ch < _adc_ch_nums)
		return adc_get_voltage(_adc_avg[ch]);
	/*
	if(ch < _adc_ch_nums) {
		uint32_t mv = _adc_avg[ch];
		mv = mv * 3300 / 4095;
		return mv;
	}
	*/
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
#ifdef _ADC_WTD_
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint16_t evt_data = 0;
#endif
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
#ifdef _ADC_WTD_
		adc_watchdog_t* awtd = &_adc_wtd[i];
		if( awtd->queue != NULL) {
			if(_adc_avg[i] > awtd->max || _adc_avg[i] < awtd->min) {
				evt_data = (uint16_t)i;
				if(_adc_avg[i] < awtd->min )
					evt_data |= 0x8000;
				xQueueSendFromISR(awtd->queue, &evt_data, &xHigherPriorityTaskWoken);
			}
		}
#endif
	}
	_adc_count = 0;
	_adc_ready = 1;
	HAL_ADC_Start_DMA(hadc, (uint32_t*)_adc_val, _adc_ch_nums);
}

/**
 * @brief  利用 Vrefint 矫正 ADC 采集值，返回实际电压毫伏值（mV）
 * @param  adc_value：需要矫正的 ADC 采集值（其他通道的原始数字值）
 * @retval 矫正后的实际电压，单位：mV（无符号16位，最大可表示 65535 mV，满足 3.3V 供电需求）
 */
uint16_t adc_get_voltage(uint16_t adc_value)
{
    // 步骤1：获取关键参数
    // 1. 读取 Vrefint 出厂校准值（从 Flash 系统存储区读取半字数据）
    uint16_t Vrefint_CAL = *(uint16_t *)VREFINT_CAL_ADDR;

    // 2. 读取你已保存的 Vrefint 实际 ADC 采集平均值
    uint16_t D_vref = _adc_avg[ADC_CH_MAX-1]; // 假设 Vrefint 的采集值存储在 _adc_avg 数组的最后一个位置（索引为 ADC_CH_MAX-1）

    // 步骤2：异常值判断（避免除零错误，提升函数健壮性）
    if (D_vref == 0 || Vrefint_CAL == 0)
    {
        return 0; // 采集异常或校准值读取失败，返回 0 标识错误
    }

    // 步骤3：计算当前实际的 Vdda 供电电压（单位：mV）
    // 公式：Vdda_actual(mV) = (Vrefint 标称校准电压 × 出厂校准值) / Vrefint 实际采集值
    uint32_t Vdda_actual = 3300 * Vrefint_CAL / D_vref;

    // 步骤4：计算并返回矫正后的实际输入电压（单位：mV）
    // 公式：Vin(mV) = (ADC采集值 × Vdda实际电压) / ADC满量程值
    // 强制转换为 uint32_t 避免 16 位数据溢出
    uint32_t actual_voltage_mv = (uint32_t)adc_value * Vdda_actual / ADC_FULL_SCALE;

    // 转换为 uint16_t 返回（满足函数返回值类型要求，3.3V 供电下最大约 3300 mV，不会溢出）
    return (uint16_t)actual_voltage_mv;
}

uint16_t adc_voltage_to_adc(uint16_t voltage_mv)
{
    // 步骤1：获取关键参数
    // 1. 读取 Vrefint 出厂校准值（从 Flash 系统存储区读取半字数据）
    uint16_t Vrefint_CAL = *(uint16_t *)VREFINT_CAL_ADDR;

    // 2. 读取你已保存的 Vrefint 实际 ADC 采集平均值
    uint16_t D_vref = _adc_avg[ADC_CH_MAX-1]; // 假设 Vrefint 的采集值存储在 _adc_avg 数组的最后一个位置（索引为 ADC_CH_MAX-1）

    // 步骤2：异常值判断（避免除零错误，提升函数健壮性）
    if (D_vref == 0 || Vrefint_CAL == 0)
    {
        return 0; // 采集异常或校准值读取失败，返回 0 标识错误
    }

    // 步骤3：计算当前实际的 Vdda 供电电压（单位：mV）
    // 公式和之前一致，先获取精准的 Vdda 实际值
    uint32_t Vdda_actual = 3300 * Vrefint_CAL / D_vref;

    // 步骤4：异常判断：输入电压超过 Vdda 实际值（超出 ADC 采集范围）
    if (voltage_mv > Vdda_actual)
    {
        return ADC_FULL_SCALE; // 返回 ADC 满量程值（4095），标识超出采集范围
    }

    // 步骤5：反推对应的 ADC 数字值
    // 逆用 ADC 转换公式：ADC值 = (输入电压 × ADC满量程值) / Vdda实际值
    // 强制转换为 uint32_t 避免 16 位数据溢出
    uint32_t adc_value = (uint32_t)voltage_mv * ADC_FULL_SCALE / Vdda_actual;

    // 转换为 uint16_t 返回（符合函数返回值类型要求，范围 0~4095，无溢出）
    return (uint16_t)adc_value;
}

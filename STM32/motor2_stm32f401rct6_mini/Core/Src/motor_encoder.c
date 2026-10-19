/*
 * motor_encoder.c
 *
 *  Created on: Jan 27, 2026
 *      Author: Luke 1281772125@qq.com
 */
#include "cmsis_os.h"
#include "motor_encoder.h"
#include <stdint.h>

static motor_encoder_t _motor_encoders[MOTOR_ENCODER_MAX_CHS];

void motor_encoder_init() {
	motor_encoder_t *mep;
	for(int i=0; i<MOTOR_ENCODER_MAX_CHS; i++) {
		mep = &_motor_encoders[i];
		mep->magic = 0x1971;
		mep->htim = NULL;
		mep->is_halfword_timer = 0;
	}
}

motor_encoder_t * motor_encoder_find(TIM_HandleTypeDef *htim) {
	motor_encoder_t *mep;
	for(int i=0; i<MOTOR_ENCODER_MAX_CHS; i++) {
		mep = &_motor_encoders[i];
		if(mep->magic == 0x1971 && mep->htim == htim)
			return mep;
	}
	return NULL;
}

motor_encoder_t * motor_encoder_add(TIM_HandleTypeDef *htim) {
	motor_encoder_t *mep = motor_encoder_find(htim);
	if(mep != NULL)
		return mep;
	mep = motor_encoder_find(NULL);
	if(mep == NULL)
		return NULL;
	mep->htim = htim;
	__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_UPDATE);
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
	if(htim->Instance != TIM5 && htim->Instance != TIM2 )	{ //Timer2 和 Timer5的计数范围是32位，不会溢出，因此不需要开启溢出中断
		__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);     		//手动开启溢出中断
		mep->is_halfword_timer = 1;
	}
	else {
		mep->is_halfword_timer = 0;
	}
	return mep;
}

#ifdef _VERSION_1_
void motor_encoder_set(motor_encoder_t * mep, int32_t value) {
	if(mep == NULL || mep->magic != 0x1971 || value < 0)
		return;
	if(mep->is_halfword_timer) {
		uint32_t uvalue = (uint32_t)value;
		mep->high_count = uvalue >> 16;
		__HAL_TIM_SET_COUNTER(mep->htim, uvalue & 0xFFFF);
	}
	else {
		__HAL_TIM_SET_COUNTER(mep->htim, value);
	}
}


static int32_t _motor_encoder_get(motor_encoder_t *mep) {
    if(mep->is_halfword_timer) {
        uint16_t low;
        int32_t high;
        
        // 1. 读取当前的溢出统计值
        high = mep->high_count;
        // 2. 读取硬件CNT
        low = __HAL_TIM_GET_COUNTER(mep->htim);
        
        /* 
         * 关键：检查是否有“漏掉”的溢出。
         * 如果溢出标志置位了，且 CNT 已经翻转回到了较小的数值（比如小于 0x7FFF），
         * 说明计数器已经溢出了，但回调函数 motor_period_elapsed_callback 可能还没跑。
         */
        if (__HAL_TIM_GET_FLAG(mep->htim, TIM_FLAG_UPDATE) != RESET) {
            if (low < 0x7FFF) { // 说明是向上溢出
                high++;
            } else {            // 说明是向下溢出
                high--;
            }
            // 注意：这里不需要清空标志位，留给中断服务函数去清。
            // 这里只是逻辑补偿。
        }
        
        return (int32_t)((high << 16) | low);
    }
    else {
        return (int32_t)__HAL_TIM_GET_COUNTER(mep->htim);
    }
}
#endif
void motor_encoder_set(motor_encoder_t * mep, int32_t value) 
{
    if (mep == NULL || mep->magic != 0x1971 || value < 0) return;

    if (mep->is_halfword_timer) {
        uint32_t uvalue = (uint32_t)value;

		/* ================= 进入临界区 ================= */
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        mep->high_count = uvalue >> 16;
        __HAL_TIM_SET_COUNTER(mep->htim, uvalue & 0xFFFF);
        __HAL_TIM_CLEAR_FLAG(mep->htim, TIM_FLAG_UPDATE); // 清除可能由赋值引起的误触发标志
		
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        /* ================= 退出临界区 ================= */
    } else {
        __HAL_TIM_SET_COUNTER(mep->htim, value);
    }
}

static int32_t _motor_encoder_get(motor_encoder_t *mep) 
{
    if (mep->is_halfword_timer) {
        uint32_t high1, high2;
        uint16_t low;

        /* ================= 进入临界区 ================= */
        // 在临界区内，中断被关闭，任务切换被禁止，确保读取 high 和 low 具有绝对原子性
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        /* 
         * 双读取机制：
         * 确保在读取硬件 CNT (low) 的前后，软件高位计数器 high_count 没有被中断修改。
         * 如果两次读取的 high 不一致，说明在读取期间刚好发生了溢出中断，必须重新读取。
         */
        do {
            high1 = mep->high_count;
            low   = __HAL_TIM_GET_COUNTER(mep->htim);
            high2 = mep->high_count;
        } while (high1 != high2);

        /* 
         * 临界区边界补偿：
         * 如果硬件溢出标志已经置位，但中断服务函数还没来得及运行（即 high1==high2 依然成立），
         * 我们需要根据当前定时器的计数方向，手动对高位进行临时补偿。
         */
        if (__HAL_TIM_GET_FLAG(mep->htim, TIM_FLAG_UPDATE) != RESET) {
            // 再次读取一次 CNT，确保不是在刚清空标志后读取的伪状态
            low = __HAL_TIM_GET_COUNTER(mep->htim);
            
            // 根据定时器实际的方向硬件指示来决定加减，比判断 low < 0x7FFF 更精准
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(mep->htim)) {
                // 如果是向下计数且低位已经翻转到高区（比如接近0xFFFF），说明确实漏了减1
                if (low > 0x7FFF) {
                    high1--;
                }
            } else {
                // 如果是向上计数且低位已经翻转到低区（比如接近0x0000），说明确实漏了加1
                if (low < 0x7FFF) {
                    high1++;
                }
            }
        }

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        /* ================= 退出临界区 ================= */
        return (int32_t)((high1 << 16) | low);
    } else {
        return (int32_t)__HAL_TIM_GET_COUNTER(mep->htim);
    }
}

int32_t motor_encoder_get_dir(motor_encoder_t *mep, int* dir) {
	if(mep == NULL || mep->magic != 0x1971)
		return 0;
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(mep->htim))
		*dir = -1;
	else
		*dir = 1;
	return _motor_encoder_get(mep);
}

int32_t motor_encoder_get(motor_encoder_t *mep) {
	if(mep == NULL || mep->magic != 0x1971)
		return 0;
	return _motor_encoder_get(mep);
}

void motor_period_elapsed_callback(TIM_HandleTypeDef *htim) {
	motor_encoder_t *mep = motor_encoder_find(htim);
	if(mep == NULL || mep->magic != 0x1971)
		return;
	if(!mep->is_halfword_timer) 
		return;
	
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(mep->htim))
		mep->high_count--;
	else
		mep->high_count++;
}
/*
 * echo.c
 *
 *  Created on: Sep 9, 2025
 *      Author: Luke 1281772125@qq.com
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio_sr04p.h"

//SR04P最大有效探测距离为450cm，60毫秒超时
//考虑到扫描效率，限制最大探测距离为250cm
#define SR04P_TIMEOUT_MS  (30)
#define SR04P_MAX_DISTANCE_MM  (340*SR04P_TIMEOUT_MS/2)

/*
超声波通道0 由EXITI0中断改为EXITI10中断，对应PCB2026/03版本的I2C_SCL
超声波通道1 由EXITI1中断改为EXITI11中断，对应PCB2026/03版本的I2C_SDA
*/

#define ECHO_STAGE_IDLE        0
#define ECHO_STAGE_WAIT_TRIG   1
#define ECHO_STAGE_WAIT_ECHO   2
#define ECHO_STAGE_END         3

static GPIO_TypeDef * _trig_gpiox = Echo_Trig_GPIO_Port;
static uint16_t _trig_pin = Echo_Trig_Pin;    //触发引脚，采用统一触发引脚

typedef struct sr04p_s {
	GPIO_TypeDef * gpiox;
	uint16_t echo_pin;
    volatile int64_t trig_us;            //触发的时间（echo_pin上升沿）
    volatile int64_t echo_us;            //回声时间
    volatile uint32_t distance;          //以毫米为单位的距离值
    uint32_t distance_ms;                //距离测量的时间戳
    volatile uint16_t stage;             //当前测距的阶段
} sr04p_t;

sr04p_t _sr04ps[SR04P_NUM] = {
    //{ .gpiox = Echo_0_GPIO_Port, 	.echo_pin = Echo_0_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    //{ .gpiox = Echo_1_GPIO_Port, 	.echo_pin = Echo_1_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_2_GPIO_Port,	.echo_pin = Echo_2_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_3_GPIO_Port,	.echo_pin = Echo_3_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_4_GPIO_Port,	.echo_pin = Echo_4_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_5_GPIO_Port,    .echo_pin = Echo_5_Pin,     .trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_6_GPIO_Port,    .echo_pin = Echo_6_Pin,     .trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_7_GPIO_Port,	.echo_pin = Echo_7_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_8_GPIO_Port,	.echo_pin = Echo_8_Pin, 	.trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
    { .gpiox = Echo_9_GPIO_Port,    .echo_pin = Echo_9_Pin,     .trig_us=0, .echo_us=0, .distance = 0, .distance_ms = 0, .stage = 0 },
};

#ifdef _RTOS_
//SemaphoreHandle_t _sr04p_mutex = NULL;
uint16_t _sr04p_interval_ms = SR04P_TIMEOUT_MS*2;       //距离评估的间隔时间（毫秒）
#endif

int16_t process_echo_int(uint16_t GPIO_Pin) {
	//根据GPIO_Pin找到SR04P序号
	int8_t index;
	for(index=0; index<SR04P_NUM; index++)
		if(GPIO_Pin == _sr04ps[index].echo_pin)
			break;
	if(index == SR04P_NUM)
		return -1;
    if(_sr04ps[index].stage == ECHO_STAGE_WAIT_TRIG) {
        //在echo的上升沿记录当前时间（微秒）
        _sr04ps[index].trig_us = get_us();
        _sr04ps[index].stage = ECHO_STAGE_WAIT_ECHO;
        return 0;
    } else if(_sr04ps[index].stage == ECHO_STAGE_WAIT_ECHO) {
        //在echo的下降沿计算距离
        _sr04ps[index].echo_us = get_us();
        _sr04ps[index].distance_ms = _sr04ps[index].echo_us/1000;
        int64_t us = _sr04ps[index].echo_us - _sr04ps[index].trig_us;
        _sr04ps[index].distance = 343 * us / 2000;
        _sr04ps[index].stage = ECHO_STAGE_END;
    }
    return 0;
}

void set_echo_interval(uint16_t interval_ms) {
#ifdef _RTOS_
    _sr04p_interval_ms = interval_ms;
#endif
}

uint16_t wait_echo_end(uint16_t index, uint32_t timeout_ms) {
    uint64_t stm = millis();
    while(millis() - stm < timeout_ms) {
        if(_sr04ps[index].stage == ECHO_STAGE_END)
            return _sr04ps[index].stage;
        //vTaskDelay(pdMS_TO_TICKS(1));
        HAL_Delay(1);
    }
    return _sr04ps[index].stage;
}

//获得距离值以及距离有效的时间戳
uint32_t get_echo_distance(uint16_t index, uint32_t *time_ms) {
    if(wait_echo_end(index, SR04P_TIMEOUT_MS) != ECHO_STAGE_END)
        return 0;
    //xSemaphoreTake(_sr04p_mutex, portMAX_DELAY);
    if(time_ms != NULL)
        *time_ms = _sr04ps[index].distance_ms;
    uint16_t distance = _sr04ps[index].distance;
    if(distance > SR04P_MAX_DISTANCE_MM)
        distance = 0;
    //xSemaphoreGive(_sr04p_mutex);
    return distance;
}

//获得全部距离值以及距离有效的时间戳
void get_echos_distance(uint32_t* distance_array, uint32_t *time_ms_array) {
    if(distance_array == NULL || time_ms_array == NULL)
        return;
    //xSemaphoreTake(_sr04p_mutex, portMAX_DELAY);
    for(uint16_t i = 0; i < SR04P_NUM; i++ ) {
        distance_array[i] = _sr04ps[i].distance;
        if(distance_array[i] > SR04P_MAX_DISTANCE_MM)
            distance_array[i] = 0;
        time_ms_array[i] = _sr04ps[i].distance_ms;
    }
    //xSemaphoreGive(_sr04p_mutex);
}

#ifdef _RTOS_
void task_echo(void *arg) {
    int index, busy;
    uint64_t stm;
    if(arg != NULL)
        printf("%s\n", (char*)arg);
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("task_echo: Begin ...\n");
    while(1) {
        //等待所有echo pin为低电平
        stm = millis();
        while(1) {
            busy = 0;
            for(index = 0; index < SR04P_NUM; index++)
                if(HAL_GPIO_ReadPin(_sr04ps[index].gpiox, _sr04ps[index].echo_pin) == GPIO_PIN_SET)
                    busy = index+1;
            if(!busy)
                break;
            if(millis() - stm > SR04P_TIMEOUT_MS)   //超时
                break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if(busy) {
            ; //printf("task_echo: Echo[%d] is busy!\n", busy-1);
        }

        //清除距离标记
        xSemaphoreTake(_sr04p_mutex, portMAX_DELAY);
        for(index=0; index<SR04P_NUM; index++) {
            _sr04ps[index].trig_us = 0;
            _sr04ps[index].distance_ms = 0;
            _sr04ps[index].stage = ECHO_STAGE_WAIT_TRIG;
        }
        //触发超声波
        HAL_GPIO_WritePin(_trig_gpiox, _trig_pin, GPIO_PIN_SET);
        //延迟10us
        int64_t stu = get_us();
        while(get_us() - stu <= 10)
        	;
        HAL_GPIO_WritePin(_trig_gpiox, _trig_pin, GPIO_PIN_RESET);
        //检查回声
        stm = millis();
        while(1) {
            busy = 0;
            for(index = 0; index < SR04P_NUM; index++)
                if(_sr04ps[index].stage != ECHO_STAGE_END)
                    busy = index+1;
            if(!busy)
                break;
            if(millis() - stm > SR04P_TIMEOUT_MS) //超过测试距离触发超时
                break;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        xSemaphoreGive(_sr04p_mutex);

        if(busy) {
        	/*
            printf("Echo[%d] Pin-%d: stage:%u, trig:%ld, echo:%ld, Time-out!\n", index,
                _sr04ps[busy-1].echo_pin, _sr04ps[busy-1].stage,
                _sr04ps[busy-1].trig_us, _sr04ps[busy-1].echo_us);
            */
        }

        //等待下一个测距周期
        vTaskDelay(pdMS_TO_TICKS(_sr04p_interval_ms));
    }
}

int16_t init_sr04p() {
    //printf("init_sr04p(): Initializing Echos ...\n");
    //初始化互斥锁
    _sr04p_mutex = xSemaphoreCreateMutex();

    //创建SR04P测距线程
    if(xTaskCreate(task_echo, "Task Echo", 256, NULL, 2, NULL) != pdPASS) {
    	printf("init_sr04p(): Failed to create task_echo!\n");
    	return -1;
    }

    return 0;
}

#else
int16_t init_sr04p() {
    return 0;
}

void process_echo() {
    int index, busy;
    uint64_t stm;

    //等待所有echo pin为低电平
    stm = millis();
    while(1) {
        busy = 0;
        for(index = 0; index < SR04P_NUM; index++)
            if(HAL_GPIO_ReadPin(_sr04ps[index].gpiox, _sr04ps[index].echo_pin) == GPIO_PIN_SET)
                busy = index+1;
        if(!busy)
            break;
        if(millis() - stm > SR04P_TIMEOUT_MS)   //超时
            break;
        HAL_Delay(1);
    }
    if(busy) {
        ; //printf("task_echo: Echo[%d] is busy!\n", busy-1);
    }

    //清除距离标记
    for(index=0; index<SR04P_NUM; index++) {
        _sr04ps[index].trig_us = 0;
        _sr04ps[index].distance_ms = 0;
        _sr04ps[index].stage = ECHO_STAGE_WAIT_TRIG;
    }
    //触发超声波
    HAL_GPIO_WritePin(_trig_gpiox, _trig_pin, GPIO_PIN_SET);
    //延迟10us
    int64_t stu = get_us();
    while(get_us() - stu <= 10)
        ;
    HAL_GPIO_WritePin(_trig_gpiox, _trig_pin, GPIO_PIN_RESET);
    //检查回声
    stm = millis();
    while(1) {
        busy = 0;
        for(index = 0; index < SR04P_NUM; index++)
            if(_sr04ps[index].stage != ECHO_STAGE_END)
                busy = index+1;
        if(!busy)
            break;
        if(millis() - stm > SR04P_TIMEOUT_MS) //超过测试距离触发超时
            break;
        HAL_Delay(1);
    }

    if(busy) {
        /*
        printf("Echo[%d] Pin-%d: stage:%u, trig:%ld, echo:%ld, Time-out!\n", index,
            _sr04ps[busy-1].echo_pin, _sr04ps[busy-1].stage,
            _sr04ps[busy-1].trig_us, _sr04ps[busy-1].echo_us);
        */
    }
}

static uint32_t _trigger_stm = 0;
void trigger_echo() {
    int index, busy;

    //等待所有echo pin为低电平
    _trigger_stm = millis();
    while(1) {
        busy = 0;
        for(index = 0; index < SR04P_NUM; index++)
            if(HAL_GPIO_ReadPin(_sr04ps[index].gpiox, _sr04ps[index].echo_pin) == GPIO_PIN_SET)
                busy = index+1;
        if(!busy)
            break;
        if(millis() - _trigger_stm > SR04P_TIMEOUT_MS)   //超时
            break;
        HAL_Delay(1);
    }
    if(busy) {
        ; //printf("task_echo: Echo[%d] is busy!\n", busy-1);
    }

    //清除距离标记
    for(index=0; index<SR04P_NUM; index++) {
        _sr04ps[index].trig_us = 0;
        _sr04ps[index].distance_ms = 0;
        _sr04ps[index].stage = ECHO_STAGE_WAIT_TRIG;
    }
    //触发超声波
    HAL_GPIO_WritePin(_trig_gpiox, _trig_pin, GPIO_PIN_SET);
    //延迟10us
    int64_t stu = get_us();
    while(get_us() - stu <= 10)
        ;
    HAL_GPIO_WritePin(_trig_gpiox, _trig_pin, GPIO_PIN_RESET);
    //更新超声波触发时间
    _trigger_stm = millis();
}

void wait_echo() {
    int index, busy;
    //检查回声
    while(1) {
        busy = 0;
        for(index = 0; index < SR04P_NUM; index++)
            if(_sr04ps[index].stage != ECHO_STAGE_END)
                busy = index+1;
        if(!busy)
            break;
        if(millis() - _trigger_stm > SR04P_TIMEOUT_MS) //超过测试距离触发超时
            break;
        HAL_Delay(1);
    }

    if(busy) {
        /*
        printf("Echo[%d] Pin-%d: stage:%u, trig:%ld, echo:%ld, Time-out!\n", index,
            _sr04ps[busy-1].echo_pin, _sr04ps[busy-1].stage,
            _sr04ps[busy-1].trig_us, _sr04ps[busy-1].echo_us);
        */
    }
}
#endif

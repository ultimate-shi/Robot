/*
 * motor.c
 *
 *  Created on: Jan 24, 2026
 *      Author: Luke 1281772125@qq.com
 */

#include <motor_servo.h>
#include "main.h"
#include "adc.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "nvs.h"
#include "motor_encoder.h"
#include "stm32f4xx_hal.h"

#define NVS_CONFIG_VERSION	0x71062403
#define MOTOR_ID_BEGIN 	0

#define MOTOR_ADC_GAP 	5		//绝对编码器临近极限角度时的保护角度
#define MOTOR_STUCK_MS	1000	//关节低速旋转时间保护

// 多段减速区（核心：按误差大小分3级减速）
#define ENCODER_SLOW_ZONE_1_RATIO   25 	// 远减速区相对于最终精度的倍数
#define MOTOR_SLOW_SPEED1_RATIO		70	// 远减速区的马达驱动功率相对于最大功率的比例

#define ENCODER_SLOW_ZONE_2_RATIO   20   // 中减速区相对于最终精度的倍数
#define MOTOR_SLOW_SPEED2_RATIO		30  // 中减速区马达驱动功率相对于最大功率的比例

#define ENCODER_SLOW_ZONE_3_RATIO   10  // 近减速区相对于最终精度的倍数

#define MOTOR_MAGIC		0x0624
#define MOTOR_CFG_MAGIC	0x0130

extern IWDG_HandleTypeDef hiwdg;
static int32_t _encoders[MOTOR_MAX_CHANNELS];
static motor_cfg_t _motor_cfgs[MOTOR_MAX_CHANNELS];
static motor_t _motors[MOTOR_MAX_CHANNELS];
static osThreadId _motor_pid_task;
static char _motor_log[MOTOR_LOG_SIZE];
static SemaphoreHandle_t _motor_log_mutex;
static int _motor_log_newline;

int _debug = 0;

void task_motor_monitor(const void *arg);
void motor_pid_init(motor_t *motor);
static void motor_pid_abs_inc(motor_t *motor, int report);
void task_motor_pid(const void *arg);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;


static motor_io_t _motor_ios[MOTOR_MAX_CHANNELS] = {
		{ &htim1, GPIOC, Motor1_PWM0_Pin, Motor1_PWM1_Pin, 0, 4 },		//LEG_RIGHT_FRONT
		{ &htim2, GPIOC, Motor2_PWM0_Pin, Motor2_PWM1_Pin, 1, 5 },		//LEG_RIGHT_REAR
		{ &htim3, GPIOC, Motor3_PWM0_Pin, Motor3_PWM1_Pin, 2, 6 },		//LEG_LEFT_FRONT
		{ &htim4, GPIOC, Motor4_PWM0_Pin, Motor4_PWM1_Pin, 3, 7 },		//LEG_LEFT_REAR
};

/*大腿控制器缺省配置：
*	J0 0 20 100 2000
*	P0 10 2 25 100 0
*	I0 5566 -1
*	I1 5566 -1
*	I2 5566 -1
* 	I3 5566 -1
* 	A0 975 1427 -45 30
* 	A1 975 1447 -30 45
* 	A2 975 1597 -30 45
*   A3 975 1727 -45 30
*/
/*车轮控制器缺省配置：
*	J4 1 10 100 1000
*	P4 35 10 30 100 0
*	I4 11600 -1
*	I5 11600 1
*	I6 11600 -1
* 	I7 11600 1
* 	A4 926 1650 -90 90
* 	A5 926 1650 -90 90
* 	A6 926 1650 -90 90
*   A7 926 1650 -90 90
*/
void motor_cfg_init(motor_cfg_t *mcfg) {
	mcfg->magic = MOTOR_CFG_MAGIC;
	mcfg->kp = 0.1f;
	mcfg->ki = 0.02f;
	mcfg->kd = 0.25f;
	mcfg->err_angle = 1.0f;			//误差控制为1.0度
	mcfg->min_rate = 20;			//速率低于15%时，电机失去扭力
	mcfg->max_rate = 100;			//限制电机的最大速度，用于小扭矩高精度的场景
	mcfg->ppr = 11 * 504;			//霍尔编码器每圈11个脉冲，减速比504
	mcfg->encoder_dir = -1;			//关节顺时针旋转时，编码器值增加
	mcfg->apa_mv = 9.7f; 			//实测与理论值不同 3300.0f / 333.3f; 	//SV01角度传感器的旋转范围是333度，满量程电压3.3V
	mcfg->gravity_angle_dir = 0;	//当关节顺时针旋转时指向重力方向
	mcfg->zero_angle_mv = 1650;		//当关节角度为0度时的SV01处于中间位置 1.65V
	mcfg->min_angle = -90;			//限制运动范围，逆时针90度
	mcfg->max_angle = 90;			//限制运动范围，顺时针90度
	mcfg->timeout_ms = 3000;		//关节驱动运行连续不超过4秒
	mcfg->mode = MOTOR_MODE_ABSnINC;	//缺省使用绝对编码器+增量编码器的角度位置模式
	mcfg->id = 0;					//设置电机ID
}

//把角度值转为编码器值
int32_t motor_get_encoder_by_angle(motor_t *motor, float angle) {
	int32_t count = motor->pulse_per_angle * angle + MOTOR_ENCODER_ABS_MIDDLE;
	return count;
}

//把编码器值转为角度值
float motor_get_angle_by_encoder(motor_t *motor, int32_t encoder) {
	float angle = (encoder - MOTOR_ENCODER_ABS_MIDDLE) / motor->pulse_per_angle;
	return angle;
}

//设置编码器当前绝对值
void motor_set_encoder_by_angle(motor_t *motor, float angle) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	motor_encoder_set(motor->encoder, motor_get_encoder_by_angle(motor, angle));
}

//检查允许的旋转方向
int motor_check_limit(motor_t * motor) {

	return motor->allowed_dir;
}

//在配置修改后重新更新马达的动态配置
void motor_update_param(motor_t *motor) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	motor_cfg_t *mcfg = motor->config;
	//静态数据
	motor->pulse_per_angle = (mcfg->ppr * 2 * mcfg->encoder_dir) / 360.0f;

	//PID参数初始化依赖Motor其他项目，因此必须放在初始化的最末尾
	motor_pid_init(motor);
	//旋转方向控制
	float angle = motor_get_angle(motor, NULL);
	if(angle <= motor->config->min_angle)
		motor->allowed_dir = 1;
	else if(angle >= motor->config->max_angle)
		motor->allowed_dir = -1;
	else
		motor->allowed_dir = 0;
}

//从NVS读取配置信息
int motor_load_cfg() {
	//初始化配置数据
	for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
		motor_cfg_init(&_motor_cfgs[i]);
		_motor_cfgs[i].id = i + MOTOR_ID_BEGIN;
	}
	//从NVS读取配置数据
	if(nvs_get_data(NVS_CONFIG_VERSION, (void*)_motor_cfgs, sizeof(_motor_cfgs)) != sizeof(_motor_cfgs)) {
		//没有配置数据
		return -1;
	}
	return 0;
}

// 0: Saved, -1: Failed
int motor_save_cfg() {
	int ret = nvs_set_data(NVS_CONFIG_VERSION, (void*)_motor_cfgs, sizeof(_motor_cfgs));
	return ret;
}

void motor_init() {
	motor_t * mp;
	_motor_log[0] = 'L';
	_motor_log[1] = 0;
	_motor_log_newline = 0;
	_motor_log_mutex = xSemaphoreCreateMutex();
	//读取配置
	motor_load_cfg();
	//依次生成每个关节的硬件参数
	for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
		motor_io_t *mio = &_motor_ios[i];
		mp = &_motors[i];
		mp->magic = MOTOR_MAGIC;
		mp->config = &_motor_cfgs[i];	//Motor其他项目依赖于config，因此必须最先初始化config
		mp->io = &_motor_ios[i];		//Motor I/O是静态初始化，与硬件端口相关
		mp->encoder =  motor_encoder_add(mio->htim_encoder);
		mp->pwm1 = gpio_pwm_add(mio->gpiox_pwm, mio->pwm_pin_a);
		mp->pwm2 = gpio_pwm_add(mio->gpiox_pwm,mio->pwm_pin_b);
		mp->adc_ch = mio->adc_ch;
		mp->target_encoder = INVALID_ENCODER_VALUE;		//目标编码器的值不能设置为0
		mp->target_angle = 0;
		mp->rate = 0;
		mp->last_encoder = 0;
		mp->spin_time_ms = 0;
		mp->stuck_time_ms = 0;
		mp->eps_f = 0.0f;
		mp->target_eps_f = 0.0f;
		mp->acs712_quite_voltage = 2.5f;
		mp->timeout_ms = mp->config->timeout_ms;
		mp->last_cmd = '.';
		//设置动态数据
		motor_update_param(mp);
	}

	//启动马达PID进程
	osThreadDef(motorPIDTask, task_motor_pid, osPriorityNormal, 0, 1024);
	_motor_pid_task = osThreadCreate(osThread(motorPIDTask), NULL);
}

//static char _pid_log[MOTOR_LOG_SIZE];
void append_motor_log(char *msg) {
	xSemaphoreTake(_motor_log_mutex, portMAX_DELAY);
	int len = strlen(msg);
	if(len + strlen(_motor_log) + 2>= MOTOR_LOG_SIZE) {
		xSemaphoreGive(_motor_log_mutex);
		return;
	}
	strcat(_motor_log, msg);
	if(msg[len-1] == '|' || msg[len-1] == '\n') {
		if(_debug) {
			if(msg[len-1] == '|')
				strcat(_motor_log, "\n");
			usb_send(_motor_log);
			_motor_log_newline = 0;
			_motor_log[0] = 'L';
			_motor_log[1] = 0;
		}
		_motor_log_newline++;
	}
	xSemaphoreGive(_motor_log_mutex);
}

int motor_get_log(char *res) {
	int ret = 0;
	xSemaphoreTake(_motor_log_mutex, portMAX_DELAY);
	if(_motor_log_newline) {
		strcpy(res, _motor_log);
		res[strlen(res)-1] = '\n';
		_motor_log[0] = 'L';
		_motor_log[1] = 0;
		ret = _motor_log_newline;
		_motor_log_newline = 0;
	}
	xSemaphoreGive(_motor_log_mutex);
	return ret;
}

void motor_reset_encoder() {
	for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
		if(_motors[i].config->mode == MOTOR_MODE_ABSnINC)
			motor_set_encoder_by_angle(&_motors[i], motor_get_angle(&_motors[i], NULL));
		else if(_motors[i].config->mode == MOTOR_MODE_INC) {
			motor_encoder_set(_motors[i].encoder, MOTOR_ENCODER_INC_MIDDLE);
			_motors[i].last_encoder = MOTOR_ENCODER_INC_MIDDLE;
		}
	}
}

motor_t * motor_get(uint16_t index) {
	if(index >= MOTOR_MAX_CHANNELS)
		return NULL;
	return  &_motors[index];
}

motor_t* motor_get_by_id(int16_t id) {
	motor_t * mp;
	for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
		mp = &_motors[i];
		if(mp->magic == MOTOR_MAGIC && mp->config->id == id)
			return mp;
	}
	return NULL;
}

//将关节角度转换为ADC对应的值，
//返回值超过范围时返回最小或最大值
uint16_t motor_angle_to_adc(motor_t *motor, float angle)
{
	motor_cfg_t * mcfg = motor->config;
    // 1. 计算ADC值变化量：角度差 × 每度ADC变化量
    // 规则：角度增大→ADC增大，角度减小→ADC减小（符合逆时针旋转ADC减少）
    float delta_adc_mv = angle * mcfg->apa_mv;

    // 2. 计算目标ADC值（参考ADC值 + 变化量）
    float target_voltage_mv = (float)mcfg->zero_angle_mv + delta_adc_mv;

    // 3. 四舍五入为整数（贴近ADC实际采样的整数特性）
    int target_adc_mv = (target_voltage_mv+0.5f);

    return adc_voltage_to_adc(target_adc_mv);
}

float motor_adc_to_angle(motor_t *motor, uint16_t adc_value)
{
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return 0.0f;
    // 1. 先将ADC值转换为电压值
    float adc_float_mv = (float)adc_get_voltage(adc_value);

    // 2. 逆向计算ADC电压差值：输入ADC电压值 - 0度的ADC电压值
    float delta_adc_mv = adc_float_mv - (float)motor->config->zero_angle_mv;

    // 3. 逆向计算角度差值：ADC差值 / 每度ADC变化量
    // 规则：ADC增大→角度增大，ADC减小→角度减小（与原函数完全逆向）
    float delta_angle = 0;
    if(motor->config->apa_mv)
    	delta_angle = delta_adc_mv / motor->config->apa_mv;
    else
    	delta_angle = 0;

    // 6. 返回浮点型角度值（保留精度）
    return delta_angle;
}

void motor_set_range(motor_t* motor, uint16_t zero_angle_mv, int angle_min, int angle_max) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	motor_stop_pid(motor);
	motor->config->zero_angle_mv = zero_angle_mv;
	motor->config->min_angle = angle_min;
	motor->config->max_angle = angle_max;
	//Set Encoder Counter
	float abs_angle = motor_get_angle(motor, NULL);
	motor_set_encoder_by_angle(motor, abs_angle);
	//更新静态数据
	motor_update_param(motor);
}

//adc_dir: 0 not change, -1/+1
//encoder_dir: 0 not change, -1/+1
void motor_set_dir(motor_t* motor, int gravity_dir, int encoder_dir) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	if(gravity_dir < -1 || gravity_dir > 1)
		return;
	if(encoder_dir < -1 || encoder_dir > 1)
		return;
	motor_stop_pid(motor);
	int modified = 0;
	if(gravity_dir && motor->config->gravity_angle_dir != gravity_dir) {
		motor->config->gravity_angle_dir = gravity_dir;
		modified++;
	}
	if(encoder_dir && motor->config->encoder_dir != encoder_dir) {
		motor->config->encoder_dir = encoder_dir;
		modified++;
	}
	//Set Encoder Counter
	if(modified) {
		float abs_angle = motor_get_angle(motor, NULL);
		motor_set_encoder_by_angle(motor, abs_angle);
		//更新静态数据
		motor_update_param(motor);
	}
}

float motor_get_dps(motor_t *motor) {
    return motor->eps_f / motor->config->ppr * 6.0f;
}

void motor_spin(motor_t* motor, int rate, int time_ms) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	if(rate > 0) {
		if(rate > 100)
			rate = 100;
		motor->rate = rate;
		gpio_pwm_set(motor->pwm1, 0);
		gpio_pwm_set(motor->pwm2, rate);
	}
	else if(rate < 0) {
		if(rate < -100)
			rate = -100;
		motor->rate = rate;
		gpio_pwm_set(motor->pwm2, 0);
		gpio_pwm_set(motor->pwm1, -rate);
	}
	else {
		motor->rate = 0;
		gpio_pwm_set(motor->pwm2, 100);
		gpio_pwm_set(motor->pwm1, 100);
		motor->spin_time_ms = 0;
		motor->stuck_time_ms = 0;
		return;
	}
	if(time_ms > 0) {
		motor->timeout_ms = time_ms;
		motor->spin_time_ms = HAL_GetTick();
	} else if(motor->spin_time_ms == 0) {
		motor->spin_time_ms = HAL_GetTick();
	}
}

void motor_release(motor_t* motor) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	gpio_pwm_set(motor->pwm1, 0);
	gpio_pwm_set(motor->pwm2, 0);
	motor->spin_time_ms = 0;
	motor->rate = 0;
}

void motor_break(motor_t* motor) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	gpio_pwm_set(motor->pwm1, 100);
	gpio_pwm_set(motor->pwm2, 100);
	motor->spin_time_ms = 0;
	motor->rate = 0;
}

float motor_get_angle(motor_t * motor, float *encoder_angle) {
	if(encoder_angle != NULL) {
		*encoder_angle = motor_get_angle_by_encoder(motor, motor_encoder_get(motor->encoder));
		/*
		float angle = 0.0f;
		angle = motor_encoder_get(motor->encoder);
		angle -= MOTOR_ENCODER_MIDDLE_VALUE;
		*encoder_angle = angle / motor->pulse_per_angle;
		*/
	}
	uint16_t adc = adc_read(motor->adc_ch);
	return motor_adc_to_angle(motor, adc);
}

int motor_set_pid(motor_t *motor, float kp, float ki, float kd) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return -1;
	motor_stop_pid(motor);
	motor->config->kp = kp;
	motor->config->ki = ki;
	motor->config->kd = kd;
	motor_update_param(motor);
	return 0;
}

void motor_pid_load_cfg(motor_t *motor) {
    if (motor == NULL || motor->magic != MOTOR_MAGIC)
    	return;
    float angle = motor_get_angle(motor, NULL);
    motor_cfg_t * mcfg = motor->config;
    pid_ctl_t * pid = &motor->pid;
    pid->kp = mcfg->kp;
    pid->ki = mcfg->ki;
    pid->kd = mcfg->kd;
    pid->max_rate = mcfg->max_rate;
    if(pid->max_rate > 100) {
    	printf("!Max Rate:%d > 100, Reset to 100!\n", pid->max_rate);
    	pid->max_rate = 100;
    }
    pid->min_rate = mcfg->min_rate;
    if(pid->min_rate < 5) {
    	printf("!Min Rate:%d < 5, Reset to 5!\n", pid->min_rate);
    	pid->min_rate = 5;
    }

    int32_t err_encoder = 20;
	if(mcfg->mode == MOTOR_MODE_ABSnINC) {
		err_encoder = (float)mcfg->ppr * mcfg->err_angle / 360.0f + 0.5f; //PID精度控制
	} 
	else if(mcfg->mode == MOTOR_MODE_INC) {
		err_encoder = mcfg->ppr / 360 ; //PID精度控制到1度以内
	}
    pid->err_encoder = err_encoder;
	//printf("!M:%hd err_encoder:%ld\n",motor->config->id,err_encoder);

    if(mcfg->gravity_angle_dir > 0) {
    	if(motor->target_angle > angle) { //需要重力补偿
    		pid->max_rate = mcfg->max_rate;
    		pid->min_rate = mcfg->min_rate;
    	}
    	else {	//反向重力补偿
    		pid->max_rate = mcfg->max_rate / 3;
    		pid->min_rate = mcfg->min_rate;
    	}
    }
    else if(mcfg->gravity_angle_dir < 0) {
    	if(motor->target_angle < angle) { //需要重力补偿
    		pid->max_rate = mcfg->max_rate;
    		pid->min_rate = mcfg->min_rate;
    	}
    	else {	//反向重力补偿
    		pid->max_rate = mcfg->max_rate / 3;
    		pid->min_rate = mcfg->min_rate;
    	}
    }

    pid->ENCODER_SLOW_ZONE_1 = ENCODER_SLOW_ZONE_1_RATIO * err_encoder;			//远端减速范围
    pid->SPIN_SLOW_RATE_1 = pid->max_rate * MOTOR_SLOW_SPEED1_RATIO / 100;	//远端转动功率

    pid->ENCODER_SLOW_ZONE_2 = ENCODER_SLOW_ZONE_2_RATIO * err_encoder; 		//中段减速范围
    pid->SPIN_SLOW_RATE_2 = pid->max_rate * MOTOR_SLOW_SPEED2_RATIO / 100;	//中段转动功率

    pid->ENCODER_SLOW_ZONE_3 = ENCODER_SLOW_ZONE_3_RATIO * err_encoder; 		//近端减速范围
    pid->SPIN_SLOW_RATE_3 = pid->min_rate;		//近端转动功率
}

/**
 * @brief 初始化电机PID参数（适配STM32F103）
 * @param motor 电机结构体指针
 */
void motor_pid_init(motor_t *motor)
{
    if (motor == NULL || motor->magic != MOTOR_MAGIC)
    	return;
    pid_ctl_t * pid = &motor->pid;

    // 终极调参：以“无过冲”为核心，牺牲少量响应速度换稳定性
    pid->err_limit = 200;
    pid->integral_limit = 10; // 积分限幅极低，杜绝积分超调

    // 常量初始化
    motor_pid_load_cfg(motor);

    // 状态初始化
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
    pid->arrived_ms = 0;

    // 禁止PID控制
    pid->mutex = xSemaphoreCreateMutex();
    pid->enabled = 0;

    pid->final_report = 1;
}

static void _motor_stop_pid(motor_t *motor) {
	motor->pid.enabled = 0;
	motor_break(motor);
}

void motor_stop_pid(motor_t *motor) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return;
	pid_ctl_t * pid = &motor->pid;
	if (xSemaphoreTake(pid->mutex, 50) == pdPASS) {
		_motor_stop_pid(motor);
		xSemaphoreGive(pid->mutex);
	}
	else {
		//BUG? Anyway, STOP the motor!
		_motor_stop_pid(motor);
	}
}

//双编码器的位置控制PID算法
static char pid_float_p[16], pid_float_i[16], pid_float_d[16];
char pid_log[256];
static void motor_pid_abs_inc(motor_t *motor, int report)
{
	char log[32];
    if (motor == NULL || motor->magic != MOTOR_MAGIC) {
    	printf("!pid: Not an MOTOR!\n");
    	return;
    }
    if (!motor->pid.enabled || motor->target_encoder == INVALID_ENCODER_VALUE) {
    	return;
    }

    pid_ctl_t * pid = &motor->pid;
    //int pre_break = 0;
    // 1. 获取当前编码器值 + 计算误差
    int32_t curr_encoder = motor_encoder_get(motor->encoder);
    int32_t err;
	int err_dir;
	err = motor->target_encoder - curr_encoder;
	if(err > 0) 
		err_dir = 1;
	else 
		err_dir = -1;

    if(report && _debug) {
		sprintf(pid_log,"E%ld ",err);
    	append_motor_log(pid_log);
	}
    // --------------------------
    // 核心逻辑1：到达目标 → 强制刹车+清零所有PID项
    // --------------------------
	int32_t err_abs = abs(err);
    if (err_abs <= pid->err_encoder) {
        // 彻底清零PID累积项，杜绝回弹
        pid->integral = 0.0f;
        pid->last_err = 0.0f;
        //pre_break = 1;
        motor_break(motor);
        // 惯性抵消（仅用极小速度，避免反向冲）,反而会造成反冲
        if(fabs(motor->eps_f) < 100.0f) {
            motor_break(motor);
            pid->arrived_ms = HAL_GetTick();
            _motor_stop_pid(motor);
            if(report && _debug) {
            	append_motor_log("BRK ");
			}
            return;
        }
    }

    // --------------------------
    // 核心逻辑2：多段减速 + 动态PID参数（减速过程明显）
    // --------------------------

    // 2. 步骤1：根据误差大小，动态限制最大速度（核心减速逻辑）
    int max_speed = pid->max_rate; // 默认最大速度100
    if (err_abs < pid->ENCODER_SLOW_ZONE_3) {
    	max_speed = pid->SPIN_SLOW_RATE_3;  // 近减速区：限15（接近爬行）
    	//pre_break = 1;
    	if(report && _debug)
    		append_motor_log("CL ");
    }
    else if (err_abs < pid->ENCODER_SLOW_ZONE_2) {
    	max_speed = pid->SPIN_SLOW_RATE_2;  // 中减速区：限30
    	if(report && _debug)
    		append_motor_log("NR ");
    }
    else if (err_abs < pid->ENCODER_SLOW_ZONE_1) {
    	max_speed = pid->SPIN_SLOW_RATE_1; // 远减速区：限80
    	if(report && _debug)
    		append_motor_log("MD ");
    }
    else
    	if(report && _debug)
    		append_motor_log("FR ");
    int min_speed = pid->min_rate;
    if(max_speed <= min_speed)
    	max_speed = min_speed+5;

    // 3. 步骤2：动态调整PID参数（越近目标，KP/KI越小，KD越大）
    float dynamic_kp = motor->pid.kp;
    float dynamic_ki = motor->pid.ki;
    float dynamic_kd = motor->pid.kd;

    if (err_abs < pid->ENCODER_SLOW_ZONE_1) {
        dynamic_kp = motor->pid.kp * 0.8f;
        dynamic_kd = motor->pid.kd * 1.5f;
    }
    /*
    if (err_abs < pid->ENCODER_SLOW_ZONE_2) {
        dynamic_kp = motor->pid.kp * 0.4f; // 中减速区：KP降为40%
        dynamic_ki = motor->pid.ki * 0.2f; // KI降为20%
        dynamic_kd = motor->pid.kd * 2.0f; // KD提升50%
    }

    if (err_abs < pid->ENCODER_SLOW_ZONE_3) {
        dynamic_kp = motor->pid.kp * 0.2f; // 近减速区：KP仅20%
        dynamic_ki = 0.0f;                 // 近目标：完全关闭积分
        dynamic_kd = motor->pid.kd * 4.0f; // KD翻倍，最强减速预判
    }
    */
    if(report && _debug) {
		float_to_string(dynamic_kp, pid_float_p, 2);
		float_to_string(dynamic_ki, pid_float_i, 2);
		float_to_string(dynamic_kd, pid_float_d, 2);
    	sprintf(pid_log, "P%s I%s D%s ", pid_float_p, pid_float_i, pid_float_d);
		append_motor_log(pid_log);
	}

    // 4. 步骤3：误差限幅（随减速区缩小）
    /*
    int32_t err_limit = motor->pid.err_limit;
    if (err_abs < pid->ENCODER_SLOW_ZONE_2)
    	err_limit = motor->pid.err_limit / 2;
    if (err_abs < pid->ENCODER_SLOW_ZONE_3)
    	err_limit = motor->pid.err_limit / 4;
    if (err > err_limit)
    	err = err_limit;
    else if (err < -err_limit)
    	err = -err_limit;
	*/

    // 5. 步骤4：PID核心计算（强化微分的减速效果）
    // 比例项（动态KP）
    float p_out = dynamic_kp * err;

    // 积分项（仅远区累积，中/近区衰减，近区关闭）
    float i_out = 0.0f;
#ifdef _KI_SEGMENT_
    if (err_abs > pid->ENCODER_SLOW_ZONE_2) {
        motor->pid.integral += dynamic_ki * err;
        // 积分限幅（随减速区缩小）
        float integral_limit = motor->pid.integral_limit;
        if (err_abs < pid->ENCODER_SLOW_ZONE_1)
        	integral_limit *= 0.5f;
        if (motor->pid.integral > integral_limit)
            motor->pid.integral = integral_limit;
        else if (motor->pid.integral < -integral_limit)
            motor->pid.integral = -integral_limit;
        i_out = motor->pid.integral;
    } else if (err_abs > pid->ENCODER_SLOW_ZONE_3) {
        motor->pid.integral *= 0.8f; // 中减速区：积分缓慢衰减
        i_out = motor->pid.integral;
    }
#else
    // 1. 标准积分累积：无论误差大小，均累积（这是标准PID的核心特征）
    motor->pid.integral += dynamic_ki * err;

    // 2. 标准积分限幅：固定上限，不随误差区间动态调整（工程必备的通用优化）
    float integral_limit = motor->pid.integral_limit; // 固定限幅值，无0.5倍衰减
    // 限幅逻辑和原代码一致，仅去掉误差区间的条件
    if (motor->pid.integral > integral_limit) {
        motor->pid.integral = integral_limit;
    } else if (motor->pid.integral < -integral_limit) {
        motor->pid.integral = -integral_limit;
    }

    // 3. 积分项输出：直接使用累积值（无分段条件）
    i_out = motor->pid.integral;
#endif

    // 微分项（动态KD + 误差变化率放大，强化减速预判）
    float delta_err = err - motor->pid.last_err;
    float d_out = dynamic_kd * delta_err * 1.5f; // 微分效果放大1.5倍
    motor->pid.last_err = err;

    // 6. PID总输出
    float pid_out = p_out + i_out + d_out;

    // 7. 速度处理（强制限幅+规避无效区）
    int speed = (int)roundf(pid_out);
    if(report && _debug) {
		sprintf(log, "Po%d%% ",speed);
		append_motor_log(log);
	}
    // 强制按减速区限幅（核心：减速过程明显）
    if (abs(speed) > max_speed) {
    	speed = max_speed * err_dir;
    }
    else if (abs(speed) < min_speed) {
    	speed = min_speed * err_dir;
    }

    // 8. 驱动电机（减速过程可通过speed值直观看到）
	motor_spin(motor, speed * motor->config->encoder_dir, 0);
	if(report && _debug) {
		sprintf(pid_log, "R%d%% ", speed * motor->config->encoder_dir);
		append_motor_log(pid_log);
	}
}

//单编码器的速度PID控制
#define MOTOR_PID_SPEED_SCALE 10
static void motor_pid_inc_speed(motor_t *motor, int report)
{
    if (motor == NULL || motor->magic != MOTOR_MAGIC || (!motor->pid.enabled)) {
    	return;
    }
	pid_ctl_t *pid = &motor->pid;
	int rate = 0;
	int dir = motor->config->encoder_dir;

    //计算速度差
    float err_eps = (motor->target_eps_f + motor->eps_f*dir)/MOTOR_PID_SPEED_SCALE; // 计算转速误差
    if(report && _debug) {
		sprintf(pid_log, "T%d-S%d*%d E%hd ", 
			(int)motor->target_eps_f, (int)motor->eps_f, dir, (int16_t)err_eps*MOTOR_PID_SPEED_SCALE);
		append_motor_log(pid_log);
	}

    //转速到达目标不再进行PID干涉
	float err_abs = fabs(err_eps);
    if (err_abs <= pid->err_encoder) {
		if(report && _debug) {
			append_motor_log("MATCH ");
		}
    }
	//驱动功率限制
	int max_rate = pid->max_rate; // 默认最大速度100
	int min_rate = pid->min_rate;
	if(max_rate <= min_rate)
		max_rate = min_rate+5;

    //如果当前速度为0，则表明是启动状态，使用速度比例的软起动
	if(fabs(motor->eps_f) < 0.001f) {
		rate = (int)roundf(motor->target_eps_f / 80.0f);
		pid->integral = (float)rate; // 让 PID 从软启动的功率开始累加
		if(report && _debug) {
			sprintf(pid_log, "Rst%d%% ", rate);
			append_motor_log(pid_log);
		}
	}
	else {
		//PID参数
		/*
		if(report && _debug) {
			float_to_string(pid->kp, pid_float_p, 2);
			float_to_string(pid->ki, pid_float_i, 2);
			float_to_string(pid->kd, pid_float_d, 2);
			sprintf(pid_log, "P=%s I=%s D=%s ", pid_float_p, pid_float_i, pid_float_d);
			append_motor_log(pid_log);
		}
		*/
		// 1. 比例项
		float p_out = pid->kp * err_eps;

		// 2. 积分项：累加误差！
		pid->integral += pid->ki * err_eps;

		// --- 关键：积分限幅（防止飞车） ---
		if (pid->integral > max_rate) pid->integral = max_rate;
		if (pid->integral < -max_rate) pid->integral = -max_rate;

		// 3. 微分项
		float d_out = pid->kd * (err_eps - pid->last_err);
		pid->last_err = err_eps;

		// 4. PID总输出
		float pid_out = p_out + pid->integral + d_out;
		float_to_string(pid_out, pid_float_d, 2);
		if(report && _debug) {
			sprintf(pid_log, "Po%s%% ", pid_float_d);
			append_motor_log(pid_log);
		}
		// 驱动功率处理
    	rate = (int)roundf(pid_out);
	}
    // 限幅
	if (rate > max_rate) 
		rate = max_rate;
	if (rate < -max_rate) 
		rate = -max_rate;

	if(fabs(motor->target_eps_f) < 0.001f) {
		pid->integral = 0; // 目标为0时，重置积分
		rate = 0;
	}	

    // 驱动电机
	motor_spin(motor, rate * motor->config->encoder_dir, 0);
	if(report && _debug) {
		sprintf(pid_log, "R%d%% ", rate * motor->config->encoder_dir);
		append_motor_log(pid_log);
	}
}

static int _motor_allow_spin(motor_t *motor, float target_angle) {
	float angle = motor_get_angle(motor, NULL);
	if(motor->allowed_dir) {
		if(motor->allowed_dir == 1 && target_angle <= angle)
			return 0;
		if(motor->allowed_dir == -1 && target_angle >= angle)
			return 0;
	}
	return 1;
}

static void _motor_start_pid(motor_t *motor, int dur_ms) {
	motor_pid_load_cfg(motor);
	motor->pid.enabled = 1;
	motor->pid.final_report = 0;
	motor->spin_time_ms = 0;
	motor->stuck_time_ms = 0;
	motor->pid.arrived_ms = 0;
	if(dur_ms > 0)
		motor->timeout_ms = dur_ms;
	else
		motor->timeout_ms = motor->config->timeout_ms;
}

// 以位置环开启PID模式，如果当前是速度环，则先停止
int motor_start_pid_by_encoder(motor_t *motor, int32_t target_encoder, float ratio, int dur_ms) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return -1;
	char msg[32];
	if(dur_ms > 30*1000) {
		sprintf(pid_log, "T%d Tmo:%d > 30000 ms\n", motor->config->id, dur_ms);
		append_motor_log(pid_log);
		return -1;
	}
	if (xSemaphoreTake(motor->pid.mutex, portMAX_DELAY) == pdPASS) {
		int32_t encoder_err = target_encoder - motor_encoder_get(motor->encoder);
		if(motor->config->mode == MOTOR_MODE_INC) { //单编码器模式下，要检查是否处于速度环
			if(motor->pid.enabled && fabs(motor->target_eps_f) >= 1.0f ) {
				_motor_stop_pid(motor);
				motor->target_eps_f = 0.0f;
			}
			motor->target_angle = INVALID_ANGLE_VALUE;  //没有绝对编码器
			if(_debug) {
				float_to_string(ratio, msg, 2);
				sprintf(pid_log,"T%d Eerr:%ld Rat:%s Tmo:%d\n",motor->config->id, encoder_err, msg, dur_ms);
				append_motor_log(pid_log);
			}
		}
		else if(motor->config->mode == MOTOR_MODE_ABSnINC) { 
			//带有绝对编码器时，要计算绝对角度位置
			motor->target_angle = motor_get_angle_by_encoder(motor, target_encoder);
		}
		//设置位置环目标编码器值
		motor->target_encoder = target_encoder;		
		motor->ratio = ratio;	 
		if(encoder_err < 0)
			motor->target_dir = -1;
		else
			motor->target_dir = 1;
		//每个轮子的速度控制按照最大值和比例
		motor->target_eps_f = - motor->target_dir * MAX_ENCODER_SPEED * motor->config->encoder_dir * ratio; 	
		if(!motor->pid.enabled) {
			//启动PID控制
			_motor_start_pid(motor,  dur_ms);
		}
		else {
			//如果马达已启动，则只是复位超时
			motor->spin_time_ms = HAL_GetTick();
			motor->timeout_ms = dur_ms;
		}
		xSemaphoreGive(motor->pid.mutex);
		return 0;
	}
	return -4;
}

int motor_start_pid_by_angle(motor_t *motor, float target_angle) {
	if(motor->config->mode != MOTOR_MODE_ABSnINC)
		return -1;
	if(target_angle < motor->config->min_angle ||
			target_angle > motor->config->max_angle)
		return -2;
	if(!_motor_allow_spin(motor, target_angle))
		return -3;
	int32_t target_encoder = motor_get_encoder_by_angle(motor, target_angle);
	int32_t curr_encoder = motor_encoder_get(motor->encoder);
	float delta_angle = abs(curr_encoder - target_encoder);
	delta_angle = delta_angle / motor->pulse_per_angle;
	if(delta_angle > abs(motor->config->max_angle - motor->config->min_angle) ) {
		return -4;
	}
	return motor_start_pid_by_encoder(motor, target_encoder, 1.0f, motor->config->timeout_ms);
}

//速度环只适合单编码器模式
int motor_start_pid_by_speed(motor_t *motor, int16_t target_eps, int dur_ms) {
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return -1;
	if(motor->config->mode != MOTOR_MODE_INC)
		return -2;
	if (xSemaphoreTake(motor->pid.mutex, portMAX_DELAY) == pdPASS) {
		//在设置速度环之前检查当前是否处于位置环
		if(motor->pid.enabled && motor->target_encoder != INVALID_ENCODER_VALUE) {
			//如果当前处于位置环，则先停止PID控制
			_motor_stop_pid(motor);
		}
		motor->target_encoder = INVALID_ENCODER_VALUE;
		motor->target_eps_f = target_eps;
		if(!motor->pid.enabled) {
			//启动PID控制
			_motor_start_pid(motor, dur_ms);
		}
		else {
			//如果马达已启动，则只是复位超时
			motor->spin_time_ms = HAL_GetTick();
			if(dur_ms > 0 )
				motor->timeout_ms = dur_ms;
			else
				motor->timeout_ms = motor->config->timeout_ms;
		}
		xSemaphoreGive(motor->pid.mutex);
		return 0;
	}
	return -2;
}

//返回指定马达的PID执行进度
int motor_get_pid_status(motor_t *motor, int32_t *ticks_err) { 
	if(motor == NULL || motor->magic != MOTOR_MAGIC)
		return -1;
	if(ticks_err != NULL)
		*ticks_err = motor->last_encoder - motor->target_encoder;
	if(motor->pid.arrived_ms > 0 || motor->pid.final_report)
		return 100;	//抵达目标
	if(!motor->pid.enabled) {
		//没有抵达终点
		return -100; //没有抵达目标
	}
	uint32_t est_ms = HAL_GetTick() - motor->spin_time_ms;
	if(est_ms >= motor->timeout_ms)
		return 99;
	return (int)(100.0f * est_ms / motor->timeout_ms - 1.0f);
}

//读取使用ACS712-5A传感器获得的马达电流
float motor_get_current(motor_t *motor,int16_t debug) {
    //计算分压前的电压 0V ~ 5V
    float delta_volt = (float)adc_read_mv(motor->io->acs712_ch)/1000.0f - motor->acs712_quite_voltage;
    //计算ACS712电流
    float current = delta_volt / 0.185;
    if(debug) {
        //printf(".get_current(ADC[%u]): %dmV %dmA\n", motor->io->acs712_ch, (int)(delta_volt*1000+0.5), (int)(current*1000+0.5));
		;
    }
    return current;
}

#define _DYNAMIC_QUITE_VOLTAGE_
void motor_reset_acs712() {
	motor_t *motor;
	for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
		motor = &_motors[i];
#ifdef _DYNAMIC_QUITE_VOLTAGE_
		motor->acs712_quite_voltage = (float)adc_read_mv(motor->io->acs712_ch)/1000.0f;
		//printf(".motor_reset_acs712(): Quite[%d]: %dmV\n", i, (int)(motor->acs712_quite_voltage*1000.f + 0.5));
		if(fabs(motor->acs712_quite_voltage-2.5f) > 0.3f) {
			//printf("!motor_reset_acs712(): Quite[%d] voltage is out of range! Use default voltage!\n", i);
			motor->acs712_quite_voltage = 2.22f; //因为1N4448的存在，实测静止时电压约为2.22V，而非理论值2.5V
		}
#else
		motor->acs712_quite_voltage = 2.22f;	//因为1N4448的存在，实测静止时电压约为2.22V，而非理论值2.5V
#endif
	}
}

//检测马达堵转
void motor_detect_stuck(motor_t *motor, uint32_t ms) {
	if(motor->rate) {
		//当马达被驱动时，进行堵转检测
		if(fabs(motor->eps_f) < 0.001f) {
			if(motor->stuck_time_ms == 0) {
				//对于刚刚检测到的堵转，只记录堵转开始时间
				motor->stuck_time_ms = ms;
				//printf(".%lu:M%d Stuck!!!\n", osKernelSysTick(), motor->config->id);
			}
			else if(ms - motor->stuck_time_ms >= MOTOR_STUCK_MS) {
				//如果堵转超过限制，则执行停止马达操作
				sprintf(pid_log, ".%lu M%d STOP of Stuck!\n",osKernelSysTick(), motor->config->id);
				append_motor_log(pid_log);
				if(motor->pid.enabled)
					motor_stop_pid(motor);
				else
					motor_break(motor);
			}
		}
		else {
			//解除堵转检测
			motor->stuck_time_ms = 0;
			//printf(".%lu:M[%d]: Stuck Clear\n", osKernelSysTick(), i);
		}
	}
}

//处理增量编码器的速度控制
void motor_process_inc_speed(motor_t *motor, int report) { 
	//执行PID控制
	if (xSemaphoreTake(motor->pid.mutex, portMAX_DELAY) == pdPASS) {
		if(motor->pid.enabled) {
			if(report && _debug) {
				float current = motor_get_current(motor,0);
				sprintf(pid_log, ".%lu M%d S%d %dmA ", osKernelSysTick(), motor->config->id, (int)motor->eps_f, (int)(current*1000));
				append_motor_log(pid_log);
			}
			motor_pid_inc_speed(motor, report);
			if(report && _debug) {
				append_motor_log("\n");
			}
		}
		xSemaphoreGive(motor->pid.mutex);
	}
	else {
		//printf(".%lu:M%d Mutex err! \n", osKernelSysTick(), motor->config->id);
		;
	}
}

//处理双编码器马达的位置控制
void motor_process_abs_inc_position(motor_t *motor, int report) {
	int out_of_range;
	//检查马达位置
	out_of_range = 0;
	motor_cfg_t * mcfg = motor->config;
	float enc_angle = 0;
	float abs_angle = motor_get_angle(motor, &enc_angle);
	//检查绝对编码器与增量编码器的角度差异，过大则可能是编码器异常
	if( fabs(enc_angle - abs_angle) > 10.0f) {
		//TODO! Something went wrong!
		if(_debug && motor->rate != 0) {
			sprintf(pid_log, ".%lu M%d Enc Err Inc=%d Abs=%d |", osKernelSysTick(), motor->config->id, (int)enc_angle, (int)abs_angle);
			append_motor_log(pid_log);
			motor_stop_pid(motor);
		}
		return;
	}
	//检查是否超过范围，并设置允许的旋转方向
	if(enc_angle <= mcfg->min_angle) {
		motor->allowed_dir = 1;
		out_of_range = 1;
	}
	else if(enc_angle >= mcfg->max_angle) {
		motor->allowed_dir = -1;
		out_of_range = 1;
	}
	else
		motor->allowed_dir = 0;
	//执行PID控制
	if (xSemaphoreTake(motor->pid.mutex, portMAX_DELAY) == pdPASS) {
		if(motor->pid.enabled) {
			if(report && _debug) {
				float current = motor_get_current(motor,0);
				sprintf(pid_log, ".%lu M%d S%d C%dmA ", osKernelSysTick(), motor->config->id, (int)motor->eps_f, (int)(current*1000));
				append_motor_log(pid_log);
			}
			motor_pid_abs_inc(motor, report);
			if(report && _debug) {
				append_motor_log("|");
			}
		}
		else {
			if(motor->pid.final_report == 0) {
				float current = motor_get_current(motor,0);
				sprintf(pid_log, ".%lu M%d S%d C%dmA ", osKernelSysTick(), motor->config->id, (int)motor->eps_f, (int)(current*1000));
				append_motor_log(pid_log);
				int32_t curr_encoder = motor_encoder_get(motor->encoder);
				int32_t err = motor->target_encoder - curr_encoder;
				sprintf(pid_log, "ERR=%ld ARRVIED |",err);
				append_motor_log(pid_log);
				motor->pid.final_report = 1;
			}
			if(out_of_range) {
				if(motor->allowed_dir < 0 && motor->rate > 5) {
					motor_break(motor);
					sprintf(pid_log, ".%lu:M%d Out of range! |", osKernelSysTick(), motor->config->id);
					append_motor_log(pid_log);
					motor_stop_pid(motor);
				}
				else if(motor->allowed_dir > 0 && motor->rate < -5) {
					motor_break(motor);
					sprintf(pid_log, ".%lu:M%d Out of range! |", osKernelSysTick(), motor->config->id);
					append_motor_log(pid_log);
					motor_stop_pid(motor);
				}
			}
		}
		xSemaphoreGive(motor->pid.mutex);
	}
	else {
		//printf(".%lu:M%d Mutex err! \n", osKernelSysTick(), motor->config->id);
		;
	}
}

#ifdef _PID_POSITION_
//单编码器位置环控制
static void motor_pid_inc(motor_t *motor, int report)
{
    if (motor == NULL || motor->magic != MOTOR_MAGIC) {
    	return;
	}
	if ((!motor->pid.enabled) || (motor->target_encoder == INVALID_ENCODER_VALUE)) {
		return;
	}
	pid_ctl_t *pid = &motor->pid;
	if (fabs(motor->target_eps_f) < 1.0f) {
		sprintf(pid_log, "M%d Ts=0 STOP\n",motor->config->id);
		append_motor_log(pid_log);
		motor_break(motor);
		pid->arrived_ms = HAL_GetTick();
		_motor_stop_pid(motor);		
		return;
	}
	int rate = 0;
	int dir = motor->config->encoder_dir;

    //计算速度差
    float err_eps = (motor->target_eps_f + motor->eps_f*dir)/MOTOR_PID_SPEED_SCALE; // 计算转速误差
	//计算距离误差
	int32_t curr_encoder = motor_encoder_get(motor->encoder);
	int32_t err = motor->target_encoder - curr_encoder;
    if(report && _debug) {
		sprintf(pid_log, "%d=>%d(%hd) D%d ", (int)motor->eps_f,
			(int)motor->target_eps_f, (int16_t)err_eps*MOTOR_PID_SPEED_SCALE, dir);
		append_motor_log(pid_log);
	}
	int arrived = 0;
	if(motor->target_dir > 0) {
		//目标编码器值应该大于当前编码器值
		if(curr_encoder + pid->err_encoder >= motor->target_encoder) 
			arrived = 1;
	}
	else {
		//目标编码器值应该小于当前编码器值
		if(curr_encoder + pid->err_encoder <= motor->target_encoder)
			arrived = 1;
	}
	if( arrived ) {
		//到达目的地，停车
        // 彻底清零PID累积项，杜绝回弹
        pid->integral = 0.0f;
        pid->last_err = 0.0f;
		pid->arrived_ms = HAL_GetTick();
        motor_break(motor);
		_motor_stop_pid(motor);
		sprintf(pid_log, "BRK<%ld ", pid->err_encoder);
		if(report) {
			sprintf(pid_log, "STOP@Et%ld ", err);
			append_motor_log(pid_log);
		}
		else {
			sprintf(pid_log, "M%d STOP@Et%ld/%ld Ee%ld\n", motor->config->id, curr_encoder, motor->target_encoder, err);
			append_motor_log(pid_log);
		}
		return;
	}
	else if(report && _debug) {
		sprintf(pid_log,"Ee%ld>%ld ",err, pid->err_encoder);
		append_motor_log(pid_log);
	}

    //转速到达目标不再进行PID干涉
	float err_abs = fabs(err_eps);
    if (err_abs <= pid->err_encoder) {
		if(report && _debug) {
			append_motor_log("MCHs ");
		}
    }
	//驱动功率限制
	int max_rate = pid->max_rate; // 默认最大速度100
	int min_rate = pid->min_rate;
	if(max_rate <= min_rate)
		max_rate = min_rate+5;

    //如果当前速度为0，则表明是启动状态，使用速度比例的软起动
	if(fabs(motor->eps_f) < 0.001f) {
		rate = (int)roundf(motor->target_eps_f / 80.0f);
		if(rate < motor->pid.min_rate)
			rate = motor->pid.min_rate;
		pid->integral = (float)rate; // 让 PID 从软启动的功率开始累加
		if(report && _debug) {
			sprintf(pid_log, "Rst%d%% ", rate);
			append_motor_log(pid_log);
		}
	}
	else {
		//PID参数
		/*
		if(report) {
			float_to_string(pid->kp, pid_float_p, 2); float_to_string(pid->ki, pid_float_i, 2); float_to_string(pid->kd, pid_float_d, 2);
			sprintf(pid_log, "P=%s I=%s D=%s ", pid_float_p, pid_float_i, pid_float_d);
			append_motor_log(pid_log);
		}
		*/
		// 1. 比例项
		float p_out = pid->kp * err_eps;
		// 2. 积分项：累加误差！
		pid->integral += pid->ki * err_eps;
		// 积分限幅（防止飞车
		if (pid->integral > max_rate) pid->integral = max_rate;
		if (pid->integral < -max_rate) pid->integral = -max_rate;
		// 3. 微分项
		float d_out = pid->kd * (err_eps - pid->last_err);
		pid->last_err = err_eps;
		// 4. PID总输出
		float pid_out = p_out + pid->integral + d_out;
		float_to_string(pid_out, pid_float_d, 2);
		if(report && _debug) {
			sprintf(pid_log, "Po%s%% ", pid_float_d);
			append_motor_log(pid_log);
		}
		// 驱动功率处理
    	rate = (int)roundf(pid_out);
	}
    // 限幅
	if (rate > max_rate) 
		rate = max_rate;
	if (rate < -max_rate) 
		rate = -max_rate;
	if(fabs(motor->target_eps_f) < 0.001f) {
		pid->integral = 0; // 目标为0时，重置积分
		rate = 0;
	}	
    // 驱动电机
	rate *= motor->target_dir * motor->config->encoder_dir;
	motor_spin(motor, rate, 0);
	if(report && _debug) {
		sprintf(pid_log, "Dt%d R%d%% ", motor->target_dir, rate );
		append_motor_log(pid_log);
	}
}


//处理单编码器马达的位置PID控制
void motor_process_inc_position(motor_t *motor, int report) {
	//执行PID控制
	if (xSemaphoreTake(motor->pid.mutex, portMAX_DELAY) == pdPASS) {
		if(motor->pid.enabled) {
			if(report && _debug) {
				//float current = motor_get_current(motor,0);
				sprintf(pid_log, "%lu M%d ", osKernelSysTick(), motor->config->id);
				append_motor_log(pid_log);
			}
			motor_pid_inc(motor, report);
			if(report && _debug) {
				append_motor_log("|");
			}
		}
		else {
			if(motor->pid.final_report == 0) {
				int32_t curr_encoder = motor_encoder_get(motor->encoder);
				int32_t err = motor->target_encoder - curr_encoder;
				sprintf(pid_log, ".%lu M%d S%d Ee%ld STOP\n", osKernelSysTick(), motor->config->id, (int)motor->eps_f, err);
				append_motor_log(pid_log);
				motor->pid.final_report = 1;
			}
		}
		xSemaphoreGive(motor->pid.mutex);
	}
	else {
		//printf(".%lu:M%d Mutex err! \n", osKernelSysTick(), motor->config->id);
		;
	}
}
#endif

//检查是否同步到达指定位置
void process_position_sync(int report) {
	motor_t *motor;
	int valid_cnt = 0;
	int arrived_cnt = 0;
	int stop_cnt = 0;
	uint32_t arrived_ms = 0;
	
	//检查马达一致性
	for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
		motor = &_motors[i];
		pid_ctl_t *pid = &motor->pid;
		if(motor->config->mode != MOTOR_MODE_INC)
			return;			//只限增量模式
		if(motor->pid.enabled == 0)
			stop_cnt++;		//统计电机伺服是否停止
		if(motor->target_encoder != INVALID_ENCODER_VALUE) {
			valid_cnt++;	//所有电机必须都有目标位置
			//检查是否已经抵达指定位置
			int32_t curr_encoder = motor_encoder_get(motor->encoder);
			int32_t err = motor->target_encoder - curr_encoder;
			if(abs(err) <= pid->err_encoder) {
				pid->integral = 0.0f;
        		pid->last_err = 0.0f;
				pid->arrived_ms = HAL_GetTick();
				motor_stop_pid(motor);
				if(report) {
					sprintf(pid_log, "STOP@Er%ld ", err);
					append_motor_log(pid_log);
				}
				else {
					sprintf(pid_log, "M%d STOP@Enc%ld/%ld Er%ld\n", motor->config->id, curr_encoder, motor->target_encoder, err);
					append_motor_log(pid_log);
				}
			}
			//记录最早抵达的时间点
			if(pid->arrived_ms > 0) {
				arrived_cnt++;	//统计电机伺服是否已经抵达
				if(arrived_ms == 0 || pid->arrived_ms < arrived_ms)
					arrived_ms = pid->arrived_ms;
			} 
		}
	}
	if(	valid_cnt < MOTOR_MAX_CHANNELS 
		|| arrived_cnt == MOTOR_MAX_CHANNELS 
		|| stop_cnt == MOTOR_MAX_CHANNELS)
		return;
	//如果距离第一个抵达位置的时间过去500ms，则全体停机以防拖拽
	if(arrived_ms > 0) {
		if(_debug) {
			sprintf(pid_log, "Sync: 1st ARV:%lums, ARV_CNT:%d\n", arrived_ms, arrived_cnt);
			append_motor_log(pid_log);
		}
		if( (HAL_GetTick() - arrived_ms >= SYNC_TIMEOUT_MS) 
		|| arrived_cnt > MOTOR_MAX_CHANNELS/2) {
			for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
				motor = &_motors[i];
				if(motor->pid.enabled) {
					int32_t err = motor->target_encoder - motor_encoder_get(motor->encoder) ;
					sprintf(pid_log, "M%d SYNC TMO STOP@Et%ld\n", motor->config->id, err);
					append_motor_log(pid_log);
					motor_stop_pid(motor);
				}
			}
		}
	}
}

//马达PID控制线程
void task_motor_pid(const void *arg) {
    motor_t *motor;
    TickType_t last_tick = 0, last_ms = 0;
    int report = 0;

    while(1) {
    	//获取当前时间
		TickType_t tick = osKernelSysTick();
		uint32_t ms = HAL_GetTick();
		//计算速度以及超时检查
		if(tick - last_tick >= 50) {
			//刷新看门狗
			HAL_IWDG_Refresh(&hiwdg);
			//检测堵转和超时
			report = 1;
			last_tick = tick;
			for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
				motor = &_motors[i];
				motor_detect_stuck(motor, ms);
				//检测超时
				uint32_t dur = ms - motor->spin_time_ms;
				if(motor->spin_time_ms && ( dur > motor->timeout_ms) ) {
					sprintf(pid_log, "!%lu:M%d Timeout=%lums!\n", osKernelSysTick(), motor->config->id, dur);
					append_motor_log(pid_log);
					if(motor->pid.enabled)
						motor_stop_pid(motor);
					else
						motor_release(motor);
				}
			}
		} 
		else
			report = 0;

 		//处理马达的速度控制
		float est = ms - last_ms;
		if(est >= 20.0f) {
			//printf(".%lu:est=%dms\n", osKernelSysTick(), (int)(est));
			est = est / 1000.0f; //转换为秒
			last_ms = ms;
			//更新速度和编码器
			for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
				motor = &_motors[i];
				_encoders[i] = motor_encoder_get(motor->encoder);
				float raw_eps = 0;
				if(motor->last_encoder != 0 && _encoders[i] != motor->last_encoder) {
					raw_eps = (float)(_encoders[i] - motor->last_encoder)/est + 0.5f;
				}
				motor->eps_f = (raw_eps * 0.8f) + (motor->eps_f * 0.2f);
				motor->last_encoder = _encoders[i];
			}
			//检查同步抵达情况
			process_position_sync(report);
			//执行PID算法
			for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
				motor = &_motors[i];
				if(motor->config->mode == MOTOR_MODE_INC) {
#ifdef _PID_POSITION_
					if(motor->target_encoder != INVALID_ENCODER_VALUE)
						motor_process_inc_position(motor, report);
					else 
#endif
					if(fabs(motor->target_eps_f) > 0.001f )
						motor_process_inc_speed(motor, report);
				}
			}
		}
		//处理马达的位置控制
		for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
			motor = &_motors[i];
			if(motor->config->mode == MOTOR_MODE_ABSnINC)
				motor_process_abs_inc_position(motor, report);
		}
		//延迟
		osDelay(10);
    }
}

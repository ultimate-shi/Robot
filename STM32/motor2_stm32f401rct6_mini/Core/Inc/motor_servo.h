/*
 * 用途：定义 DMC 电机接线、持久化参数和运行时控制接口。
 * 使用方法：板级接线极性写入 motor_io_t，速度环从该结构读取反馈极性。
 */

#ifndef INC_MOTOR_GPIO_H_
#define INC_MOTOR_GPIO_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "gpio_pwm.h"
#include "adc.h"
#include "motor_encoder.h"
#include <stdint.h>

#define MOTOR_LOG_SIZE		1024
#define MOTOR_MAX_CHANNELS	4

#define MOTOR_MODE_ABSnINC 	0	//角度控制(绝对编码器+增量编码器)
#define MOTOR_MODE_INC 		1	//速度控制(仅使用增量编码器, 无绝对编码器)
//#define MOTOR_MODE_ABS  	2	//角度控制(仅使用绝对编码器，无增量编码器)

#define INVALID_ENCODER_VALUE	INT32_MIN
#define INVALID_ANGLE_VALUE	 	-999.0f

//TODO! 改为动态配置
#define MAX_ENCODER_SPEED           2500	//根据实际测试而来，
#define SYNC_TIMEOUT_MS		500				//第一个因抵达位置而停机时间到强制停机时间的间隔

typedef struct motor_io_s {
	TIM_HandleTypeDef *htim_encoder;
	GPIO_TypeDef* gpiox_pwm;
	uint16_t pwm_pin_a;
	uint16_t pwm_pin_b;
	uint16_t adc_ch;
	uint16_t acs712_ch;
	int8_t speed_feedback_sign; // 0 按旧规则 -encoder_dir；非零为该通道实测的原始速度反馈符号
} motor_io_t;

typedef struct {
	uint16_t magic;		//配置的MAGIC值
	float kp;
	float ki;
	float kd;
	float err_angle;	//PID控制的误差角度
	int	min_rate;		//电机允许的最低速率阈值绝对值
	int max_rate;		//电机允许的最大速率阈值绝对值
	uint16_t ppr;		//关节每转一圈编码器发出的脉冲数pulses per revolution
	int encoder_dir;	//当关节角度增加（顺时针旋转）时，编码器的增量方向
	float apa_mv;			//绝对编码器ADC的每度电压变化量(mV)，对于3.3V的SV01角度传感器来说是10.0
	int zero_angle_mv;	//绝对编码器0度时电压参考值(mV)
	int gravity_angle_dir; //关节旋转角度的重力方向，0 水平旋转， 1 顺时针方向为重力方向（须补偿）  -1 逆时针方向为反重力方向
	int min_angle;		//末端允许的最小角度
	int max_angle;		//末端允许的最大角度
	int timeout_ms;		//关节连续运转超时保护时间（毫秒）
	int mode;			//电机模式，0 角度控制(绝对编码器+增量编码器)，1 速度控制(无绝对编码器), 2 角度控制(仅绝对编码器)	
	int id;				//电机ID, 可配置
} motor_cfg_t;

typedef struct {
    float kp;                // 比例系数
    float ki;                // 积分系数
    float kd;                // 微分系数
    int min_rate;			 // 最小速率
    int max_rate;			 // 最大速率
    float integral;          // 积分项
    float last_err;          // 上一次误差（微分项用）
    float err_limit;         // 误差限幅（防止积分饱和）
    float integral_limit;    // 积分限幅
    int32_t err_encoder;	 // 允许的误差
    int32_t ENCODER_SLOW_ZONE_1;	// 远减速区
    int SPIN_SLOW_RATE_1;
    int32_t ENCODER_SLOW_ZONE_2;	// 中减速区
    int SPIN_SLOW_RATE_2;
    int32_t ENCODER_SLOW_ZONE_3;	// 近减速区
    int SPIN_SLOW_RATE_3;
    uint32_t arrived_ms;      // 到达标记
    SemaphoreHandle_t mutex;
    uint8_t enabled;		 // 是否由PID控制
    int final_report;		 // 最终报告
} pid_ctl_t;

typedef struct motor_s {
	uint16_t magic;
	gpio_pwm_t *pwm1;
	gpio_pwm_t *pwm2;
	int16_t adc_ch;
	motor_encoder_t *encoder;
	motor_cfg_t *config;
	motor_io_t *io;
	pid_ctl_t pid;
	//运行时统计参数
	int32_t target_encoder;		//目标编码器值，用于位置PID控制（增量编码器）
	int target_dir;				//目标编码器值与当前编码器值的方向，+1 目标编码器值比当前大， -1 目标编码器值比当前小
	float target_angle;			//目标对应的角度，用于位置PID控制（绝对值编码器）
	float target_eps_f;			//目标转速(编码/秒)，用于速度PID控制
	int allowed_dir;			//当前允许的方向，0 任意方向， 1 顺时针方向， -1 逆时针方向
	int rate;					//当前驱动功率 -100 ~ 100
	uint32_t spin_time_ms;		//旋转的时间计数器
	int32_t last_encoder;		//上次编码器值,仅用于转速计算
    uint32_t stuck_time_ms; 	//低速运行超时保护计数器
	float eps_f;				//当前实际转速(编码/秒)	
	char last_cmd;				//上一次执行的命令
	//差速控制
	float ratio;				//与最快转速的转速比例
	//静态参数
	float pulse_per_angle;
	float acs712_quite_voltage;	//ACS712静态电压（无电流时的电压值）
	int timeout_ms;				//当前连续运行超时保护时间（毫秒）	
} motor_t;

void motor_init();
void motor_reset_acs712();
void motor_reset_encoder();
void motor_set_range(motor_t* motor, uint16_t zero_angle_mv, int angle_min, int angle_max);
void motor_spin(motor_t* motor, int rate, int time_ms);
void motor_break(motor_t* motor);
void motor_release(motor_t* motor);
float motor_get_angle(motor_t * motor, float *encoder_angle);
void motor_stop_pid(motor_t *motor);
int motor_start_pid_by_angle(motor_t *motor, float target_angle);
int motor_start_pid_by_encoder(motor_t *motor, int32_t target_encoder, float ratio, int dur_ms);
int motor_start_pid_by_speed(motor_t *motor, int16_t target_eps, int dur_ms);
int motor_save_cfg();
int motor_load_cfg();
uint16_t motor_angle_to_adc(motor_t *motor, float angle);
float motor_adc_to_angle(motor_t *motor, uint16_t adc_value);
int motor_set_pid(motor_t *motor, float kp, float ki, float kd);
motor_t * motor_get(uint16_t index);
void motor_set_dir(motor_t* motor, int gravity_dir, int encoder_dir);
int32_t motor_get_encoder_by_angle(motor_t *motor, float angle);
float motor_get_angle_by_encoder(motor_t *motor, int32_t encoder);
void motor_set_encoder_by_angle(motor_t *motor, float angle) ;
float motor_get_current(motor_t *motor,int16_t debug);
void motor_update_param(motor_t *motor);
motor_t * motor_get_by_id(int16_t id);
float motor_get_dps(motor_t *motor);
int motor_get_log(char *res);
int motor_get_pid_status(motor_t *motor, int32_t *ticks_err);
#endif /* INC_MOTOR_GPIO_H_ */

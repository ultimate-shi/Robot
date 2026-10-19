/*
 * ec11.c
 *
 *  Created on: Jan 18, 2026
 *      Author: Luke 1281772125@qq.com
 */

#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "motor_encoder.h"
#include "motor_servo.h"


extern int _debug;
static char _cmd_resp[1024];                      //命令返回缓存
extern char pid_log[];
void append_motor_log(char *msg);

//生成命令返回内容
void send_cmd_err(char cmd, int16_t id, char* resp) {
    sprintf(_cmd_resp, "%c%d %s\n", cmd, id, resp);
    usb_send(_cmd_resp);
}

//2位定点数转int16
float i16_f(int16_t value) {
	return (float)value/100.0f;
}

int16_t f_i16(float value) {
	return (int16_t)(value*100.0f+0.5f);
}

void process_R(char *buffer, uint32_t len) {
    int16_t target_angle;
    motor_t *motor = NULL;
    float angle, dps;
    int ret = 0, n;
    int16_t id, current;
    if(buffer[1] >= '0' && buffer[1] <= '9') {
        //单通道命令
        n = sscanf(buffer, "R%hd %hd", &id, &target_angle);
        if(n <= 0) {
            send_cmd_err(buffer[0], -1, "R<id> [<目标角度f_int16>]!");
            return;
        }
        motor = motor_get_by_id(id);
        if(motor == NULL) {
            send_cmd_err(buffer[0], id, "Motor not found!");
            return;
        }
        motor->last_cmd = buffer[0];
        if(n == 2) {
            //执行绝对角度位置控制 
            ret = motor_start_pid_by_angle(motor, i16_f(target_angle));
        }
        //读取电机状态
        motor_get_angle(motor, &angle);
        current = motor_get_current(motor,0) * 1000.0f;
        dps = motor_get_dps(motor);
        sprintf(_cmd_resp, "R%d %hd %hd %hd %d\n", id, 
            f_i16(angle), f_i16(dps), current, ret);
        usb_send(_cmd_resp);
        return;
    }
    //多通道命令
    int16_t tag[MOTOR_MAX_CHANNELS];
    n = sscanf(buffer, "R %hd %hd %hd %hd", &tag[0], &tag[1], &tag[2], &tag[3]);
    sprintf(_cmd_resp, "%c ", buffer[0]);
    char res[64];
    //发送命令
    for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
        motor = motor_get(i);
        if(motor == NULL) {
            send_cmd_err(buffer[0], i, "Motor not found!");
            return;
        }
        if(n == MOTOR_MAX_CHANNELS && tag[i] != 0x7FFF) {
            motor->last_cmd = buffer[0];
            ret = motor_start_pid_by_angle(motor, i16_f(tag[i]));
        }
        //读取电机状态
        motor_get_angle(motor, &angle);
        current = motor_get_current(motor,0) * 1000.0f;
        dps = motor_get_dps(motor);
        sprintf(res, "%hd %hd %hd ", f_i16(angle), f_i16(dps), current);
        strcat(_cmd_resp, res);
    }
    strcat(_cmd_resp, "\n");
    usb_send(_cmd_resp);
}

void process_S(char *buffer, uint32_t len) {
    int16_t target_speed, id, current, dur_ms;
    int32_t encoder;
    int ret = 0, n;
    motor_t *motor = NULL;
    if(buffer[1] >= '0' && buffer[1] <= '9') {
        n = sscanf(buffer, "S%hd %hd %hd", &id, &target_speed, &dur_ms);
        if(n <= 0) {
            send_cmd_err(buffer[0], -1, "S<id> [<目标速度(编码/秒)>] [<运行时间(毫秒)>]!");
            return;
        }
        motor = motor_get_by_id(id);
        if(motor == NULL) {
            send_cmd_err(buffer[0], id, "Motor not found!");
            return;
        }
        motor->last_cmd = buffer[0];
        if(n == 2) {
            if(target_speed == 0) {
                //刹车
                motor_stop_pid(motor);
            }
            else {
                //执行速度控制
                ret = motor_start_pid_by_speed(motor, target_speed, 0);
            }
        }
        else if(n == 3) { 
            if(target_speed == 0) {
                //刹车
                motor_stop_pid(motor);
            }
            else {
                //执行带时间限制的速度控制
                ret = motor_start_pid_by_speed(motor, target_speed, dur_ms);
            }
        }
        //读取电机状态
        current = motor_get_current(motor,0) * 1000.0f;
        encoder = motor_encoder_get(motor->encoder);
        sprintf(_cmd_resp, "S%hd %hd %lX %hd %d\n", id, 
            (int16_t)motor->eps_f, encoder, current, ret);
        usb_send(_cmd_resp);
        return;
    }
    //多通道命令
    int16_t tag[MOTOR_MAX_CHANNELS];
    dur_ms = 0;
    n = sscanf(buffer, "S %hd %hd %hd %hd %hd", &tag[0], &tag[1], &tag[2], &tag[3], &dur_ms);
    sprintf(_cmd_resp, "%c ", buffer[0]);
    char res[64];
    for(int i=0; i<MOTOR_MAX_CHANNELS; i++) { 
        motor = motor_get(i);
        if(motor == NULL) {
            send_cmd_err(buffer[0], i, "Motor not found!");
            return;
        }
        if(n >= MOTOR_MAX_CHANNELS  && tag[i] != 0x7FFF) {
            motor->last_cmd = buffer[0];
            if(tag[0] == 0 ) 
                //刹车
                motor_stop_pid(motor);
            else 
                ret = motor_start_pid_by_speed(motor, tag[i], dur_ms);
        }
        //读取电机状态
        current = motor_get_current(motor,0) * 1000.0f;
        encoder = motor_encoder_get(motor->encoder);
        sprintf(res, "%hd %08lX %hd ", (int16_t)motor->eps_f, encoder, current);
        strcat(_cmd_resp, res);
    }
    strcat(_cmd_resp, "\n");
    usb_send(_cmd_resp);
}

void process_T(char *buffer, uint32_t len) { 
    int32_t target_encoder;
    int16_t id;
    int32_t encoder;
    int ret = 0, n;
    motor_t *motor = NULL;
    if(buffer[1] >= '0' && buffer[1] <= '9') { 
        n = sscanf(buffer, "T%hd %lX", &id, &target_encoder);
        if(n <= 0) {
            send_cmd_err(buffer[0], -1, "T<id> [<目标编码值>]!");
            return;
        }
        motor = motor_get_by_id(id);
        if(motor == NULL) {
            send_cmd_err(buffer[0], id, "Motor not found!");
            return;
        }
        if(motor->config->mode != MOTOR_MODE_INC) {
            send_cmd_err(buffer[0], id, "Motor is not in INC mode!");
            return;
        }
        motor->last_cmd = buffer[0];
        if(n == 2) { 
            //检查目标位置与当前位置距离,不能超过1圈
            if(abs(target_encoder - motor_encoder_get(motor->encoder)) > motor->config->ppr) {
                sprintf(_cmd_resp,"T%hd Target encoder value should be within %d of current value!\n", id, motor->config->ppr);
                usb_send(_cmd_resp);
                return;
            }
            //执行编码值控制
            ret = motor_start_pid_by_encoder(motor, target_encoder, 1.0f, motor->config->timeout_ms);
        }
        //读取电机状态
        int32_t curr_ma = motor_get_current(motor,0) * 1000.0f;
        encoder = motor_encoder_get(motor->encoder);
        sprintf(_cmd_resp, "T%hd %hd %lX %ld %d\n", id, 
            (int16_t)motor->eps_f, encoder, curr_ma, ret);
        usb_send(_cmd_resp);
        return;
    }
    //多通道命令
    int32_t tag[MOTOR_MAX_CHANNELS];
    sprintf(_cmd_resp, "%c ", buffer[0]);
    int rel = 0;
    char res[64];
    if(buffer[1] == ' ')
        n = sscanf(buffer, "T %lX %lX %lX %lX", &tag[0], &tag[1], &tag[2], &tag[3]);
    else if(buffer[1] == 'r') { //相对值
        n = sscanf(buffer, "Tr %ld %ld %ld %ld", &tag[0], &tag[1], &tag[2], &tag[3]);
        rel = 1;
    }
    else
        n = 0;
    
    if(n < MOTOR_MAX_CHANNELS) {
        //状态查询命令
        int prog = 100;
        for(int i=0; i<MOTOR_MAX_CHANNELS; i++) { 
            motor = motor_get(i);
            int p = motor_get_pid_status(motor, &tag[i]);
            if(p < prog)
                prog = p; //以最慢的进度作为总体进度
        }
        sprintf(_cmd_resp, "%c %d %ld %ld %ld %ld \n", buffer[0], prog, tag[0], tag[1], tag[2], tag[3]);
        usb_send(_cmd_resp);
        return;
    }
    if(rel) {
        for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
            motor = motor_get(i);
            int32_t encoder = motor_encoder_get(motor->encoder);
            sprintf(pid_log,"Tr M%d E%ld + D%ld = %ld\n", i, encoder, tag[i], encoder+tag[i]);
            append_motor_log(pid_log);
            tag[i] += encoder;
        }
    }
    float ratios[MOTOR_MAX_CHANNELS];
    int32_t max_dist = 0;
    int32_t current[MOTOR_MAX_CHANNELS];
    //找出最大编码距离
    for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
        ratios[i] = 1.0f;
        motor = motor_get(i);
        if(motor == NULL) {
            send_cmd_err(buffer[0], i, "Motor not found!");
            return;
        }
        if(motor->config->mode != MOTOR_MODE_INC) {
            send_cmd_err(buffer[0], i, "Motor is not in INC mode!");
            return;
        }
        //找出最大编码距离
        current[i] = motor_encoder_get(motor->encoder);
        int32_t dist = abs(tag[i] - current[i]);
        if(dist > max_dist)
            max_dist = dist;
    }
    //计算每个轮子的转速系数和预测的运行时间
    float tmo_s = 0;
    for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
        motor = motor_get(i);
        int32_t dist = abs(tag[i] - current[i]);
        ratios[i] = (float)dist / (float)max_dist;
        //找到运行时间最慢的
        float tmo = (float)(dist) / ((float)MAX_ENCODER_SPEED * ratios[i]) * 1.5f;
        if(tmo > tmo_s)
            tmo_s = tmo;
    }
    int dur_ms = (int)(tmo_s * 1000.0f);
    if(dur_ms < 200)    //低于200ms时，马达尚未启动完成
        dur_ms = 200;
    if(dur_ms > 30000) {
        send_cmd_err(buffer[0], -1, "Duration time too long!");
        return;
    }
    
    //启动马达
    for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
        motor = motor_get(i);
        if(motor == NULL) {
            send_cmd_err(buffer[0], i, "Motor not found!");
            return;
        }
        if(n == MOTOR_MAX_CHANNELS) {
            motor->last_cmd = buffer[0];
            ret = motor_start_pid_by_encoder(motor, tag[i], ratios[i], dur_ms);
        }
        //读取电机状态
        int32_t curr_ma = motor_get_current(motor,0) * 1000.0f;
        encoder = motor_encoder_get(motor->encoder);
        sprintf(res, "%hd %lX %ld ", (int16_t)motor->eps_f, encoder, curr_ma);
        strcat(_cmd_resp, res);
    }
    sprintf(res,"%d\n", dur_ms);
    strcat(_cmd_resp, "\n");
    usb_send(_cmd_resp);
}

void process_D(char *buffer, uint32_t len) {
    int rate, ms = 1000;
    motor_t *motor = NULL;
    int n;
    int16_t id, current;
    if(buffer[1] >= '0' && buffer[1] <= '9') {
        n = sscanf(buffer, "D%hd %d %d", &id, &rate, &ms);
        if(n <= 0) {
            send_cmd_err(buffer[0], -1, "D<id> [<电机PWM占空比(+/- 0~100%%)>] [<电机转动时间(毫秒)>]!");
            return;
        }
        motor = motor_get_by_id(id);
        if(motor == NULL) {
            send_cmd_err(buffer[0], id, "Motor not found!");
            return;
        }
        if(n >= 2) {
            if(motor->pid.enabled) {
                if(rate != 0) {
                    send_cmd_err(buffer[0], id, "Motor is busy!");
                    return;
                }
                motor_stop_pid(motor);
            }
            else {
                motor->last_cmd = buffer[0];
                if(n >= 2) {
                    motor->spin_time_ms = 0;
                    motor->stuck_time_ms = 0;
                    motor_spin(motor, rate, ms);
                }
            }
        }
        //读取编码器位置信息
        uint16_t adc = adc_read_mv(motor->adc_ch);
        int32_t encoder = motor_encoder_get(motor->encoder);
        current = motor_get_current(motor,0) * 1000.0f;
        sprintf(_cmd_resp, "D%d %u %lX %hd\n", id, 
            adc, encoder, current);
        usb_send(_cmd_resp);
        return;
    }
    //多通道状态查询
    sprintf(_cmd_resp,"D ");
    char res[64];
    for(uint16_t i=0; i<MOTOR_MAX_CHANNELS; i++) {
        motor = motor_get(i);
        if(motor == NULL) {
            send_cmd_err(buffer[0], i, "Motor not found!");
            return;
        }
        //读取编码器位置信息
        uint16_t adc = adc_read_mv(motor->adc_ch);
        int32_t encoder = motor_encoder_get(motor->encoder);
        current = motor_get_current(motor,0) * 1000.0f;
        sprintf(res, "%u %ld %hd ", adc, encoder, current);
        strcat(_cmd_resp, res);
    }
    strcat(_cmd_resp, "\n");
    usb_send(_cmd_resp);
    return;
}

//处理上位机命令
void process_command(char *buffer, uint32_t len) {
    int n, ret;
    int16_t id;
    motor_t *motor = NULL;

    switch(buffer[0]) {
        //双编码器直流减速电机命令:R<id> [<目标角度 f_int16>] 
        case 'R':   
            process_R(buffer, len);
            return;
        case 'S':   //单增量编码器直流减速电机命令:S<id> [<目标速度(编码/秒)int16>]
            process_S(buffer, len);
            return;
        case 'T':   //单增量编码器直流减速电机命令:T<id> [<目标编码值int32>]
            process_T(buffer, len);
            return;
        case 'D':   //单步运行命令:D<id> [<电机PWM占空比(+/- 0~100%)int16>] [<电机转动时间(毫秒)int16>]
            process_D(buffer, len);
            return;
        case 'I':   //增量编码器配置命令:I<id> [<每圈脉冲数int16> <顺时针旋转时编码器数值增量方向int16>]
            if(buffer[1] == 'r') { //复位增量编码器到中间值 
                motor_reset_encoder();
                sprintf(_cmd_resp, "Ir %ld\n", MOTOR_ENCODER_INC_MIDDLE);
                usb_send(_cmd_resp);
                return;
            }
            int16_t ppr, dir;
            n = sscanf(buffer, "I%hd %hd %hd", &id, &ppr, &dir);
            if(n == 0) {
                send_cmd_err(buffer[0], -1, "I<id> [<每圈脉冲数>] [<顺时针旋转时编码器数值增量方向>]!");
                return;
            }
            motor = motor_get_by_id(id);
            if(motor == NULL) {
                send_cmd_err(buffer[0], id, "Motor not found!");
                return;
            }
            if(n == 3) {
                if(ppr <= 0) {
                    send_cmd_err(buffer[0], id, "Invalid ppr!");
                    motor->last_cmd = 'i';
                    return;
                }
                if(dir != 1 && dir != -1) {
                    send_cmd_err(buffer[0], id, "Invalid dir!");
                    motor->last_cmd = 'i';
                    return;
                }
                motor->last_cmd = buffer[0];
                //执行配置
                motor->config->ppr = ppr;
                motor_set_dir(motor, 0, dir);   
            }
            sprintf(_cmd_resp, "I%d %hu %d\n", id, motor->config->ppr, motor->config->encoder_dir);
            usb_send(_cmd_resp);
            return;
        case 'A':   //绝对编码器配置命令:A<id> [<角度每变化1度电压增量(mV)f_int16>] [<零点电压值(mV)int16>] [<最小限位角度int16>] [<最大限位角度int16>]
            int16_t mv, zero_mv, min_angle, max_angle;
            n = sscanf(buffer, "A%hd %hd %hd %hd %hd", &id, &mv, &zero_mv, &min_angle, &max_angle);
            if(n == 0) {
                send_cmd_err(buffer[0], -1, "A<id> [<每度电压增量(mV)f_int16>] [<零点电压值(mV)>] [<最小限位角度>] [<最大限位角度>]!");
                return;
            }
            motor = motor_get_by_id(id);
            if(motor == NULL) {
                send_cmd_err(buffer[0], id, "Motor not found!");
                return;
            }
            if(n == 5) {
                if(mv <= 0) {
                    send_cmd_err(buffer[0], id, "Invalid mv!");
                    motor->last_cmd = 'a';
                    return;
                }
                if(zero_mv < 0) {
                    send_cmd_err(buffer[0], id, "Invalid zero_mv!");
                    motor->last_cmd = 'a';
                    return;
                }
                if(min_angle < -180 || max_angle >= 180 || min_angle >= max_angle) {
                    send_cmd_err(buffer[0], id, "Invalid angle limits!");
                    motor->last_cmd = 'a';
                    return;
                }
                motor->last_cmd = buffer[0];
                //执行配置
                motor->config->apa_mv = i16_f(mv) ;
                motor_set_range(motor, zero_mv, min_angle, max_angle);
            }
            sprintf(_cmd_resp, "A%d %hd %hd %hd %hd\n", id, f_i16(motor->config->apa_mv), 
                motor->config->zero_angle_mv, motor->config->min_angle, motor->config->max_angle);
            usb_send(_cmd_resp);
            return;
        case 'J':   //关节配置命令:J<id> [工作模式int16] [<电机驱动最低PWM占空比int16>] [<电机驱动最高PWM占空比int16>] [每条命令连续运行最长时间(毫秒)int16]
            int16_t mode, min_rate, max_rate, timeout_ms;
            n = sscanf(buffer, "J%hd %hd %hd %hd %hd", &id, &mode, &min_rate, &max_rate, &timeout_ms);
            if(n == 0) {
                send_cmd_err(buffer[0], -1, "J<id> [<mode 0:Abs&Inc, 1:Inc>] [<min_rate %%>] [<max_rate %%>] [<timeout(ms)>]!");
                return;
            }
            motor = motor_get_by_id(id);
            if(motor == NULL) {
                send_cmd_err(buffer[0], id, "Motor not found!");
                return;
            }
            if(n == 5) {
                if(mode != MOTOR_MODE_ABSnINC && mode != MOTOR_MODE_INC) {
                    send_cmd_err(buffer[0], id, "Invalid mode!");
                    motor->last_cmd = 'j';
                    return;
                }
                if(min_rate < 0 || min_rate > 100) {
                    send_cmd_err(buffer[0], id, "Invalid min_rate!");
                    motor->last_cmd = 'j';
                    return;
                }
                if(max_rate < 0 || max_rate > 100) {
                    send_cmd_err(buffer[0], id, "Invalid max_rate!");
                    motor->last_cmd = 'j';
                    return;
                }
                if(timeout_ms < 0) {
                    send_cmd_err(buffer[0], id, "Invalid timeout_ms!");
                    motor->last_cmd = 'j';
                    return;
                }
                motor->last_cmd = buffer[0];
                //执行配置
                motor->config->mode = mode;
                motor->config->min_rate = min_rate;
                motor->config->max_rate = max_rate;
                motor->config->timeout_ms = timeout_ms;
                motor_update_param(motor);   
            }
            sprintf(_cmd_resp, "J%d %d %d %d %d\n", id, motor->config->mode, 
                motor->config->min_rate, motor->config->max_rate, motor->config->timeout_ms);
            usb_send(_cmd_resp);
            return;
        case 'P':   //电机PID配置命令:P<id> [Kp f_int16] [Ki f_int16] [Kd f_int16] [误差控制精度(角度)f_int16] [重力补偿方向int16]
            int16_t kp, ki, kd, err_angle, gravity_dir;
            n = sscanf(buffer, "P%hd %hd %hd %hd %hd %hd", &id, &kp, &ki, &kd, &err_angle, &gravity_dir);
            if(n == 0) {
                send_cmd_err(buffer[0], -1, "P<id> [<Kp f_int16>] [<Ki f_int16>] [<Kd f_int16>] [<err_angle f_int16>] [<gravity_dir int16>]!");
                return;
            }
            motor = motor_get_by_id(id);
            if(motor == NULL) {
                send_cmd_err(buffer[0], id, "Motor not found!");
                return;
            }
            motor->last_cmd = buffer[0];
            if(n == 6) {
                //执行配置
                motor->config->kp = i16_f(kp);
                motor->config->ki = i16_f(ki);
                motor->config->kd = i16_f(kd);
                motor->config->err_angle = i16_f(err_angle);
                motor->config->gravity_angle_dir = gravity_dir;
                motor_update_param(motor);
            }
            sprintf(_cmd_resp, "P%d %hd %hd %hd %hd %d\n", id, 
                f_i16(motor->config->kp), f_i16(motor->config->ki), f_i16(motor->config->kd), 
                f_i16(motor->config->err_angle), motor->config->gravity_angle_dir);
            usb_send(_cmd_resp);
            return;
        case '$':   //系统控制命令
            if(strncmp(buffer,"$load", 5) == 0) {
                //加载配置命令
                ret = motor_load_cfg();
                sprintf(_cmd_resp, "$load %d\n", ret);
                usb_send(_cmd_resp);
                return;
            } else if(strncmp(buffer,"$save", 5) == 0) {
                //保存配置命令
                ret = motor_save_cfg();
                sprintf(_cmd_resp, "$save %d\n", ret);
                usb_send(_cmd_resp);
                return;
            } else if(strncmp(buffer,"$reset", 6) == 0) {
                sprintf(_cmd_resp, "$reset 0\n");
                usb_send(_cmd_resp);
                soft_reset();
                return;
            } else if(strncmp(buffer, "$debug", 6) == 0) {
                _debug = !_debug;
                sprintf(_cmd_resp, "$debug %d\n", _debug);
                usb_send(_cmd_resp);
                return;
            } else if(strncmp(buffer,"$info", 5) == 0) {
                sprintf(_cmd_resp, "$info DMC %d ", MOTOR_MAX_CHANNELS);
                for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
                    motor_t *m = motor_get(i);
                    if(m != NULL) {
                        char id[8];
                        sprintf(id, "%d ", m->config->id);
                        strcat(_cmd_resp, id);
                    }
                }
                strcat(_cmd_resp, "\n");
                usb_send(_cmd_resp);
                return;
            } else if(strncmp(buffer,"$id", 3) == 0) {
                uint16_t new_id;
                n = sscanf(buffer, "$id %hd", &new_id);
                if( (n == 1) && (new_id%MOTOR_MAX_CHANNELS == 0) ) {
                    motor_t *mp;
                    sprintf(_cmd_resp, "$id %d ", MOTOR_MAX_CHANNELS);
                    for(int i=0; i<MOTOR_MAX_CHANNELS; i++) {
                        char id[8];
                        mp = motor_get(i);
                        mp->config->id = new_id + i;
                        motor_update_param(mp);
                        sprintf(id, "%d ", mp->config->id);
                        strcat(_cmd_resp, id);
                    }
                    strcat(_cmd_resp, "\n");
                    if(motor_save_cfg() == 0)
                        usb_send(_cmd_resp);
                    else 
                        usb_send("$id save error!\n");
                    return;
                }
                else {
                    sprintf(_cmd_resp, "$id <Id0 (Dived by %d)>\n", MOTOR_MAX_CHANNELS);
                    usb_send(_cmd_resp);
                    return;
                }
            } else {
                send_cmd_err(buffer[0], -1, "Unknown command");
            }
            break;  
        case 'L': //日志获取命令
            if(motor_get_log(_cmd_resp)) {
                usb_send(_cmd_resp);
                return;
            }
            usb_send("L \n");
            break;
        default:
            sprintf(_cmd_resp, "$load | $save | $reset | $info | $id\n");
            /*
            strcat(_cmd_resp, "R<id> [<目标角度f_int16>]\n");
            strcat(_cmd_resp, "S<id> [<目标速度(角度定点/秒)int16>]\n");
            strcat(_cmd_resp, "D<id> [<电机PWM占空比(+/- 0~100)int16>] [<电机转动时间(毫秒)int16>]\n");
            strcat(_cmd_resp, "I<id> [<每圈脉冲数int16>] [<顺时针旋转时编码器数值增量方向int16>]\n");
            strcat(_cmd_resp, "A<id> [<角度每变化1度电压增量(mV)f_int16>] [<零点电压值(mV)int16>] [<最小限位角度int16>] [<最大限位角度int16>]\n");
            strcat(_cmd_resp, "J<id> [工作模式int16] [<电机驱动最低PWM占空比int16>] [<电机驱动最高PWM占空比int16>] [每条命令连续运行最长时间(毫秒)int16]\n");
            strcat(_cmd_resp, "P<id> [Kp f_int16] [Ki f_int16] [Kd f_int16] [误差控制精度(角度)f_int16] [重力补偿方向int16]\n");
            */
            usb_send(_cmd_resp);
            break;
    }
  send_cmd_err(buffer[0], -1, "Unknown command");
}
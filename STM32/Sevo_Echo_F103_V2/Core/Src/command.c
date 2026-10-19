/*
 * ec11.c
 *
 *  Created on: Jan 18, 2026
 *      Author: Luke 1281772125@qq.com
 */

#include "main.h"
#include "usb_device.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "adc.h"
#include "gpio_sr04p.h"
#include "tim_servo.h"

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

static char _cmd_resp[1024];                      //命令返回缓存

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

//带反馈舵机命令
void process_O(char *buffer, uint32_t len) {
    int16_t target_angle;
    int n;
    int16_t id, adc;
    if(buffer[1] >= '0' && buffer[1] <= '9') {
        //单通道命令
        n = sscanf(buffer, "O%hd %hd", &id, &target_angle);
        if(n == 0 || id < 2 || id > 5) {
            send_cmd_err(buffer[0], -1, "O<2~5> [<舵机角度f_int16>]!");
            return;
        }
        if(n == 2 ) {
            //执行角度位置控制 
            set_servo(id, i16_f(target_angle));
        }
        //读取舵机状态
        target_angle = f_i16(get_servo(id));
        adc = adc_read_mv(id-2);
        sprintf(_cmd_resp, "O%hd %hd %hd\n", id, target_angle, adc);
        usb_send(_cmd_resp);
        return;
    }
    //多通道命令
    int16_t tag[4];
    n = sscanf(buffer, "O %hd %hd %hd %hd", &tag[0], &tag[1], &tag[2], &tag[3]);
    sprintf(_cmd_resp, "%c ", buffer[0]);
    char res[32];
    //发送命令
    for(int i=0; i<4; i++) {
        if(n == 4) {
            if(tag[i] != 0x7FFF)
                set_servo(i+2, i16_f(tag[i]));
        }
        //读取电机状态
        target_angle = f_i16(get_servo(i+2));
        adc = adc_read_mv(i);
        sprintf(res, "%hd %hd ", target_angle, adc);
        strcat(_cmd_resp, res);
    }
    strcat(_cmd_resp, "\n");
    usb_send(_cmd_resp);
}

//舵机测试命令
void process_U(char *buffer, uint32_t len) {
    int n;
    int16_t pwm_us, id = -1;
    uint16_t pwms_us[SERVO_NUMS];
    if(buffer[1]>= '0' && buffer[1] <= '9') {
        n = sscanf(buffer, "U%hd %hd", &id, &pwm_us);
        if(n == 0 || id < 0 || id >= SERVO_NUMS) {
            send_cmd_err(buffer[0], -1, "U<0~5> [<舵机PWM微秒数>]!");
            return;
        }
        if(n == 2) 
            set_servo_pwm(id, pwm_us);
        //读取舵机状态
        pwm_us = get_servo_pwm(id);
        sprintf(_cmd_resp, "U%hd %hd\n", id, pwm_us);
        usb_send(_cmd_resp);
        return;
    }
    int ch_s = 0;
    int ch_e = SERVO_NUMS;
    //多通道命令
    n = sscanf(buffer, "U %hd %hd %hd %hd %hd %hd", &pwms_us[0], &pwms_us[1], &pwms_us[2], &pwms_us[3], &pwms_us[4], &pwms_us[5]);
    if(n == 2) {
        //设置头部
        if(pwms_us[0] != 0x7FFF)
            set_servo_pwm(0, pwms_us[0]);
        if(pwms_us[1] != 0x7FFF)
            set_servo_pwm(1, pwms_us[1]);
        ch_e = 2;
    }
    else if(n == 4) { 
        //设置4个轮子
        if(pwms_us[0] != 0x7FFF)
            set_servo_pwm(2, pwms_us[0]);
        if(pwms_us[1] != 0x7FFF)
            set_servo_pwm(3, pwms_us[1]);
        if(pwms_us[2] != 0x7FFF)
            set_servo_pwm(4, pwms_us[2]);
        if(pwms_us[3] != 0x7FFF)
            set_servo_pwm(5, pwms_us[3]);
        ch_s = 2;
        ch_e = 6;
    }
    else if(n == 6) {
        //设置全部
        for(int i=0; i<SERVO_NUMS; i++) {
            if(pwms_us[i] != 0x7FFF)
                set_servo_pwm(i, pwms_us[i]);
        }   
    }
    //返回所有舵机微秒数
    char res[32];
    sprintf(_cmd_resp, "U ");
    for(int i=ch_s; i<ch_e; i++) {
        pwm_us = get_servo_pwm(i);
        sprintf(res, "%hd ", pwm_us);
        strcat(_cmd_resp, res);
    }
    strcat(_cmd_resp, "\n");
    usb_send(_cmd_resp);
    return;
}

//处理上位机命令
static uint32_t _echo_dists[SR04P_NUM];
static uint32_t _echo_times[SR04P_NUM];

void process_command(char *buffer, uint32_t len) {
    int n, ret;
    int16_t id;
    char res[32];
    
    switch(buffer[0]) {
        //读取全部状态指令
        case 'F': 
            trigger_echo(); //触发一次测距
            _cmd_resp[0] = 'F';
            _cmd_resp[1] = ' ';
            _cmd_resp[2] = 0;
            //读取ADC
            for(int i=0; i<6; i++) {
                sprintf(res, "%hd ", adc_read_mv(i));
                strcat(_cmd_resp, res);
            }
            //读取超声波测距
            wait_echo();
            get_echos_distance(_echo_dists, _echo_times);
             for(int i=0; i<SR04P_NUM; i++) {
                if(_echo_times[i]) {
                    sprintf(res, "%u ", _echo_dists[i]);
                    strcat(_cmd_resp, res);
                }
                else {
                    strcat(_cmd_resp, "0 ");
                }
            }
            //添加结束符
            strcat(_cmd_resp, "\n");
            usb_send(_cmd_resp);
            return;   
        case 'f':   //小写的f返回的是ADC的值，而不是电压 
            trigger_echo(); //触发一次测距
            _cmd_resp[0] = 'f';
            _cmd_resp[1] = ' ';
            _cmd_resp[2] = 0;
            //读取ADC
            for(int i=0; i<6; i++) {
                sprintf(res, "%hd ", adc_read(i));
                strcat(_cmd_resp, res);
            }
            //读取超声波测距
            wait_echo();
            get_echos_distance(_echo_dists, _echo_times);
             for(int i=0; i<SR04P_NUM; i++) {
                if(_echo_times[i]) {
                    sprintf(res, "%u ", _echo_dists[i]);
                    strcat(_cmd_resp, res);
                }
                else {
                    strcat(_cmd_resp, "0 ");
                }
            }
            //添加结束符
            strcat(_cmd_resp, "\n");
            usb_send(_cmd_resp);
            return;   
        case 'U':   //舵机测试命令:U<0~5> [<舵机PWM微秒数>]
            process_U(buffer, len);
            return;
        case 'E':   //超声波测距命令:E<0~9>、
            process_echo();
            n = sscanf(buffer, "E%hd", &id);
            if(n == 1) {
                if(id < 0 || id >= SR04P_NUM) {
                    send_cmd_err(buffer[0], -1, "E<0~9>!");
                    return;
                }
                _echo_dists[id] = get_echo_distance(id, &_echo_times[id]);
                if(_echo_times[id])
                    sprintf(_cmd_resp, "E%hd %u\n", id, _echo_dists[id]);
                else
                    sprintf(_cmd_resp, "E%hd 0\n", id);
                usb_send(_cmd_resp);
                return;
            }
            //读取所有通道测距
            get_echos_distance(_echo_dists, _echo_times);
            sprintf(_cmd_resp, "E ");
            for(int i=0; i<SR04P_NUM; i++) {
                if(_echo_times[i]) {
                    sprintf(res, "%u ", _echo_dists[i]);
                    strcat(_cmd_resp, res);
                }
                else {
                    strcat(_cmd_resp, "0 ");
                }
            }
            strcat(_cmd_resp, "\n");
            usb_send(_cmd_resp);
            return;
        case 'K':   //输出开关命令K0 <On:1 | Off:0>
            n = sscanf(buffer, "K%hd %d", &id, &ret);
            if(n == 0 || id != 0) {
                send_cmd_err(buffer[0], -1, "K0 [<On:1 | Off:0>]!");
                return;
            }
            if(n == 2) {
                if(ret == 0)
                    HAL_GPIO_WritePin(Power_Proj_GPIO_Port, Power_Proj_Pin, GPIO_PIN_RESET);
                else
                    HAL_GPIO_WritePin(Power_Proj_GPIO_Port, Power_Proj_Pin, GPIO_PIN_SET);
            }
            //读取输出开关状态
            ret = HAL_GPIO_ReadPin(Power_Proj_GPIO_Port, Power_Proj_Pin);
            sprintf(_cmd_resp, "K0 %d\n", ret);
            usb_send(_cmd_resp);
            return;

        case '$':   //系统控制命令
            if(strncmp(buffer,"$reset", 6) == 0) {
                sprintf(_cmd_resp, "!reset 0\n");
                usb_send(_cmd_resp);
                HAL_NVIC_SystemReset();
                return;
            } else if(strncmp(buffer,"$info", 5) == 0) {
                strcpy(_cmd_resp, "$info SE2\n");
                usb_send(_cmd_resp);
                return;
            } else {
                send_cmd_err(buffer[0], -1, "Unknown command\n");
            }
            break;  
        default:
            sprintf(_cmd_resp, "$reset | $info | F | f | U | E | K\n");
            usb_send(_cmd_resp);
            break;
    }
  send_cmd_err(buffer[0], -1, "Unknown command\n");
}

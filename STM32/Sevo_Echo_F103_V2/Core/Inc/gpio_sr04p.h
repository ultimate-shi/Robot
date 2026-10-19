#include "main.h"

#define SR04P_NUM   8

#define ECHO_LEFT_FRONT             0  
#define ECHO_RIGHT_FRONT            1
#define ECHO_LEFT_REAR              2 
#define ECHO_RIGHT_REAR             3  
#define ECHO_TOP_FRONT              4
#define ECHO_SIDE_LEFT_FRONT        5
#define ECHO_SIDE_RIGHT_FRONT       6


int16_t init_sr04p();
int16_t process_echo_int(uint16_t GPIO_Pin);
void process_echo();
uint32_t get_echo_distance(uint16_t index, uint32_t *time_ms);
void get_echos_distance(uint32_t* distance_array, uint32_t *time_ms_array);
void set_echo_interval(uint16_t interval_ms);
#ifndef _RTOS_ 
void trigger_echo();
void wait_echo();
#endif

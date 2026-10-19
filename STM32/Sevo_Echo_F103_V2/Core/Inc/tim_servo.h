#include "main.h"

#define SERVO_NUMS	6

typedef struct servo_tim_s {
	TIM_HandleTypeDef *tim;
	uint32_t ch;
} ServoTIM;

;
int16_t init_servo();
void set_servo(uint16_t channel, float deg);
float get_servo(uint16_t ch);
void set_servo_pwm(uint16_t channel, uint16_t us);
uint16_t get_servo_pwm(uint16_t ch);
void process_servo(uint32_t now_ms);
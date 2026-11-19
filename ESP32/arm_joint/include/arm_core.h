/* 作用：定义不依赖ESP-IDF的协议/关节控制核心；用法：硬件任务或主机单元测试调用。 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "arm_config.h"
typedef enum { STOP=0, RUN=1, FAULT=2 } arm_state_t;
typedef enum { NONE=0, POS=1, PWM=2 } arm_mode_t;
typedef struct {
    int zero_raw, encoder_dir, motor_dir, neutral_us;
    bool zero_valid, direction_valid, neutral_valid;
    float vmax, amax, kp, ki, kd;
    int hold; /* 追加字段，旧schema=2前缀布局保持不变。 */
} arm_settings_t;
typedef struct {
    arm_settings_t cfg;
    arm_state_t state;
    arm_mode_t mode;
    int fault, raw, pwm_us;
    bool sensor_valid;
    float angle, speed, arrival_speed, target, ref, ref_speed, integral, prev_error, d_filtered;
    int64_t sample_us, deadline_us, tick_us, settled_us, bus_deadline_us;
} arm_joint_t;
typedef struct {
    char name[16], op;
    int id, argc, ttl_ms;
    float val[4];
} arm_command_t;
typedef struct {
    char data[ARM_BUS_LINE+1];
    size_t used;
    bool discard;
    int64_t started_us;
} arm_line_t;
void arm_defaults(arm_settings_t *c);
bool arm_settings_decode(arm_settings_t *c, uint32_t schema, const void *blob, size_t size);
void arm_bus_seen(arm_joint_t *j, int64_t now);
void arm_init(arm_joint_t *j, const arm_settings_t *c);
void arm_stop(arm_joint_t *j);
void arm_fault(arm_joint_t *j, int fault);
void arm_sample(arm_joint_t *j, bool valid, int raw, int64_t now);
void arm_tick(arm_joint_t *j, int64_t now);
const char *arm_move_check(const arm_joint_t *j, char op, float value);
void arm_move(arm_joint_t *j, char op, float value, int64_t deadline);
const char *arm_parse(const char *line, bool bus, arm_command_t *c);
const char *arm_setting(arm_joint_t *j, const arm_command_t *c);
uint16_t arm_crc(const char *data, size_t n);
bool arm_bus_encode(uint16_t seq, const char *body, char *out, size_t cap);
bool arm_bus_decode(char *line, uint16_t *seq, char **body);
/* 返回1完整行，-1超长/非法字符，0待收；超时后须丢到下一换行。 */
int arm_line_feed(arm_line_t *s, unsigned char byte, int64_t now, size_t max, int64_t timeout);
/* 仅用于UART总线：忽略帧外字节，遇到@重新同步；帧内仍严格组帧并由调用方校验CRC/序号。 */
int arm_bus_line_feed(arm_line_t *s, unsigned char byte, int64_t now);

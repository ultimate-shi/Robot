/* 作用：集中定义板级引脚、时序和初始控制参数；用法：确认电路后修改并重新构建，运行标定由A/J/P保存。 */
#pragma once
#ifndef ARM_MASTER
#define ARM_MASTER 1
#endif
#ifndef ARM_NODE_ID
#define ARM_NODE_ID 1
#endif
#define ARM_COUNT 2
#define ARM_UART_TX_MASTER 20
#define ARM_UART_RX_MASTER 21
#define ARM_UART_TX_SLAVE 21
#define ARM_UART_RX_SLAVE 20
#define ARM_TX_EN_PIN 7
#define ARM_PWM_PIN 10
#define ARM_SCL_PIN 4
#define ARM_SDA_PIN 3
#define ARM_I2C_HZ 100000
#define ARM_AS5600_ADDR 0x36
#define ARM_BUS_BAUD 115200
/* Q2门控回包含帧间异常字节，不能依赖空闲超时才取出不足120字节的短帧。 */
#define ARM_UART_RX_FULL_THRESHOLD 1
#define ARM_CONTROL_MS 5
#define ARM_ENCODER_STALE_US 20000
#define ARM_COMMAND_US 100000
#define ARM_POLL_US 50000
#define ARM_REPLY_US 15000
#define ARM_QUIET_US 18000
#define ARM_USB_LINE 160
#define ARM_BUS_LINE 192
#define ARM_BUS_FRAME_US 10000
#define ARM_BUS_TRACE_BYTES 96
#define ARM_PWM_HZ 50
#define ARM_PWM_MIN 500
#define ARM_PWM_MAX 2500
#define ARM_PWM_NEUTRAL 1500
#define ARM_LIMIT_DEG 90.0f
#define ARM_LIMIT_MARGIN_DEG 1.0f
#define ARM_PWM_US_PER_DPS 20.0f
#define ARM_DEFAULT_VMAX 10.0f
#define ARM_DEFAULT_AMAX 30.0f
#define ARM_DEFAULT_KP 2.0f
#define ARM_MAX_VMAX 90.0f
#define ARM_MAX_AMAX 360.0f

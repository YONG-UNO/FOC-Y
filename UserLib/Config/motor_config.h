//
// Created by DingYong on 2026/2/2.
//
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// 单片机配置参数：
#define MOTOR_PWM_FREQ        20000   // 20kHz
#define TIM_CLOCK_FREQ        72000000
#define TIM_ARR               (TIM_CLOCK_FREQ / MOTOR_PWM_FREQ) // 3600
#define TIM_RCR               4       // 每5次溢出触发一次更新事件
#define FOC_FREQ              (MOTOR_PWM_FREQ / (TIM_RCR + 1)) // 4kHz, 驱动桥pwm频率，Hz
#define MOTOR_SPEED_CALC_FREQ 930     // 电机速度计算频率，Hz

/* 电机配置 */
#define POLE_PAIRS   (14/2)       // 极对数: 磁极数/2

#endif //MOTOR_CONFIG_H
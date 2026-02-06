//
// Created by DingYong on 2026/2/3.
//
#ifndef INA199_H
#define INA199_H

/* 头文件 */
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_adc.h"

/* 芯片参数 */
#define INA199_GAIN          50.0f    // x1系列增益50,x2系列增益100,x3系列增益200
#define INA199_R_SHUNT       0.02f    // 采样电阻阻值
#define INA199_ADC_REF_VOLT  3.3f     // ADC参考电压
#define INA199_ADC_BITS      12U      // ADC分辨率
#define INA199_FILTER_ALPHA  0.1f     // 电流低通滤波系数

/* 全局变量 */
// 原始电流
extern float ina199_current_u;    // U相电流值(SI)
extern float ina199_current_v;    // V相电流值(SI)
// 滤波后的电流
extern float ina199_current_d;
extern float ina199_current_q;
// 电角度
extern float elec_angle;

/* 函数 */
void ina199_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
void ina199_adc_injected_cplt_callback(ADC_HandleTypeDef *hadc);
void ina199_get_dq_current(float *current_d, float *current_q);

#endif //INA199_H

//
// Created by DingYong on 2026/2/3.
//
#ifndef SVPWM_H
#define SVPWM_H

/* 结构体 */

// 占空比结构体
typedef struct {
    float d_u;    // U相占空比 0~1
    float d_v;    // V相占空比 0~1
    float d_w;    // W相占空比 0~1
} svpwm_duty_t;

/* 函数 */
svpwm_duty_t svpwm_polar(float theta, float amp);           // 极坐标形式SVPWM
svpwm_duty_t svpwm_dq(float elec_angle, float d, float q);  // 笛卡尔(dq)坐标形式SVPWM

#endif //SVPWM_H
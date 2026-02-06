//
// Created by DingYong on 2026/2/3.
//
#include "svpwm.h"

/* 头文件 */
#include <stdint.h>
#include "arm_math.h"

/* 宏定义 */
#define RAD_60  (PI / 3.0f)    // 六个扇区宽度
#define SQRT3    1.7320508075688772f  // 根号3,用于推导线性调制区(内接圆),判断扇区边界(|β| > √3 * |α|)

/* UVW相pwm占空比 */
svpwm_duty_t svpwm_duty = {0};

/* 数组 */
// 六个方向的电压矢量
static const int8_t svpwm_v[6][3] = {
    {1, 0, 0}, // V1  U开/VW关
    {1, 1, 0}, // V2
    {0, 1, 0}, // V3
    {0, 1, 1}, // V4
    {0, 0, 1}, // V5
    {1, 0, 1}  // V6
};
// 极坐标系:角度判断扇区
// 笛卡尔坐标系:atan2(β,α)判断扇区
// K映射扇区号
static const int8_t k_to_sector[8] = {4, 6, 5, 5, 3, 1, 2, 2};


/* 函数 */

/**
 * @brief  极坐标 SVPWM
 * @param  theta: 目标磁矢量角度(0~2PI)
 * @param  amp: 目标磁矢量幅值，范围 0~1
 * @retval 三相占空比
 */
svpwm_duty_t svpwm_polar(float theta, float amp) {
    // 幅值合法
    theta = fmodf(theta, 2.0f * PI);
    if (theta < 0) theta += 2.0f * PI;
    if (amp > 1.0f) amp = 1.0f;
    if (amp < 0.0f) amp = 0.0f;

    // 扇区计算
    int sector = (int)(theta / RAD_60) + 1;
    if (sector > 6) sector = 6;

    // 基矢量作用时间
    float t_m = amp * sinf(sector * RAD_60 - theta);             // 顺时针(m)方向的基础矢量在周期的时间
    float t_n = amp * sinf(theta - (sector * RAD_60 - RAD_60));  // 逆时针(n)方向的基础矢量在周期的时间
    float t_0 = 1.0f - t_m - t_n;  // 零矢量在周期的时间,零矢量时间平分到两端
    if (t_m < 0) t_m = 0;
    if (t_n < 0) t_n = 0;
    if (t_0 < 0) t_0 = 0;

    // 计算占空比(七段式:对称结构,谐波更小)
    // 使用svpwm_v表查找基矢量
    // 零矢量时间t_0 平分到两端
    svpwm_duty.d_u = t_m * svpwm_v[sector - 1][0] + t_n * svpwm_v[sector % 6][0] + t_0 / 2;
    svpwm_duty.d_v = t_m * svpwm_v[sector - 1][1] + t_n * svpwm_v[sector % 6][1] + t_0 / 2;
    svpwm_duty.d_w = t_m * svpwm_v[sector - 1][2] + t_n * svpwm_v[sector % 6][2] + t_0 / 2;

    // 限幅到 0~1
    if (svpwm_duty.d_u > 1.0f) svpwm_duty.d_u = 1.0f;
    if (svpwm_duty.d_u < 0.0f) svpwm_duty.d_u = 0.0f;
    if (svpwm_duty.d_v > 1.0f) svpwm_duty.d_v = 1.0f;
    if (svpwm_duty.d_v < 0.0f) svpwm_duty.d_v = 0.0f;
    if (svpwm_duty.d_w > 1.0f) svpwm_duty.d_w = 1.0f;
    if (svpwm_duty.d_w < 0.0f) svpwm_duty.d_w = 0.0f;

    return svpwm_duty;
}

/**
 * @brief  笛卡尔坐标（dq）SVPWM
 * @param  elec_angle: 电角度 (rad)
 * @param  d: d轴电压（直向，控制磁链）
 * @param  q: q轴电压（切向，控制转矩）
 * @retval 三相占空比
 */
svpwm_duty_t svpwm_dq(float elec_angle, float d, float q) {
    // 反PARK变换
    // alpha = d * cosφ - q * sinφ;
    // beta  = d * sinφ + q * cosφ;
    float cos_phi = cosf(elec_angle);
    float sin_phi = sinf(elec_angle);
    float alpha = d * cos_phi - q * sin_phi;
    float beta  = d * sin_phi + q * cos_phi;

    // 扇区判断（无atan，纯条件判断）
    int A = (beta > 0) ? 1 : 0;
    int B = (fabsf(beta) > SQRT3 * fabsf(alpha)) ? 1 : 0;
    int C = (alpha > 0) ? 1 : 0;
    int K = (A << 2) | (B << 1) | C;
    int sector = k_to_sector[K];

    // 基矢量作用时间
    float t_m = alpha * sinf(sector * RAD_60) - beta * cosf(sector * RAD_60);        // 顺时针(m)方向的基础矢量在周期的时间
    float t_n = beta * cosf(sector * RAD_60 - RAD_60) - alpha * sinf(sector * RAD_60 - RAD_60); // 逆时针(n)方向的基础矢量在周期的时间
    float t_0 = 1.0f - t_m - t_n;   // 零矢量在周期的时间,零矢量时间平分到两端
    if (t_m < 0) t_m = 0;
    if (t_n < 0) t_n = 0;
    if (t_0 < 0) t_0 = 0;

    // 计算占空比(七段式:对称结构,谐波更小)
    // 使用svpwm_v表查找基矢量
    // 零矢量时间t_0 平分到两端
    svpwm_duty.d_u = t_m * svpwm_v[sector - 1][0] + t_n * svpwm_v[sector % 6][0] + t_0 / 2;
    svpwm_duty.d_v = t_m * svpwm_v[sector - 1][1] + t_n * svpwm_v[sector % 6][1] + t_0 / 2;
    svpwm_duty.d_w = t_m * svpwm_v[sector - 1][2] + t_n * svpwm_v[sector % 6][2] + t_0 / 2;

    // 限幅到 0~1
    if (svpwm_duty.d_u > 1.0f) svpwm_duty.d_u = 1.0f;
    if (svpwm_duty.d_u < 0.0f) svpwm_duty.d_u = 0.0f;
    if (svpwm_duty.d_v > 1.0f) svpwm_duty.d_v = 1.0f;
    if (svpwm_duty.d_v < 0.0f) svpwm_duty.d_v = 0.0f;
    if (svpwm_duty.d_w > 1.0f) svpwm_duty.d_w = 1.0f;
    if (svpwm_duty.d_w < 0.0f) svpwm_duty.d_w = 0.0f;

    return svpwm_duty;
}
//
// Created by DingYong on 2026/2/3.
//
#include "filter.h"

/**
 * @brief  一阶低通滤波器（IIR实现）
 * @param  input       当前输入值（最新采样值）
 * @param  last_output 上一次滤波输出（历史值）
 * @param  alpha       滤波系数 (0.0 ~ 1.0)，决定平滑程度
 *                     - 越大：响应快，噪声抑制弱
 *                     - 越小：响应慢，噪声抑制强
 * @retval 当前滤波输出值（平滑后的结果）
 *
 * @note   算法公式：
 *         y(k) = alpha * x(k) + (1 - alpha) * y(k-1)
 *         其中 x(k) 为当前输入，y(k-1) 为上次输出
 */
float low_pass_filter(float input, float last_output, float alpha)
{
    return alpha * input + (1.0f - alpha) * last_output;
}

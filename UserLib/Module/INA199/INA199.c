//
// Created by DingYong on 2026/2/3.
//
#include "INA199.h"

/* 头文件 */
#include "filter.h"
#include "arm_math.h"
#include "motor_config.h"
#include "MT6701.h"

/* 全局变量 */
float ina199_voltage_u = 0.0f;
float ina199_voltage_v = 0.0f;
float ina199_current_u = 0.0f; // U相电流(A)
float ina199_current_v = 0.0f; // V相电流
float ina199_current_d = 0.0f; // d轴滤波之后的电流
float ina199_current_q = 0.0f; // q轴滤波之后的电流
float elec_angle = 0.0f;       // 转子电角度(用于Park变换)

/* 静态全局变量(不暴露,附加地址固定) */
static ADC_HandleTypeDef *s_hadc1 = NULL; // 空指针判断 + 句柄保存
static ADC_HandleTypeDef *s_hadc2 = NULL;

/* 函数 */

// 声明私有函数
static void ina199_get_elec_angle(float mech_angle);   // 计算电角度
static float ina199_adc_to_voltage(uint16_t adc_val);  // adc->电压
static float ina199_voltage_to_current(float voltage); // 电压->电流

// 实现
/**
 * @brief  INA199初始化（绑定ADC句柄+启动ADC注入组）
 */
void ina199_init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2) {
    if (hadc1 == NULL || hadc2 == NULL) return;

    // 保存句柄地址,用于adc回调判断
    s_hadc1 = hadc1;
    s_hadc2 = hadc2;

    // 启用ADC注入组
    HAL_ADC_Start(hadc1);
    HAL_ADC_Start(hadc2);
    HAL_ADCEx_InjectedStart_IT(hadc1);
    HAL_ADCEx_InjectedStart_IT(hadc2);
}

/**
 * @brief  ADC注入组完成回调（核心：电流计算+坐标变换）
 * @todo   零漂问题
 */
void ina199_adc_injected_cplt_callback(ADC_HandleTypeDef *hadc) {

    if (hadc != s_hadc1 && hadc != s_hadc2) return;

    // 读取ADC采样值(注入组Rank1)
    uint16_t adc_val_u = HAL_ADCEx_InjectedGetValue(s_hadc1, ADC_INJECTED_RANK_1);
    uint16_t adc_val_v = HAL_ADCEx_InjectedGetValue(s_hadc2, ADC_INJECTED_RANK_1);

    // 计算U/V相实际电流(ADC->电压->电流)
    ina199_voltage_u = ina199_adc_to_voltage(adc_val_u);
    ina199_voltage_v = ina199_adc_to_voltage(adc_val_v);
    ina199_current_u = ina199_voltage_to_current(ina199_voltage_u);
    ina199_current_v = ina199_voltage_to_current(ina199_voltage_v);

    // // 零飘死区(无电流会零飘到0.08A)
    // const float DEAD_ZONE = 0.08f;   // 10mA视作为零飘
    // if (fabs(ina199_current_u) < DEAD_ZONE) ina199_current_u = 0;
    // if (fabs(ina199_current_v) < DEAD_ZONE) ina199_current_v = 0;

    // Clark变换: U/V相电流->α/β静止坐标系
    static float i_alpha = 0.0f;
    static float i_beta = 0.0f;
    arm_clarke_f32(ina199_current_u, ina199_current_v, &i_alpha, &i_beta);

    // Park变换: α/β静止坐标系 -> d/q旋转坐标系(依赖转子角度)
    ina199_get_elec_angle(encoder_angle);
    float sin_rotor = arm_sin_f32(elec_angle);
    float cos_rotor = arm_cos_f32(elec_angle);
    static float i_d = 0.0f;
    static float i_q = 0.0f;
    arm_park_f32(i_alpha, i_beta, &i_d, &i_q, sin_rotor, cos_rotor);

    // 电流滤波(低通滤波器)
    ina199_current_d = low_pass_filter(i_d, ina199_current_d, INA199_FILTER_ALPHA);
    ina199_current_q = low_pass_filter(i_q, ina199_current_q, INA199_FILTER_ALPHA);

    // 告诉单片机,我已经处理完了,下一次TIM1_TRGO触发请继续中断,再次战斗
    HAL_ADCEx_InjectedStart_IT(s_hadc1);
    HAL_ADCEx_InjectedStart_IT(s_hadc2);
}

/**
 * @brief  获取d/q轴电流（供FOC控制使用）
 */
void ina199_get_dq_current(float *current_d, float *current_q) {
    if (current_d != NULL)
        *current_d = ina199_current_d;
    if (current_q != NULL)
        *current_q = ina199_current_q;
}

/**
 * @brief  计算转子电角度(Park)
 */
static void ina199_get_elec_angle(float mech_angle) {
    elec_angle = mech_angle * POLE_PAIRS;
}

/**
 * @brief  ADC数字量转换为INA199输出电压（考虑双向电流偏移，对齐手册8.2.2）
 * @param  adc_val: ADC采样值（数字量，如12位ADC范围0~4095）
 * @retval INA199输出电压（V），范围：-REF/2 ~ +REF/2（支持双向电流）
 * @note   依赖硬件：INA199的REF引脚需接 ADC_REFERENCE_VOLT/2（如3.3V系统接1.65V）
 */
static float ina199_adc_to_voltage(uint16_t adc_val) {
    // 1. ADC数字量转原始电压（0 ~ ADC_REFERENCE_VOLT）
    // 公式：电压 = 参考电压 × 采样值 / 最大采样值
    float raw_voltage = INA199_ADC_REF_VOLT * (float) adc_val / ((1 << INA199_ADC_BITS) - 1);

    // 2. 电压偏移：减去REF引脚中点电压，实现双向电压输出（手册8.2.2）
    // 若REF引脚接地，此步骤省略（仅支持单向电流）；若接中点电压，必须保留以区分电流方向
    return raw_voltage - (INA199_ADC_REF_VOLT / 2.0f);
}

/**
 * @brief  INA199输出电压转换为实际电流（严格遵循手册公式）
 * @param  voltage: INA199输出电压（V），范围：-REF/2 ~ +REF/2
 * @retval 实际电流（A），正/负值对应电流正/反方向
 * @note   关键参数需与硬件一致：
 *         - INA199_GAIN：INA199型号增益（x1=50，x2=100，x3=200，手册4.器件比较表）
 *         - INA199_R_SHUNT：采样电阻阻值（Ω，如10mΩ=0.01f，手册7.3.2推荐满量程压降10~50mV）
 */
static float ina199_voltage_to_current(float voltage) {
    // 反向推导手册公式：I = Vout / (Gain × Rshunt)
    return voltage / (INA199_GAIN * INA199_R_SHUNT);
}

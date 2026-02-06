/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h"
#include "INA199.h"
#include "MT6701.h"
#include "svpwm.h"
#include "tim.h"
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
    */
    multimode.Mode = ADC_DUALMODE_INJECSIMULT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/* ADC2 init function */
void MX_ADC2_Init(void) {
    /* USER CODE BEGIN ADC2_Init 0 */

    /* USER CODE END ADC2_Init 0 */

    ADC_InjectionConfTypeDef sConfigInjected = {0};

    /* USER CODE BEGIN ADC2_Init 1 */

    /* USER CODE END ADC2_Init 1 */

    /** Common config
    */
    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC2_Init 2 */

    /* USER CODE END ADC2_Init 2 */
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC1) {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* ADC1 clock enable */
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PA0-WKUP     ------> ADC1_IN0
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC1 interrupt Init */
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
        /* USER CODE BEGIN ADC1_MspInit 1 */

        /* USER CODE END ADC1_MspInit 1 */
    } else if (adcHandle->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspInit 0 */

        /* USER CODE END ADC2_MspInit 0 */
        /* ADC2 clock enable */
        __HAL_RCC_ADC2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**ADC2 GPIO Configuration
        PA1     ------> ADC2_IN1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* ADC2 interrupt Init */
        HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
        /* USER CODE BEGIN ADC2_MspInit 1 */

        /* USER CODE END ADC2_MspInit 1 */
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adcHandle) {
    if (adcHandle->Instance == ADC1) {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC1_CLK_DISABLE();

        /**ADC1 GPIO Configuration
        PA0-WKUP     ------> ADC1_IN0
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

        /* ADC1 interrupt Deinit */
        /* USER CODE BEGIN ADC1:ADC1_2_IRQn disable */
        /**
        * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
        /* USER CODE END ADC1:ADC1_2_IRQn disable */

        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    } else if (adcHandle->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspDeInit 0 */

        /* USER CODE END ADC2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC2_CLK_DISABLE();

        /**ADC2 GPIO Configuration
        PA1     ------> ADC2_IN1
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

        /* ADC2 interrupt Deinit */
        /* USER CODE BEGIN ADC2:ADC1_2_IRQn disable */
        /**
        * Uncomment the line below to disable the "ADC1_2_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(ADC1_2_IRQn); */
        /* USER CODE END ADC2:ADC1_2_IRQn disable */

        /* USER CODE BEGIN ADC2_MspDeInit 1 */

        /* USER CODE END ADC2_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
typedef struct {
    float kp;
    float ki;
    float kd;

    float p_out;
    float i_out;
    float d_out;
    float sum_out;

    float i_out_limit;
    float sum_out_limit;

    float target;
    float feedback;
    float error[2];
} pid_t;

pid_t id_pid = {0};
pid_t iq_pid = {0};

void current_pid_init(pid_t *id_pid,pid_t *iq_pid);

void current_pid_init(pid_t *id_pid,pid_t *iq_pid) {
    // Id环(将d轴电流压到0,避免电机磁链波动)
    id_pid->kp = 0.0f;
    id_pid->ki = 0.0f;
    id_pid->i_out_limit = 0.0f;
    id_pid->sum_out_limit = 0.0f;

    // Iq(力矩/电流环)
    // 不写kd减小对高频噪声的进一步放大
    iq_pid->kp = 0.0f;
    iq_pid->kd = 0.0f;
    iq_pid->i_out_limit = 0.0f;
    iq_pid->sum_out_limit = 0.0f;
}

void pid_calc(pid_t *pid, float target, float feedback);
void pid_calc(pid_t *pid, float target, float feedback) {

    pid->target = target;
    pid->feedback = feedback;

    pid->error[1] = pid->error[0];
    pid->error[0] = feedback - target;


    pid->p_out = pid->kp * pid->error[0];    // p_out
    pid->i_out += pid->ki * pid->error[0];   // i_out
    pid->d_out = pid->kd * (pid->error[0] - pid->error[1]);

    if (pid->i_out > pid->i_out_limit) pid->i_out = pid->i_out_limit;
    if (pid->i_out < -pid->i_out_limit) pid->i_out = -pid->i_out_limit;

    pid->sum_out = pid->p_out + pid->i_out + pid->d_out;

    if (pid->sum_out > pid->sum_out_limit) pid->sum_out = pid->sum_out_limit;
    if (pid->sum_out < -pid->sum_out_limit) pid->sum_out = -pid->sum_out_limit;

}

// 电流环计算
void current_loop(float Id_target, float Id_feedback,
                  float Iq_target, float Iq_feedback,
                  float *Vd_out, float *Vq_out);

void current_loop(float Id_target, float Id_feedback,
                  float Iq_target, float Iq_feedback,
                  float *Vd_out, float *Vq_out) {
    pid_calc(&id_pid, Id_target, Id_feedback);
    pid_calc(&iq_pid, Iq_target, -Iq_feedback);

    *Vd_out = id_pid.sum_out;
    *Vq_out = iq_pid.sum_out;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    ina199_adc_injected_cplt_callback(hadc);

    // 转速计算
    static float motor_speed = 0.0f;
    static float last_angle = 0.0f;
    static float delta = 0.0f;

    delta = encoder_angle - last_angle;
    if (delta > PI) delta -= 2 * PI;
    if (delta < -PI) delta += 2 * PI;
    motor_speed = delta / 0.00025f;
    last_angle = encoder_angle;

    // 电流环
    static float Id_target = 0.0f;
    static float Iq_target = 0.5f;
    static float Vd = 0.0f;
    static float Vq = 0.0f;

    current_loop(Id_target, ina199_current_d, Iq_target, ina199_current_q, &Vd,&Vq);

    // 电角度(编码器)
    elec_angle = fmodf(elec_angle,2*PI);

    //svpwm
    svpwm_duty_t duty = svpwm_dq(elec_angle, Vd, Vq);

    // 更新pwm
    uint16_t ccr_u = (uint16_t) (duty.d_u * htim1.Init.Period);
    uint16_t ccr_v = (uint16_t) (duty.d_v * htim1.Init.Period);
    uint16_t ccr_w = (uint16_t) (duty.d_w * htim1.Init.Period);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_v);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_w);
}

/* USER CODE END 1 */

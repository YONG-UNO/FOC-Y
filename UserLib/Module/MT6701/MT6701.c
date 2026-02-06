//
// Created by DingYong on 2026/2/1.
//
#include "MT6701.h"

#include "arm_math.h"

/*全局变量*/
uint8_t mt6701_rx_data[3] = {0}; // 接收缓存初始化清零
float encoder_angle = 0.0f; // 编码器在当前一圈的角度值
float motor_logic_angle = 0.0f; // 逻辑角度,累计多圈的角度值
float position_cycle = MT6701_ANGLE_CYCLE; // 逻辑角度周期

/*函数*/

/**
 * @brief  MT6701初始化（就是配置片选引脚,防止cubemx配错）
 * @todo   DMA循环
 */
void mt6701_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 使能GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE(); // 所有外设都需要时钟驱动,否则初始化失败,就类似于"没有开机"

    // 推挽,上拉,高速
    GPIO_InitStruct.Pin = MT6701_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // mcu可以直接驱动高低电平,不需要额外电阻
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MT6701_CS_PORT, &GPIO_InitStruct);

    // 初始状态:片选拉高(未选中)
    HAL_GPIO_WritePin(MT6701_CS_PORT, MT6701_CS_PIN, GPIO_PIN_SET);

    // 先拉低片选（选中MT6701）拉低 CS 是 MT6701 SSI 协议要求（CS 低电平芯片才响应 CLK 并输出数据）；
    HAL_GPIO_WritePin(MT6701_CS_PORT, MT6701_CS_PIN, GPIO_PIN_RESET);
    // 启动SPI DMA收发（TX和RX共用缓存，长度3字节）
    // 启动 DMA 是 STM32 中断触发的前提（仅 DMA 传输完成才会调用回调函数，不然无法"起振"）
    HAL_SPI_TransmitReceive_DMA(&hspi1, mt6701_rx_data, mt6701_rx_data, 3);
}

/**
 * @brief  SPI DMA收发完成回调函数（释放spi,计算MT6701角度）
 * @param  hspi: SPI句柄
 */
void mt6701_spi_txrx_cplt_callback(SPI_HandleTypeDef *hspi) {
    // 释放片选
    HAL_GPIO_WritePin(MT6701_CS_PORT,MT6701_CS_PIN, GPIO_PIN_SET);

    // 解析14位原始角度值
    // 第1字节高6位 + 第2字节低8位 -> 右移2位
    uint16_t angle_raw = ((mt6701_rx_data[0] << 6) | (mt6701_rx_data[1] >> 2)) & 0x3FFFU;

    // 提取CRC并校验
    uint8_t crc_raw = mt6701_rx_data[2] & 0x3FU; // 提取低六位
    uint32_t crc_data = ((uint32_t) mt6701_rx_data[0] << 16) |
                        ((uint32_t) mt6701_rx_data[1] << 8) |
                        (uint32_t) mt6701_rx_data[2];
    crc_data >>= 6; // 去掉低6,保留18位有效数据

    if (mt6701_calculate_crc(crc_data) != crc_raw) {
        mt6701_spi_error_callback(hspi);
        return; // 立刻退出函数
    }

    // 转换为弧度制(0~2PI)
    encoder_angle = (float) angle_raw / (float) MT6701_RESOLUTION * MT6701_ANGLE_CYCLE;

    // 是否第一次启动
    //假设第一次读到的角度是 90°：第一次：encoder_angle_last = 90°，不算差值 第二次：读到 95° → 差值 = 95° - 90° = 5°
    static float encoder_angle_last = 0.0f;
    static uint8_t is_first_init = 1;
    if (is_first_init) {
        encoder_angle_last = encoder_angle;
        is_first_init = 0; // 保证只有第一次记录时算出的diff_angle = 0
    }

    // 计算差值,计算多圈累计角度
    float diff_angle = mt6701_cycle_diff(encoder_angle - encoder_angle_last,MT6701_ANGLE_CYCLE);
    encoder_angle_last = encoder_angle; // 更新上次角度
    // 限制累计角度在周期内
    motor_logic_angle += diff_angle;

    // 拉低片选,启动DMA,准备下一次中断战斗
    HAL_GPIO_WritePin(MT6701_CS_PORT,MT6701_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi, mt6701_rx_data, mt6701_rx_data, 3);
}

/**
 * @brief  SPI错误回调函数（MT6701通信异常处理）
 * @param  hspi: SPI句柄（仅响应SPI1）
 */
void mt6701_spi_error_callback(SPI_HandleTypeDef *hspi) {
    // 必须清除错误标志位,不然还报错
    __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
    __HAL_SPI_CLEAR_MODFFLAG(hspi);
    __HAL_SPI_CLEAR_OVRFLAG(hspi);

    // 拉低片选,启动DMA,准备下一次中断战斗
    HAL_GPIO_WritePin(MT6701_CS_PORT,MT6701_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(hspi, mt6701_rx_data, mt6701_rx_data, 3);
}

/**
 * @brief  角度差值归一化
 *         将原始角度差值折算到一个周期范围内，避免跨越0°/360°时出现大跳变。
 * @param  diff  原始角度差值
 * @param  cycle 角度周期（例如 2π 弧度）
 * @retval 归一化后的角度差值，范围为 [-cycle/2, cycle/2]
 *
 * @note   示例：当 diff = 350° - 10° = 340° 时，
 *         归一化结果为 -20°（最短路径差值）。
 */
float mt6701_cycle_diff(float diff, float cycle) {
    if (diff > cycle / 2.0f) {
        diff -= cycle;
    } else if (diff < -cycle / 2.0f) {
        diff += cycle;
    }
    return diff;
}

/**
 * @brief  计算MT6701的6位CRC校验值
 * @param  data: 待校验的18位数据（14位角度 + 4位磁强计数据）
 * @retval 6位CRC校验结果（0~63）
 */
uint8_t mt6701_calculate_crc(uint32_t data) {
    uint8_t crc = 0;
    const uint32_t polynomial = MT6701_CRC_POLY; // CRC多项式

    // 遍历18位数据逐位计算CRC
    for (int i = 17; i >= 0; i--) {
        uint8_t bit = (data >> i) & 0x01U; // 提取当前位
        crc <<= 1; // CRC左移1位
        if ((crc >> 6) ^ bit) // 最高位与当前位异或
        {
            crc ^= polynomial; // 异或多项式
        }
        crc &= 0x3FU; // 仅保留低6位（CRC结果）
    }
    return crc;
}
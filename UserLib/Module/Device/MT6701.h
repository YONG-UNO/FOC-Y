//
// Created by DingYong on 2026/2/1.
//
#ifndef MT6701_H
#define MT6701_H

#include <stdint.h>
#include <spi.h>

/* 宏定义 */
#define MT6701_CS_PIN         GPIO_PIN_4     // 片选引脚
#define MT6701_CS_PORT        GPIOA         // 片选端口
#define MT6701_RESOLUTION     16384U         // 角度分辨率:14bit(16384)
#define MT6701_ANGLE_CYCLE    (2*PI)         // 角度周期:2PI
#define MT6701_CRC_POLY       0x43U          // CRC校验多项式：X^6 + X + 1

/*全局变量*/
extern uint8_t mt6701_rx_data[3]; // MT6701接收数据缓存(3字节)
extern float encoder_angle; // 原始机械角度(0~2pi弧度)(编码器输出的角度)
extern float motor_logic_angle; // 多圈累计逻辑角度
extern float position_cycle; // 逻辑角度周期(电机一圈的角度)

/*函数*/
void mt6701_init(void); // 初始化
float mt6701_cycle_diff(float diff, float cycle); // 角度周期差值计算(解决0°/360°跳变)
uint8_t mt6701_calculate_crc(uint32_t data); // 计算MT6701的6位CRC校验值
void mt6701_spi_txrx_cplt_callback(SPI_HandleTypeDef *hspi); // SPI DMA收发完成回调函数(MT6701角度解析)
void mt6701_spi_error_callback(SPI_HandleTypeDef *hspi); // SPI错误回调函数(MT6701通信异常处理)

#endif //MT6701_H

/*
 * 文件：Device/BMI088/inc/BMI088Middleware.h
 * 作用：声明 BMI088 使用的底层通信、片选和延时接口。
 * 来源：达妙 MC02 的 CtrBoard-H7_IMU 示例。
 * 用法：BMI088_USE_SPI 选择当前 SPI 分支；实现位于 BMI088Middleware.c。
 * 注意：声明接口不等于完成硬件初始化；IIC 分支当前没有实现。
 * 整理：统一 UTF-8，补充中文说明；本次不改变可执行代码。
 */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "stdint.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif

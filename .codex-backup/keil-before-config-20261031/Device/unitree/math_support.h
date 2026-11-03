/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：DM02_control-main/dm02_motor/User_File/algorithm/math_support/math_support.h
 * 作用：通用数学与单位辅助。
 * 用途：提供量化、角度、字节序与限幅相关辅助；依赖 CMSIS 数学库，保留原实现。
 * 主要符号（节选）：uint_to_float、float_to_uint_f、float_to_uint_i、motor_max_min、LimitMax、Math_Endian_Reverse_16、chassis_math_angle_set_ref。
 * 依赖：struct_typedef.h、arm_math.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
#ifndef MATH_SUPPORT_H
#define MATH_SUPPORT_H

#include "struct_typedef.h"

#include "arm_math.h"

#define PI 3.14159265358979323846f
extern const float MATH_RPM_TO_RADPS;
extern const float MATH_DEG_TO_RAD;
extern const float MATH_CELSIUS_TO_KELVIN;
extern const float PI2;

template<typename Type>
Type Basic_Math_Abs(Type x)
{
    return ((x > 0) ? x : -x);
}


/**
  * @brief          int 转 fp32 
  * @param[in]      x_int 要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */

fp32 uint_to_float(uint16_t x_int, fp32 x_min, fp32 x_max, uint8_t bits);

/**
  * @brief          fp32 转 int 
  * @param[in]      x     要转换的数据
  * @param[in]      x_min 要转换的数据的最小值
  * @param[in]      x_max 要转换的数据的最大值
  * @param[in]      bits  要转换的数据位数
  * @retval         none
  */
uint16_t float_to_uint_f(fp32 x, fp32 x_min, fp32 x_max, uint8_t bits);

uint16_t float_to_uint_i(fp32 x, uint16_t x_min, uint16_t x_max, uint8_t bits);

fp32 motor_max_min(fp32 proto,fp32 max,fp32 min);

void LimitMax(fp32 input,fp32 max);

uint16_t Math_Endian_Reverse_16(void *Source, void *Destination);

fp32 chassis_math_angle_set_ref(fp32 ref,fp32 set);

#endif

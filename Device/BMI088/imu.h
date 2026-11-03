#ifndef _IMU_H
#define _IMU_H

#include <stdbool.h>
#include "dog.h"

typedef enum
{
    Roll = 0,
    Pitch,
    Yaw
} IMU;

#ifdef __cplusplus
extern "C" {
#endif

/* 输出单位为度：ZYX 欧拉角，Pitch 为 [-90, 90]，Yaw 为连续相对角。 */
extern float imu_angle[3];

/**
************************************************************************
* @brief:      imu_data_process - 将 BMI088 的一组角速度、加速度转换为姿态角
* @param:      gyro - 三元素机身角速度数组，单位 rad/s，零偏由调用方处理
* @param:      accel - 同一机身坐标系的三元素加速度数组，单位 m/s²
* @param:      dt_s - 相邻有效采样的实际时间间隔，单位秒
* @retval:     true - 姿态已更新；false - 输入无效，输出保持不变
* @details:    X 向前、Y 向左、Z 向上，首次采样保持静止；每组采样调用一次。
*              初始航向为零，输出写入 imu_angle。明显的运动加速度会影响
*              倾角结果，连续偏航长期会漂移。本接口不读取 SPI。
************************************************************************
**/
bool imu_data_process(const float gyro[3], const float accel[3], float dt_s);

/**
************************************************************************
* @brief:      get_imu_angle - 读取指定轴的姿态角缓存
* @param:      axis - 有效枚举值 Roll、Pitch 或 Yaw
* @retval:     对应轴角度，单位度
* @details:    在首次有效解算后使用；只取值，不执行采样或积分。
************************************************************************
**/
float get_imu_angle(IMU axis);

/**
************************************************************************
* @brief:      Euler_Update - 向动作和步态代码发布最新姿态
* @param:      euler - 控制层姿态结构体指针
* @retval:     void
* @details:    使用 Update_Yaw 作为航向零点；每次调用更新 Last_*，
*              不执行积分，也不改变 Target_Yaw。
************************************************************************
**/
void Euler_Update(Euler *euler);

/**
************************************************************************
* @brief:      Yaw_Reset - 将当前朝向设为软件航向零点
* @param:      euler - 控制层姿态结构体指针
* @retval:     void
* @details:    更新 Update_Yaw，清零 Yaw、Last_Yaw；不改变原始连续偏航角、
*              内部四元数或 Target_Yaw，不执行芯片复位和零偏标定。
************************************************************************
**/
void Yaw_Reset(Euler *euler);

#ifdef __cplusplus
}
#endif

#endif

/*
 * 工程文件说明
 * 文件：DM02_control-main/dm02_motor/User_File/task/motor_task.h
 * 作用：宇树电机任务入口声明。
 * 用途：声明 motor_task_init 和 motor_task，供控制任务调用。
 * 主要符号（节选）：motor_task_init、motor_task。
 * 依赖：struct_typedef.h。
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#ifdef __cplusplus

extern "C"
{
#endif
	
#include "struct_typedef.h"

#define MOTOR_COUNT 8U

/* 上层统一使用减速器输出端的 rad、rad/s、N·m，槽号不是通信 ID。 */
typedef struct
{
    float Pos;
    float W;
    float T;
} Motor_Value;

typedef struct
{
    Motor_Value target;
    Motor_Value measure;
    float K_p;                 // 输出端 N·m/rad
    float K_w;                 // 输出端 N·m/(rad/s)
    uint8_t command_mode;       // 0：锁定；1：FOC
    uint8_t mode;               // 电机实际反馈模式
    uint8_t feedback_valid;     // 未收到合法反馈的槽保持为0
    uint8_t error;
    float temperature;
} Motor_Control;

extern Motor_Control motor[MOTOR_COUNT];

struct Pos_motor;
/* 仅设置全部逻辑槽的增益，不发送、不使能、不叠加 Tfb。 */
void Set_All_Motor_Param(const struct Pos_motor *param);

// void motor_task_init();
uint8_t motor_task_init(void);
void motor_task();
//两个测试函数
void motor_test(void);
void motor_control_text();
#ifdef __cplusplus
};

#endif

#endif

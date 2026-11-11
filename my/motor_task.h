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
/* 各运行方式均用 command_mode=1（FOC）；P/W/T 指 target.Pos/W/T。
 * 位控：P=目标位置，W=目标速度（保持时为0），T=0，K_p>0，K_w>=0（提供阻尼）。
 * 速度：W=目标速度，T=0，K_p=0，K_w>0；P不起作用。
 * 阻尼：W=T=K_p=0，K_w>0；P不起作用，不保持原位置。
 * 力矩（含足端VMC）：T=目标力矩，K_p=K_w=0；P/W不起作用。
 * 零力矩：T=K_p=K_w=0；P/W不起作用，不保持位置。
 * 混控：P/W=目标位置/速度，T=前馈力矩，K_p>0，K_w>=0。
 * 输出力矩=T+K_p*(P-反馈位置)+K_w*(W-反馈速度)。
 * 以下为输出轴端的协议表示范围，不是实际可用上限：
 * P约±65051 rad（多圈），W约±127 rad/s，T约±810 N·m；
 * K_p约0~1025.72 N·m/rad，K_w约0~1025.72 N·m/(rad/s)。
 * 厂家最大输出力矩23.7 N·m、最高转速30 rad/s（24V），非持续同时工作点。
 * 当前遥控测试：W限±0.5 rad/s，ID0/1的T限±0.9/±0.8 N·m，内置增益均为0。
 * 装腿后关节角约束：髋[-60,+60]°，相对膝[-150,+30]°；须经标定转换到P。
 * 上述协议范围、关节角约束是说明，本结构体及发送层不自动实施限幅。
 */
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

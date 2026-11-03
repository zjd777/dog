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

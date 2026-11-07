/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：DM02_control-main/dm02_motor/User_File/task/control_task.h
 * 作用：原工程任务入口。
 * 用途：注册遥控并提供定时器控制分发；电机初始化/使能当前存在注释关闭的代码。
 * 主要符号（节选）：control_task_init、control_task。
 * 依赖：无直接 include；由使用方按声明/宏配置引入。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus

extern "C"
{
#endif
	
uint8_t control_task_init(void);
void control_task ();
#ifdef __cplusplus
};

#endif

#endif

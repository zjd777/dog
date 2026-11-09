/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/obstacle_course.h
 * 作用：障碍赛遥控动作映射。
 * 用途：将原遥控通道映射到步态参数、方向和状态请求；通道标定与动作绑定属于原机。
 * 主要符号（节选）：GamePad_Control_Obs、GamePad_Control_Country。
 * 依赖：无直接 include；由使用方按声明/宏配置引入。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：obstacle_course.h
 * 用途：障碍赛遥控动作编排。
 * 说明：声明障碍赛与越野模式的遥控处理入口。
 */
#ifndef _OBSTACLE_COURSE_H
#define _OBSTACLE_COURSE_H

#ifdef __cplusplus
extern "C"{
#endif

void GamePad_Control_Obs(void);
void GamePad_Control_Country(void);

#ifdef __cplusplus
}
#endif
#endif


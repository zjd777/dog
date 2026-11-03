/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/pos_ctrl.h
 * 作用：足端位置轨迹控制。
 * 用途：把轨迹生成、逆运动学与电机位置目标串接；原几何和方向参数不可直接用于新机构。
 * 主要符号（节选）：line_Pos、Cycloid_Pos、Bezier_Pos、Bezier_Stance_Pos、gait_posctrl、Walk_Away_Posctrl、Stand_Up_Posctrl。
 * 依赖：stdint.h、dog.h、walk.h、action.h、imu.h、jump.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：pos_ctrl.h
 * 用途：足端位置控制。
 * 说明：声明位置控制轨迹、步态、站蹲、跳跃及控制模式切换接口，并导出电机位置环参数。
 */
#ifndef _POS_CTRL_H
#define _POS_CTRL_H

#include "stdint.h"
#include "dog.h"
#include "walk.h"
#include "action.h"
#include "imu.h"
#include "jump.h"

void line_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float T, float t);
void Cycloid_Pos(uint8_t legid, float start_x, float start_z, float end_x, float max_z, float T, float t);
void Bezier_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t);
void Bezier_Stance_Pos(uint8_t legid, float start_x, float start_z, float end_x, float end_z, float max_z, float Delta_x, float Delta_z, float T, float t);
void gait_posctrl(uint8_t legid, GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t);
void Walk_Away_Posctrl(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t);
void Stand_Up_Posctrl(float t, float tm, float *h);
void Squat_Down_Posctrl(float t, float tm, float *x, float *z);
void Keep_All_Leg_Posctrl(float X0, float X1, float X2, float X3, float Z0, float Z1, float Z2, float Z3);
bool Jump_s_pos(uint8_t legid, JumpParam *Jump_Param, action_param *Action_Param, Euler *Euler_Angle, float t);
void Change_Ctrl_State(Ctrl_State state);
Ctrl_State Get_Now_Ctrl_State(void);

extern Pos_motor prep_param;
extern Pos_motor jump_param;
extern Pos_motor luanch_param;
#endif

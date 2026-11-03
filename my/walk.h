/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/walk.h
 * 作用：四腿步态与方向修正。
 * 用途：管理摆动/支撑相、轨迹时序及方向/姿态修正；依赖原模型和实时反馈。
 * 主要符号（节选）：Set_Target_Yaw、Set_Walk_Dire、Set_Walk_Length、Pitch_Pos_Offest、Yaw_Pos_Crol、Set_Turning_Kp、Walk_Away_Posctrl。
 * 依赖：stdint.h、dog.h、action.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：walk.h
 * 用途：行走步态与姿态修正。
 * 说明：定义行走方向和步态参数，声明航向、步长、姿态修正及行走控制接口。
 */
#ifndef _WALK_H
#define _WALK_H
#include "stdint.h"
#include "dog.h"
#include "action.h"

typedef enum
{
    Stance = 0,
    Swing = 1,
} leg_state;

typedef enum
{
    First = 0,
    Later = 1,
    Stop = 2,
} walk_state;

typedef enum
{
    Front = 0,
    Back = 1,
} dire_state;
/*
typedef enum
{
    Stay = 0,
    Forward,
    Back,
    Left,
    Right
} GaitDirection; // 行走方向
*/
typedef enum
{
    Gait_None = 0,
    Trot,
    LeftTrot,         // 原地左转
    RightTrot,        // 原地右转
    LeftForwardTrot,  // 前进左转
    RightForwardTrot, // 前进右转
    LeftBackTrot,     // 后退左转
    RightBackTrot,    // 后退右转
    StepUpMove,
    LowTrot, // 低身步态,步高、步长较小
    UpHill,
    HighTrot, // 过双木桥用
    UpStep,   // 上台阶用
} Gait;       // 步态

typedef enum
{
    None = 0,  // 不走斜坡
    Upslope,   // 上坡
    Downslope, // 下坡
} scope_state; // 走斜坡状态

typedef struct
{
    float Axis_Offset; // x方向中轴偏移量
    float Std_Height;  // 走斜坡时最小站立高度
} slope;
typedef struct
{
    leg_state Leg_State[4]; // 腿的状态(支撑/摆动相)
    walk_state Walk_State;  // walk状态(起步/运动/停止)
    dire_state Dire;        // 往前还是往后走
    float stance_height;    // 站立高度 (m)
    float step_length[4];   // 步长 (m)
    float std_step_length;  // 标准步长,方便赋值(m)
    float step_height;      // 步高 (m)
    float step_height_down; // 支撑腿下沉步高
    float swing_percent;    // 摆动相百分比,指一个摆动周期内摆动的时间
    float term;             // 周期 (s)

    uint32_t Cross_Index[4]; // 每条腿迈步次数
    float t;                 // 当前时间
    float last_t;            // 暂存上次的时间,用于支撑摆动腿切换

    PD VMC_Swing_Walk[4];  // VMC摆动相参数
    PD VMC_Stance_Walk[4]; // VMC支撑相参数
    PD VMC_Euler_Walk;     // VMC姿态控制参数
    float Yaw_max;         // 偏航角矫正最大补偿修正

    Pos_motor Posctrl_Swing;  // Swing电机位控参数
    Pos_motor Posctrl_Stance; // Stance电机位控参数

    slope Scope;
} GaitParam;

extern GaitParam gait_param;
void Set_Target_Yaw(float Target_Yaw);
void Set_Walk_Dire(dire_state Dire);
void Set_Walk_Length(float Walk_Length);
void Pitch_Pos_Offest(Euler *Euler, GaitParam *gait_param);
void Yaw_Pos_Crol(Euler *Euler, GaitParam *gait_param, float Max_Changable_Step_Length);
void Set_Turning_Kp(float Kp);

void Walk_Away_Posctrl(GaitParam *gait_param, action_param *Action_Param, Euler *Euler_Angle, float t);
#endif

/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/fsm.h
 * 作用：原机器人有限状态机。
 * 用途：通过状态请求与切换许可调用 enter/run/exit；具体动作由各业务模块提供。
 * 主要符号（节选）：DogFSM、ChangeDogState、GetNowDogState、GetLastDogState、GetNextDogState、SetDogChangeAble、GetDogWhetherChangeAble。
 * 依赖：stm32h7xx.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：fsm.h
 * 用途：机器人有限状态机。
 * 说明：定义动作状态枚举、状态回调结构、切换接口和各状态回调声明。
 */
#ifndef _FMC_H

#define _FMC_H
#include "stm32h7xx.h"
typedef enum // 这里用枚举量定义了狗的状态
{
    Start = 0,   // 初始模式
    Keep,        // 保持关节位置    
    Squat,       // 蹲着
    Stand,       // 站立模式，起立
    Walk,        // 行走
    Jump,        // 跳跃
    Backflip,    // 后空翻
    CrossLeg,    // 跨腿
    Line,        // 足端走直线,用于调整重心
    JumpHurdles, // 跨栏
    Test1,
    Test2,
    Test3,
} DOG_State;

typedef struct
{
    void (*enter)();     // 创建一个enter的函数指针,指向状态机进入函数的地址
    void (*run)();       // 创建一个run的函数指针,指向状态机执行函数的地址
    void (*exit)();      // 创建一个exit的函数指针,指向状态机退出函数的地址
    DOG_State now_state; // 当前动作状态
    DOG_State next_state; // 待切换的目标状态
    DOG_State last_state; // 上一个动作状态
    FunctionalState change_able; // 是否允许退出当前状态并切换
    FunctionalState execute_able; // 是否存在待执行的切换请求
} FSM_State;
#ifdef __cplusplus
extern "C" {
#endif
void DogFSM(void);
// 提交切换请求；由 DogFSM 在获得切换许可后执行退出、进入和运行回调。
void ChangeDogState(DOG_State x);
DOG_State GetNowDogState(void);
DOG_State GetLastDogState(void);
DOG_State GetNextDogState(void);
void SetDogChangeAble(FunctionalState able);
FunctionalState GetDogWhetherChangeAble(void);
// 获取是否要切换状态
FunctionalState GetDogWhetherExecuteAble(void);

void start_enter(void);
void start_exit(void);
void start_run(void);

void keep_enter(void);
void keep_exit(void);
void keep_run(void);

void squat_enter(void);
void squat_exit(void);
void squat_run(void);

void stand_enter(void);
void stand_exit(void);
void stand_run(void);

void walk_enter(void);
void walk_exit(void);
void walk_run(void);

void jump_enter(void);
void jump_exit(void);
void jump_run(void);

void backflip_enter(void);
void backflip_exit(void);
void backflip_run(void);

void crossleg_enter(void);
void crossleg_exit(void);
void crossleg_run(void);

void line_enter(void);
void line_exit(void);
void line_run(void);

void test1_enter(void);
void test1_exit(void);
void test1_run(void);

void test2_enter(void);
void test2_exit(void);
void test2_run(void);

void test3_enter(void);
void test3_exit(void);
void test3_run(void);

void jump_hurdles_enter(void);
void jump_hurdles_exit(void);
void jump_hurdles_run(void);

#ifdef __cplusplus
}
#endif

#endif

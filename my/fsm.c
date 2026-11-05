/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/fsm.c
 * 作用：原机器人有限状态机。
 * 用途：通过状态请求与切换许可调用 enter/run/exit；具体动作由各业务模块提供。
 * 主要符号（节选）：ChangeDogState、GetNowDogState、GetLastDogState、GetNextDogState、SetDogChangeAble、GetDogWhetherChangeAble、GetDogWhetherExecuteAble。
 * 依赖：fsm.h、stdint.h、A1_motor.h、dog.h、timer.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：fsm.c
 * 用途：机器人有限状态机。
 * 说明：建立动作状态与回调函数的对应表，管理切换请求和切换许可，并在周期调用中执行状态回调。
 */
#include "fsm.h"
#include "stdint.h"
// #include "A1_motor.h"
#include "dog.h"
// #include "timer.h"

// 数组下标与 fsm.h 的状态枚举一一对应；例如 StateMap[Keep] 绑定保持状态的进入、运行、退出回调。
//里面是三个状态函数，以及五个参数,,0=start 0=disabls
FSM_State StateMap[] =
    {
        {start_enter, start_run, start_exit, Start, Start, Start, ENABLE, DISABLE},
        {keep_enter, keep_run, keep_exit},
        {squat_enter, squat_run, squat_exit},
        {stand_enter, stand_run, stand_exit},
        {walk_enter, walk_run, walk_exit},
        {jump_enter, jump_run, jump_exit},
        {backflip_enter, backflip_run, backflip_exit},
        {crossleg_enter, crossleg_run, crossleg_exit},
        {line_enter, line_run, line_exit},
        {jump_hurdles_enter, jump_hurdles_run, jump_hurdles_exit}, // 跨栏
        {test1_enter, test1_run, test1_exit},
        {test2_enter, test2_run, test2_exit},
        {test3_enter, test3_run, test3_exit},
};

FSM_State *pCurrentState = &StateMap[Start]; // 一个存储当前状态的指针,可以随时切换

// 提交目标状态并置位切换请求；本函数不立即切换，实际切换由 DogFSM 周期执行。
//切换状态的请求函数，x 是要切换的状态
//切换的顺序是提出 请求，允许请求，再切换
void ChangeDogState(DOG_State x)
{
    if (pCurrentState->now_state != x)
    {
        pCurrentState->next_state = x;
        pCurrentState->execute_able = ENABLE;
    }
}

// 获取当前状态
DOG_State GetNowDogState()
{
    return pCurrentState->now_state;
}

// 获取上一个状态
DOG_State GetLastDogState()
{
    return pCurrentState->last_state;
}

// 获取下一个待切换的状态
DOG_State GetNextDogState()
{
    return pCurrentState->next_state;
}

// 设置切换许可；许可和请求均为 ENABLE 且目标不同于当前状态时，DogFSM 才切换。
void SetDogChangeAble(FunctionalState able)
{
    pCurrentState->change_able = able;
}

// 获取是否可切换状态
FunctionalState GetDogWhetherChangeAble()
{
    return pCurrentState->change_able;
}

// 获取是否要切换状态
FunctionalState GetDogWhetherExecuteAble()
{
    return pCurrentState->execute_able;
}

//这个是放在循环里面随时检测的切换函数，同时也是每个状态下的执行函数
void DogFSM()
{
    if ((pCurrentState->change_able == ENABLE) && (pCurrentState->execute_able == ENABLE) 
                && (pCurrentState->next_state != pCurrentState->now_state)) // 切换状态
    {
        DOG_State now_state = pCurrentState->now_state;   // 保存当前状态
        DOG_State next_state = pCurrentState->next_state; // 保存下一个状态

        pCurrentState->exit(); // 执行当前状态的退出函数

        pCurrentState = &StateMap[pCurrentState->next_state]; // 指向目标状态条目；随后显式更新状态记录和切换标志。

        pCurrentState->last_state = now_state;  // 更新上一个状态
        pCurrentState->now_state = next_state;  // 更新当前状态
        pCurrentState->next_state = next_state; // 更新下一个状态
        pCurrentState->change_able = DISABLE;   // 禁止切换状态
        pCurrentState->execute_able = DISABLE;

        ////参数设置完就开始挨个执行动作函数

        pCurrentState->enter(); // 执行当前状态的进入函数
        pCurrentState->run();   // 循环执行run
    }
    else
    {
        pCurrentState->run(); // 循环执行run
    }
}

__weak void start_enter() {}
__weak void start_exit() {}
__weak void start_run() {}

__weak void passive_enter() {}
__weak void passive_exit() {}
__weak void passive_run() {}

__weak void keep_enter() {}
__weak void keep_exit() {}
__weak void keep_run() {}

__weak void squat_enter() {}
__weak void squat_exit() {}
__weak void squat_run() {}

__weak void stand_enter() {}
__weak void stand_exit() {}
__weak void stand_run() {}

__weak void walk_enter() {}
__weak void walk_exit() {}
__weak void walk_run() {}

__weak void jump_enter() {}
__weak void jump_exit() {}
__weak void jump_run() {}

__weak void backflip_enter() {}
__weak void backflip_exit() {}
__weak void backflip_run() {}

__weak void crossleg_enter() {}
__weak void crossleg_exit() {}
__weak void crossleg_run() {}

__weak void line_enter() {}
__weak void line_exit() {}
__weak void line_run() {}

__weak void test1_enter() {}
__weak void test1_exit() {}
__weak void test1_run() {}

__weak void test2_enter() {}
__weak void test2_exit() {}
__weak void test2_run() {}

__weak void test3_enter() {}
__weak void test3_exit() {}
__weak void test3_run() {}

__weak void jump_hurdles_enter() {}
__weak void jump_hurdles_exit() {}
__weak void jump_hurdles_run() {}

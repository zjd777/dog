/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：DM02_control-main/dm02_motor/User_File/task/control_task.cpp
 * 作用：原工程任务入口。
 * 用途：注册遥控并提供定时器控制分发；电机初始化/使能当前存在注释关闭的代码。
 * 主要符号（节选）：Task1ms_Callback、Task500us_Callback、Task25ns_Callback、control_task_init、control_task、HAL_TIM_PeriodElapsedCallback。
 * 依赖：control_task.h、math_support.h、motor_task.h、remote_control.h、tim.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
#include "control_task.h"
#include "math_support.h"
#include "motor_task.h"
#include "remote_control.h"
#include "bsp_sbus.h"
#include "tim.h"

/**
 * @brief 1ms的回调函数
 * @return void
 */
 
bool control_system_init_bl;
void Task1ms_Callback(){
	
	if(control_system_init_bl)motor_task();
    //测试函数
	// if(control_system_init_bl)motor_control_text();

	
}

/**
 * @brief 500us的回调函数
 * @return void
 */
void Task500us_Callback (){

}


void Task25ns_Callback (){

}

/**
 * @brief 控制任务的初始化
 * @return void
 */
void control_task_init(){
    control_system_init_bl = 0;
	remote_control_init();
	if(motor_task_init()==0u)
        return;
   HAL_TIM_Base_Start_IT(&htim8);
   control_system_init_bl=1;
}


/**
 * @brief 控制任务
 * @return void
 */
void control_task (){
	SBUS_Handle();
	motor_control_text();

}


/**
 * @brief 定时器的中断回调函数
 * @param htim   定时器TIM_HandleTypeDef的指针
 * @return void
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        Task500us_Callback();    //500ns的定时
    }
    else if (htim->Instance == TIM8)
    {
        Task1ms_Callback();   //1ms的定时
    }

}



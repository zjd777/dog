/*
 * 文件：DM02_control-main/dm02_motor/User_File/task/motor_task.cpp
 * 作用：宇树电机任务，保留原有 RS485 接收、初始化及周期发送。
 * 用途：USART2 接收 ID 0 的宇树反馈；motor_task_init 初始化，motor_task 周期发送。
 * 依赖：motor_task.h、motor_YS.h、crc_ccitt.h、tim.h。
 * 接口：motor[] 统一保存上层目标和反馈，在本任务边界换算力矩与增益。
 * 编码：UTF-8。
 */
#include "motor_task.h"
#include "dog.h"
#include "motor_YS.h"
#include "crc_ccitt.h"
#include "tim.h"
#include "bsp_sbus.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//宇树电机类和控制所需数据
Class_Motor_YS motor_YS[8];
Motor_Control motor[MOTOR_COUNT] = {};
// motor_control_text使用的测试槽：0为髋电机，1为膝电机。
static constexpr float motor_ys_gear_ratio = 6.33f;

void motor_YS_Data_send(Class_Motor_YS *__motor_YS, const Motor_Control *data);
void motor_YS_Data_recv(Class_Motor_YS *__motor_YS, Motor_Control *data);

void Set_All_Motor_Param(const Pos_motor *param)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    {
        motor[i].K_p = param->kp;
        motor[i].K_w = param->kw;
    }
}


/**
 * @brief 串口2的回调函数
 * @param Header 串口指针
 * @param *Buffer 串口收数据数组
 * @return void
 */
void USART2_RxHandler(UART_HandleTypeDef *Header, uint8_t *Buffer){
    if (Buffer[0] != 0xFD || Buffer[1] != 0xEE)
        return;
    uint16_t crc = crc_ccitt(Buffer,RS485_recv_Data_N-2);
    if((Buffer[14] != (crc&0xFF)) || (Buffer[15] != ((crc>>8) & 0xFF))){
        return;
    }
    uint8_t User_Rx_usart_id = Buffer[2]&0xF;
    switch (User_Rx_usart_id )
    {
    case  0:
        motor_YS[0].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[0], &motor[0]);
    break;
    case  1:
        motor_YS[1].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[1], &motor[1]);
    break;
    case  2:
        motor_YS[2].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[2], &motor[2]);
    break;
    case  3:
        motor_YS[3].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[3], &motor[3]);
    break;       
    case  4:
        motor_YS[4].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[4], &motor[4]);
    break;    
       case  5:
        motor_YS[5].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[5], &motor[5]);
    break;
        case  6:
        motor_YS[6].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[6], &motor[6]);
    break;
        case  7:
        motor_YS[7].UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS[7], &motor[7]);
    break;
    default:
        break;
    }
}

/**
 * @brief 串口3的回调函数
 * @param Header 串口指针
 * @param *Buffer 串口收数据数组
 * @return void
 */
void USART3_RxHandler(UART_HandleTypeDef *Header, uint8_t *Buffer){
 
}

/**
 * @brief 宇数电机的初始化
 * @param huart uart的指针
 * @param __motor_YS 电机的类指针
 * @param id 电机id
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
// void motor_YS_Init(UART_HandleTypeDef *huart,Class_Motor_YS *__motor_YS,uint8_t id,Motor_Control *data){
//      __motor_YS->Init(huart,id,Motor_YS_Pos_control);
//     //  motor_ys_feedback_received = false;
//      __motor_YS->enable();
//      HAL_Delay(10);
//      while(__motor_YS->Get_Angle() ==0){
//         __motor_YS->enable();
//         HAL_Delay(1);
//     }
//     motor_YS_Data_recv(__motor_YS,data);
//     __motor_YS->zero();
//     __motor_YS->enable();
//     HAL_Delay(1);
// }

bool motor_YS_Init(UART_HandleTypeDef *huart, Class_Motor_YS *__motor, uint8_t id, Motor_Control *data){
    __motor->Init(huart, id, Motor_YS_Pos_control);
    data->feedback_valid = 0U;

    const uint32_t start_tick = HAL_GetTick();
    const uint32_t timeout_ms = 500U;  // 示例超时时间

    while (!data->feedback_valid)
    {
        if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms)
        {
            __motor->lose();  // 尝试发送失能帧
            return false;
        }

        __motor->enable();
        HAL_Delay(1);
    }

    motor_YS_Data_recv(__motor, data);

    // 收到反馈后，再检查电机是否报告故障
    if (data->error != 0U)
    {
        __motor->lose();
        return false;
    }

    data->command_mode = 1U; // 沿用现有初始化成功后使能的行为。
    return true;
}

/**
 * @brief 宇数电机的接收数据传递
 * @param __motor_YS 电机的类指针
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
//电机反馈的实时信息，后面可用来做闭环控制
void motor_YS_Data_recv(Class_Motor_YS *__motor_YS, Motor_Control *data){
    data->error = __motor_YS->Get_MError();
    data->mode = __motor_YS->Get_mode();
    // 底层已将位置、速度换算到输出端，只有力矩仍为转子端。
    data->measure.Pos = __motor_YS->Get_Now_Pos();
    data->measure.W = __motor_YS->Get_Now_W();
    data->measure.T = __motor_YS->Get_Now_T() * motor_ys_gear_ratio;
    data->temperature = __motor_YS->Get_Now_Temperature();
    data->feedback_valid = 1U;
}

/**
 * @brief 宇树电机的发送数据传递
 * @param __motor_YS 电机的类指针
 * @param data 电机发送数据(外部定义方便传递数据)
 * @return void
 */
void motor_YS_Data_send(Class_Motor_YS *__motor_YS, const Motor_Control *data){
    __motor_YS->Set_Pos(data->target.Pos);
    __motor_YS->Set_W(data->target.W);
    // 宇树 SDK 使用转子端力矩、增益；位置和速度的减速比已由底层处理。
    __motor_YS->Set_T(data->target.T / motor_ys_gear_ratio);
    __motor_YS->Set_Kp(data->K_p / (motor_ys_gear_ratio * motor_ys_gear_ratio));
    __motor_YS->Set_Kd(data->K_w / (motor_ys_gear_ratio * motor_ys_gear_ratio));
    __motor_YS->Set_Status(data->command_mode == 1U ?
                         Motor_YS_Status_ENABLE : Motor_YS_Status_DISABLE);
}

/**
 * @brief 电机的初始化
 * @return void
 */
// void motor_task_init(){
uint8_t motor_task_init(void)
{
    USART_RX485_init(&huart2, USART2_RxHandler);

    for (uint8_t id = 0U; id < 2U; ++id)
    {
        if (!motor_YS_Init(&huart2, &motor_YS[id], id, &motor[id]))
            return 0U;
    }

    return 1U;
}

/**
 * @brief 电机任务
 * @return void
 */
 //在这里面把数据处理好之后发给电机,再中断里被调用
void motor_task(){
    static uint8_t index = 0U;
    if (index == 2U)
        index = 0U;

    motor_YS_Data_send(&motor_YS[index], &motor[index]);
    motor_YS[index].UART_send();
    index++;
}

void motor_test(void)
{
    extern bool control_system_init_bl;

    if (!control_system_init_bl ||
        __get_IPSR() != 0U ||
        __get_PRIMASK() != 0U)
    {
        return;
    }

    const float torque_limit[2] = {0.3f, 0.5f}; // ID0、ID1输出轴力矩限幅，N·m
    const float hold_kp = 5.0f; // 输出轴 N·m/rad
    const float hold_kd = 0.2f; // 输出轴 N·m/(rad/s)
    const uint32_t hold_ms = 10000U;
    const uint32_t pause_ms = 1000U;

    // 一次性设置参数，避免定时中断读取到中间状态
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint8_t id = 0U; id < 2U; ++id)
    {
        motor[id].target.Pos = 0.0f;
        motor[id].target.T = 0.0f;
        motor[id].target.W = 0.0f;
        motor[id].K_p = 0.0f;
        motor[id].K_w = 0.0f;
        motor[id].command_mode = 1U;
    }

    __set_PRIMASK(primask);

    HAL_Delay(pause_ms);

    for (uint8_t id = 0U; id < 2U; ++id)
    {
        motor[id].target.Pos = motor[id].measure.Pos;
        const uint32_t start_tick = HAL_GetTick();

        while ((uint32_t)(HAL_GetTick() - start_tick) < hold_ms)
        {
            const float error = motor[id].target.Pos - motor[id].measure.Pos;
            float torque = hold_kp * error - hold_kd * motor[id].measure.W;

            if (torque > torque_limit[id])
                torque = torque_limit[id];
            else if (torque < -torque_limit[id])
                torque = -torque_limit[id];

            motor[id].target.T = torque;
            HAL_Delay(2U);
        }

        motor[id].target.T = 0.0f;
        HAL_Delay(pause_ms);
    }
}


void motor_control_text()
{
    static uint32_t last_update_tick = 0U;
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_update_tick) < 2U)
        return;
    last_update_tick = now;

    const float max_speed = 0.5f; // 输出轴 rad/s
    const float speed_gain = 1.6f; // 软件速度误差增益，输出轴 N·m/(rad/s)
    const float torque_limit[2] = {0.3f, 0.5f}; // ID0、ID1输出轴 N·m
    const bool enabled = SBUS_IsOnline() && g_sbus_channels[5] == 2000;
    const float stick[2] = {
        g_sbus_channels[0] - 1500.0f,
        g_sbus_channels[1] - 1500.0f
    };
    const bool outside_deadzone =
        stick[0] * stick[0] + stick[1] * stick[1] >= 200.0f * 200.0f;

    for (uint8_t id = 0U; id < 2U; ++id)
    {
        float target_speed = enabled && outside_deadzone ?
                             max_speed * stick[id] / 500.0f : 0.0f;
        if (target_speed > max_speed)
            target_speed = max_speed;
        else if (target_speed < -max_speed)
            target_speed = -max_speed;

        // 允许控制时回中做零速制动；SWB低档或离线只归零力矩。
        float torque = enabled ?
                       speed_gain * (target_speed - motor[id].measure.W) : 0.0f;
        if (torque > torque_limit[id])
            torque = torque_limit[id];
        else if (torque < -torque_limit[id])
            torque = -torque_limit[id];

        // 内置PD关闭，全部控制力矩都经过上面的软件限幅。
        motor[id].K_p = 0.0f;
        motor[id].K_w = 0.0f;
        motor[id].command_mode = 1U;
        motor[id].target.Pos = motor[id].measure.Pos;
        motor[id].target.W = target_speed;
        motor[id].target.T = torque;
    }
}



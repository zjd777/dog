/*
 * 文件：DM02_control-main/dm02_motor/User_File/task/motor_task.cpp
 * 作用：宇树电机任务，保留原有 RS485 接收、初始化及周期发送。
 * 用途：USART2 接收 ID 1 的宇树反馈；motor_task_init 初始化，motor_task 周期发送。
 * 依赖：motor_task.h、motor_YS.h、crc_ccitt.h、tim.h。
 * 整理：仅剔除灵足、大疆、达妙电机及其 CAN 相关内容，宇树现有逻辑保留。
 * 编码：UTF-8。
 */
#include "motor_task.h"
#include "motor_YS.h"
#include "crc_ccitt.h"
#include "tim.h"
#include "bsp_sbus.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//宇树电机类和控制所需数据
Class_Motor_YS motor_YS;
motor_YS_control motor_ys_data;
// 中断写入，初始化函数读取
static volatile bool motor_ys_feedback_received = false;

void motor_YS_Data_send(Class_Motor_YS *__motor_YS,Struct_send_motor_YS data);
void motor_YS_Data_recv(Class_Motor_YS *__motor_YS,Struct_recv_motor_YS *data);


/**
 * @brief 串口2的回调函数
 * @param Header 串口指针
 * @param *Buffer 串口收数据数组
 * @return void
 */
void USART2_RxHandler(UART_HandleTypeDef *Header, uint8_t *Buffer){
    uint16_t crc = crc_ccitt(Buffer,RS485_recv_Data_N-2);
    if((Buffer[14] != (crc&0xFF)) || (Buffer[15] != ((crc>>8) & 0xFF))){
        return;
    }
    uint8_t User_Rx_usart_id = Buffer[2]&0xF;
    switch (User_Rx_usart_id )
    {
    case  0:
        motor_YS.UART_recv(Buffer);
	    motor_YS_Data_recv(&motor_YS,&motor_ys_data.recv);
        motor_ys_feedback_received = true;
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
// void motor_YS_Init(UART_HandleTypeDef *huart,Class_Motor_YS *__motor_YS,uint8_t id,Struct_recv_motor_YS *data){
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

bool motor_YS_Init(UART_HandleTypeDef *huart, Class_Motor_YS *__motor, uint8_t id, Struct_recv_motor_YS *data){
    __motor->Init(huart, id, Motor_YS_Pos_control);
    motor_ys_feedback_received = false;

    const uint32_t start_tick = HAL_GetTick();
    const uint32_t timeout_ms = 500U;  // 示例超时时间

    while (!motor_ys_feedback_received)
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
    if (data->MError != 0U)
    {
        __motor->lose();
        return false;
    }

    return true;
}

/**
 * @brief 宇数电机的接收数据传递
 * @param __motor_YS 电机的类指针
 * @param *data 电机接收数据指针(外部定义方便计算)
 * @return void
 */
//电机反馈的实时信息，后面可用来做闭环控制
void motor_YS_Data_recv(Class_Motor_YS *__motor_YS,Struct_recv_motor_YS *data){
    data->MError = __motor_YS->Get_MError();
    data->mode = __motor_YS->Get_mode();
    data->Now_Angle = __motor_YS->Get_Now_Angle();
    data->Now_Pos = __motor_YS->Get_Now_Pos();
    data->Now_W = __motor_YS->Get_Now_W();
    data->Now_T = __motor_YS->Get_Now_T();
    data->Now_Temperature = __motor_YS->Get_Now_Temperature();
}

/**
 * @brief 宇树电机的发送数据传递
 * @param __motor_YS 电机的类指针
 * @param data 电机发送数据(外部定义方便传递数据)
 * @return void
 */
void motor_YS_Data_send(Class_Motor_YS *__motor_YS,Struct_send_motor_YS data){
    __motor_YS->Set_Angle(data.Angle);
    __motor_YS->Set_Pos(data.Pos);
    __motor_YS->Set_T(data.T);
    __motor_YS->Set_W(data.W);
    __motor_YS->Set_Kp(data.Kp);
    __motor_YS->Set_Kd(data.Kd);
}

/**
 * @brief 电机的初始化
 * @return void
 */
// void motor_task_init(){
uint8_t motor_task_init(){

	// HAL_TIM_Base_Start_IT(&htim8);

   USART_RX485_init(&huart2,USART2_RxHandler);//串口2回调函数的初始化

 //宇树电机初始化,成功返回
	// motor_YS_Init(&huart2,&motor_YS,1,&motor_ys_data.recv);
    return motor_YS_Init(
        &huart2, &motor_YS, 0, &motor_ys_data.recv
    ) ? 1U : 0U;
}

/**
 * @brief 电机任务
 * @return void
 */
 //在这里面把数据处理好之后发给电机
void motor_task(){

	motor_YS_Data_send(&motor_YS,motor_ys_data.send);
	motor_YS.UART_send();
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

    const float test_speed = 1.0f; // 输出轴目标速度，rad/s
    const float test_kd = 0.01f;   // 速度增益，需根据实际响应调整

    // 一次性设置参数，避免定时中断读取到中间状态
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    motor_ys_data.send.Angle = 0.0f;
    motor_ys_data.send.Pos   = 0.0f;
    motor_ys_data.send.T     = 0.0f;
    motor_ys_data.send.T_ff  = 0.0f;
    motor_ys_data.send.Kp    = 0.0f;
    motor_ys_data.send.Kd    = test_kd;
    motor_ys_data.send.W     = test_speed;

    motor_YS.Set_Status(Motor_YS_Status_ENABLE);

    __set_PRIMASK(primask);

    HAL_Delay(10000U);

    motor_ys_data.send.W = 0.0f;
    HAL_Delay(3000U);

    motor_ys_data.send.W = test_speed;
    HAL_Delay(5000U);

    motor_ys_data.send.W = 0.0f;

    // 给 TIM8 留出发送零速目标的时间
    HAL_Delay(5U);
}


void motor_control_text()
{
    const float run_speed = 1.0f;  // 输出轴目标速度，rad/s
    const float speed_kd = 0.01f;

    motor_ys_data.send.Kp = 0.0f;
    motor_ys_data.send.Kd = speed_kd;
    motor_ys_data.send.T  = 0.0f;

    if (SBUS_IsOnline())
        motor_ys_data.send.W = run_speed;
    else
        motor_ys_data.send.W = 0.0f;

}



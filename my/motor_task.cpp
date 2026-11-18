/*
 * 文件：DM02_control-main/dm02_motor/User_File/task/motor_task.cpp
 * 作用：宇树电机任务，保留原有 RS485 接收、初始化及周期发送。
 * 用途：USART2 对应 ID0~3，USART3 对应 ID4~7；初始化并周期发送已接入电机。
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
#include <math.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//宇树电机类和控制所需数据
Class_Motor_YS motor_YS[8];
Motor_Control motor[MOTOR_COUNT] = {};
// motor_control_text使用的测试槽：0为髋电机，1为膝电机。
static constexpr float motor_ys_gear_ratio = 6.33f;
// 这里只列实际接入的ID；扩展电机时，初始化与轮发共用这份列表。
static constexpr uint8_t active_motor_ids[] = {0U, 1U};
static constexpr uint8_t active_motor_count = sizeof(active_motor_ids) / sizeof(active_motor_ids[0]);

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
    if (Buffer[0] != 0xFD || Buffer[1] != 0xEE)
        return;
    uint16_t crc = crc_ccitt(Buffer,RS485_recv_Data_N-2);
    if((Buffer[14] != (crc&0xFF)) || (Buffer[15] != ((crc>>8) & 0xFF))){
        return;
    }
    uint8_t User_Rx_usart_id = Buffer[2]&0xF;
    switch (User_Rx_usart_id )
    {
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
    USART_RX485_init(&huart3, USART3_RxHandler);

    for (uint8_t index = 0U; index < active_motor_count; ++index)
    {
        const uint8_t id = active_motor_ids[index];
        UART_HandleTypeDef *uart = id < 4U ? &huart2 : &huart3;
        if (!motor_YS_Init(uart, &motor_YS[id], id, &motor[id]))
            return 0U;
    }

    return 1U;
}

/**
 * @brief 电机任务，控制函数
 * @return void
 */
 //在这里面把数据处理好之后发给电机,再中断里被调用
void motor_task(){
    static uint8_t index = 0U;
    if (index == active_motor_count)
        index = 0U;

    const uint8_t id = active_motor_ids[index];
    motor_YS_Data_send(&motor_YS[id], &motor[id]);
    motor_YS[id].UART_send();
    index++;
}
//电机转动测试
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

//测试代码，可以删除
Single_Leg_Stage single_leg_stage = Single_Leg_Return;
Single_Leg_Debug single_leg_debug = {};

void single_leg_test(void)
{
    const float torque_limit[2] = {1.5f, 2.5f};//轴 N·m，沿用当前遥控测试限幅。
    const float return_kp = 20.0f; // 输出轴 N·m/rad
    const float return_kd = 0.4f;  // 输出轴 N·m/(rad/s)
    const float return_max_speed = 0.3f; // 输出轴 rad/s
    const float foot_kp = 150.0f;       // N/m
    const float foot_kd = 5.0f;         // N·s/m
    const float step_length = 0.080f;
    const float step_height = 0.020f;
    const float prepare_seconds = 2.0f;
    const float stance_seconds = 2.0f;
    const float swing_seconds = 2.0f;

    static bool initialized = false;
    static uint32_t last_update_tick = 0U;
    static uint32_t stage_start_tick = 0U;
    static uint32_t settled_ms = 0U;
    static float return_seconds;
    static Kine boot_motor;
    static foot reference_foot;
    static Point prepare_start;

    const uint32_t now = HAL_GetTick();
    const uint32_t update_ms = now - last_update_tick;
    if (update_ms < 2U)
        return;
    last_update_tick = now;

    LEG *test_leg = &leg[0];
    Param *param = &test_leg->const_param;
    if (!initialized)
    {
        // 每次上电或复位前，必须人工对齐髋+60°、相对膝-120°的机械标记。
        param->start_motor_Alpha = motor[0].measure.Pos;
        param->start_motor_Beta = motor[1].measure.Pos;
    }
    Get_Now_Pos_Spd(0);

    if (!initialized)
    {
        boot_motor = test_leg->Motor;
        single_leg_debug.boot_pos[0] = boot_motor.Pos.Alpha;
        single_leg_debug.boot_pos[1] = boot_motor.Pos.Beta;
        single_leg_debug.reference_pos[0] = param->start_motor_Alpha;
        single_leg_debug.reference_pos[1] = param->start_motor_Beta;
        Kine reference_joint = {};
        reference_joint.Pos.Alpha = param->start_Alpha;
        reference_joint.Pos.Beta = param->start_Beta;
        Kinematic_Solution(param, &reference_foot, &reference_joint);
        const float distance = fmaxf(
            fabsf(param->start_motor_Alpha - boot_motor.Pos.Alpha),
            fabsf(param->start_motor_Beta - boot_motor.Pos.Beta));
        // 三次平滑插值的峰值速度是平均速度的1.5倍。
        return_seconds = fmaxf(4.0f, 1.5f * distance / return_max_speed);
        for (uint8_t id = 0U; id < 2U; ++id)
        {
            motor[id].K_p = 0.0f;
            motor[id].K_w = 0.0f;
            motor[id].command_mode = 1U;
        }
        single_leg_stage = Single_Leg_Hold;
        stage_start_tick = now;
        initialized = true;
    }

    single_leg_debug.update_count++;
    single_leg_debug.stage_elapsed_ms = now - stage_start_tick;
    single_leg_debug.return_error_deg[0] =
        (param->start_motor_Alpha - test_leg->Motor.Pos.Alpha) / CHANGE_TO_RADIAN;
    single_leg_debug.return_error_deg[1] =
        (param->start_motor_Beta - test_leg->Motor.Pos.Beta) / CHANGE_TO_RADIAN;
    const float elapsed = single_leg_debug.stage_elapsed_ms * 0.001f;
    if (single_leg_stage == Single_Leg_Return || single_leg_stage == Single_Leg_Hold)
    {
        const float u = single_leg_stage == Single_Leg_Return ?
                        fminf(elapsed / return_seconds, 1.0f) : 1.0f;
        const float blend = u * u * (3.0f - 2.0f * u);
        const float blend_speed = 6.0f * u * (1.0f - u) / return_seconds;
        Kine *target = &test_leg->Pctrl.Motor_Target;
        const float hip_distance = param->start_motor_Alpha - boot_motor.Pos.Alpha;
        const float knee_distance = param->start_motor_Beta - boot_motor.Pos.Beta;
        target->Pos.Alpha = boot_motor.Pos.Alpha + blend * hip_distance;
        target->Pos.Beta = boot_motor.Pos.Beta + blend * knee_distance;
        target->Spd.Alpha = blend_speed * hip_distance;
        target->Spd.Beta = blend_speed * knee_distance;
        test_leg->Motor_Torque.Alpha =
            return_kp * (target->Pos.Alpha - test_leg->Motor.Pos.Alpha) +
            return_kd * (target->Spd.Alpha - test_leg->Motor.Spd.Alpha);
        test_leg->Motor_Torque.Beta =
            return_kp * (target->Pos.Beta - test_leg->Motor.Pos.Beta) +
            return_kd * (target->Spd.Beta - test_leg->Motor.Spd.Beta);
        Motor_Convert_To_Joint(param, &test_leg->Pctrl.Joint_Target, target);
        Kinematic_Solution(param, &test_leg->Foot_Target, &test_leg->Pctrl.Joint_Target);

        if (single_leg_stage == Single_Leg_Return)
        {
            const bool arrived = elapsed >= return_seconds &&
                fabsf(param->start_motor_Alpha - test_leg->Motor.Pos.Alpha) < 2.0f * CHANGE_TO_RADIAN &&
                fabsf(param->start_motor_Beta - test_leg->Motor.Pos.Beta) < 2.0f * CHANGE_TO_RADIAN &&
                fabsf(test_leg->Motor.Spd.Alpha) < 0.1f &&
                fabsf(test_leg->Motor.Spd.Beta) < 0.1f;
            settled_ms = arrived ? settled_ms + update_ms : 0U;
            single_leg_debug.settled_ms = settled_ms;
            if (settled_ms >= 500U)
            {
                single_leg_stage = Single_Leg_Hold;
                stage_start_tick = now;
            }
        }
        else if (elapsed >= 1.0f)
        {
            prepare_start = test_leg->Foot.p;
            single_leg_stage = Single_Leg_Prepare;
            stage_start_tick = now;
        }
    }
    else
    {
        const float front_x = reference_foot.p.x + step_length * 0.5f;
        const float rear_x = reference_foot.p.x - step_length * 0.5f;
        const float stance_z = reference_foot.p.z;
        const float duration = single_leg_stage == Single_Leg_Prepare ? prepare_seconds :
                               single_leg_stage == Single_Leg_Stance ? stance_seconds : swing_seconds;
        const float u = fminf(elapsed / duration, 1.0f);
        const float blend = u * u * (3.0f - 2.0f * u);
        const float blend_speed = 6.0f * u * (1.0f - u) / duration;
        foot *target = &test_leg->Foot_Target;
        target->p.y = 0.0f;
        target->v.y = 0.0f;
        if (single_leg_stage == Single_Leg_Prepare)
        {
            target->p.x = prepare_start.x + blend * (front_x - prepare_start.x);
            target->p.z = prepare_start.z + blend * (stance_z - prepare_start.z);
            target->v.x = blend_speed * (front_x - prepare_start.x);
            target->v.z = blend_speed * (stance_z - prepare_start.z);
        }
        else if (single_leg_stage == Single_Leg_Stance)
        {
            target->p.x = front_x - blend * step_length;
            target->p.z = stance_z;
            target->v.x = -blend_speed * step_length;
            target->v.z = 0.0f;
        }
        else
        {
            const float lift = sinf(PI * u);
            target->p.x = rear_x + blend * step_length;
            target->p.z = stance_z + step_height * lift * lift;
            target->v.x = blend_speed * step_length;
            target->v.z = step_height * PI / duration * sinf(2.0f * PI * u);
        }

        // 使用足端速度反馈作阻尼，单位不依赖调用周期。
        test_leg->VMC_Output_Force.x = foot_kp * (target->p.x - test_leg->Foot.p.x) +
                                       foot_kd * (target->v.x - test_leg->Foot.v.x);
        test_leg->VMC_Output_Force.y = 0.0f;
        test_leg->VMC_Output_Force.z = foot_kp * (target->p.z - test_leg->Foot.p.z) +
                                       foot_kd * (target->v.z - test_leg->Foot.v.z);
        VMC_Jacobi_Matrix(test_leg, param, &test_leg->VMC_Output_Force, &test_leg->Motor_Torque);

        if (elapsed >= duration)
        {
            single_leg_stage = single_leg_stage == Single_Leg_Stance ?
                               Single_Leg_Swing : Single_Leg_Stance;
            stage_start_tick = now;
        }
    }

    test_leg->Motor_Torque.Alpha = fmaxf(-torque_limit[0], fminf(torque_limit[0], test_leg->Motor_Torque.Alpha));
    test_leg->Motor_Torque.Beta = fmaxf(-torque_limit[1], fminf(torque_limit[1], test_leg->Motor_Torque.Beta));
    Set_Motor_Target_Torque(test_leg);
}
//遥控的测试
void motor_control_text()
{
    static uint32_t last_update_tick = 0U;
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_update_tick) < 2U)
        return;
    last_update_tick = now;

    const float max_speed = 0.5f; // 输出轴 rad/s
    const float speed_gain = 1.6f; // 软件速度误差增益，输出轴 N·m/(rad/s)
    const float torque_limit[2] = {0.9f, 0.8f}; // ID0、ID1输出轴 N·m
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



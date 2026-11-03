/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：DM02_control-main/dm02_motor/User_File/BSP/UART/UART.h
 * 作用：UART/RS485 业务适配层。
 * 用途：注册回调、启动 DMA 接收与发送并分发接收完成事件；依赖目标串口和内存配置。
 * 主要符号（节选）：USART_RX485_init、USART_init、UART_Transmit_Data。
 * 依赖：usart.h、stm32h7xx_hal.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
//
// Created by 20159 on 2026/4/1.
//

#ifndef UART_H
#define UART_H

#include "usart.h"
#include "stm32h7xx_hal.h"

#define USART1_recv_Data_N  1
#define SBUS_RX_BUF_NUM		25u
// #define SBUS_RX_BUF_NUM		18u
#define USART7_recv_Data_N  1
#define USART10_recv_Data_N 1
//宇树485发送数组大小
#define RS485_Send_Data_N   17
//宇树485接收数组大小
#define RS485_recv_Data_N   16


//定义串口回调函数
typedef void (*USART_Callback)(UART_HandleTypeDef *Header, uint8_t *Buffer);


void USART_RX485_init(UART_HandleTypeDef *huart,USART_Callback back);

void USART_init(UART_HandleTypeDef *huart,USART_Callback back);

uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data,uint8_t len);



#endif //DM02_UART_H
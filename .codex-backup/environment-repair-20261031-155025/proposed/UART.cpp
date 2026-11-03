/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：DM02_control-main/dm02_motor/User_File/BSP/UART/UART.cpp
 * 作用：UART/RS485 业务适配层。
 * 用途：注册回调、启动 DMA 接收与发送并分发接收完成事件；依赖目标串口和内存配置。
 * 主要符号（节选）：USART_RX485_init、USART_init、UART_Transmit_Data、HAL_UART_RxCpltCallback。
 * 依赖：UART.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 */
//
// Created by 20159 on 2026/4/1.
//

#include "UART.h"
#include "bsp_sbus.h"
#include "bsp_uart.h"
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

USART_Callback Usart1_Callback_Function;
USART_Callback Usart2_Callback_Function;
USART_Callback Usart3_Callback_Function;
USART_Callback Usart5_Callback_Function;
USART_Callback Usart7_Callback_Function;
USART_Callback Usart10_Callback_Function;

 bool SBUS_RX_init_bl;

//把数据放在0x24000000之后因为DMA原因
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart1_recv_Data[USART1_recv_Data_N];
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart2_recv_Data[RS485_recv_Data_N];
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart3_recv_Data[RS485_recv_Data_N];
__attribute__((section (".AXI_SRAM")))  uint8_t Usart5_recv_Data[SBUS_RX_BUF_NUM];
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart7_recv_Data[USART7_recv_Data_N];
__attribute__((section (".AXI_SRAM"))) static uint8_t Usart10_recv_Data[USART10_recv_Data_N];

__attribute__((section (".AXI_SRAM"))) static uint8_t SBUS_RX_Data[SBUS_RX_BUF_NUM]={
	0x00,0x04,0x20,0x00,0x01,0x58,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04
};
/**
 * @brief 串口的初始化
 * @param huart  串口编号
 * @param back 回调函数传参
 * @return void
 */
void USART_RX485_init(UART_HandleTypeDef *huart,USART_Callback back){
	HAL_RS485Ex_Init(huart,UART_DE_POLARITY_HIGH,2,2);
    if(huart==&huart2){
        Usart2_Callback_Function = back;
    }else if(huart==&huart3){
        Usart3_Callback_Function = back;
    }
    if(huart==&huart2){
        HAL_UART_Receive_DMA(huart,Usart2_recv_Data, RS485_recv_Data_N);
    }else if(huart==&huart3){
        HAL_UART_Receive_DMA(huart,Usart3_recv_Data, RS485_recv_Data_N);
    }

}

/**
 * @brief 串口的初始化
 * @param huart  串口编号
 * @param back 回调函数传参
 * @return void
 */
void USART_init(UART_HandleTypeDef *huart,USART_Callback back){
    if(huart==&huart2){
        Usart2_Callback_Function = back;
    }else if(huart==&huart3){
        Usart3_Callback_Function = back;
    }else if(huart==&huart5){
        Usart5_Callback_Function = back;
    }
    if(huart==&huart2){
        HAL_UART_Receive_DMA(huart,Usart2_recv_Data, RS485_recv_Data_N);
    }else if(huart==&huart3){
        HAL_UART_Receive_DMA(huart,Usart3_recv_Data, RS485_recv_Data_N);
    }else if(huart==&huart5){
		SBUS_RX_init_bl = 1;
        HAL_UART_Receive_IT(huart,Usart5_recv_Data, SBUS_RX_BUF_NUM);
    }
}



/**
 * @brief 串口的发送
 * @param huart  串口编号
 * @param *Data 发送数据数组指针
 * @return void
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data,uint8_t len)
{
    __attribute__((section (".AXI_SRAM")))  static uint8_t Usart_send_Data[RS485_Send_Data_N];
    for(uint8_t i=0;i<len;++i)Usart_send_Data[i] = Data[i];

    return (HAL_UART_Transmit_DMA(huart, Usart_send_Data, len));
}

/**
 * @brief 串口接收回调函数
 * @param huart  串口编号
 * @return void
 */
/*暂时只用了uart2，3，5
2 s475  3 s485的另一个接口 5 遥控器的接收
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart == &huart2)
    {

        if (Usart2_Callback_Function != nullptr)
        {
            Usart2_Callback_Function(huart, Usart2_recv_Data);
		
        }
        
    }
    else if (huart == &huart3)
    {
        if (Usart3_Callback_Function != nullptr)
        {
            Usart3_Callback_Function(huart, Usart3_recv_Data);
        }
    }
    else if (huart == &huart5)
    {
        // if (Usart5_Callback_Function != nullptr)
        // {
		// 	if(SBUS_RX_init_bl){
		// 		for(uint8_t i=0;i<SBUS_RX_BUF_NUM;i++){
		// 			if(Usart5_recv_Data[i]!=SBUS_RX_Data[i]&&i!=5){
		// 				Usart5_recv_Data[i]=0;
		// 				return;
		// 			}
		// 		}
		// 		SBUS_RX_init_bl=0;
		// 	}
        //     Usart5_Callback_Function(huart, Usart5_recv_Data);
        // }
        //遥控器的接收完成中断回调
        {
            SBUS_Reveive(RxTemp_2);
            // printf("a:%d\n", RxTemp_2);
            HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxTemp_2, 1);
        }
    }
}


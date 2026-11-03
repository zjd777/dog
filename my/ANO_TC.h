/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/ANO_TC.h
 * 作用：匿名上位机波形输出。
 * 用途：封装浮点波形并经串口发送；原通道数必须与实际数组一致，本次不改算法。
 * 主要符号（节选）：ANO_TC_To_Send、Usart_Send_To_Show32。
 * 依赖：stdint.h、usart.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：ANO_TC.h
 * 用途：匿名上位机波形通信。
 * 说明：定义发送通道数、浮点数字节提取宏及上位机发送接口。
 */
#ifndef ANO_TC_H
#define ANO_TC_H

#include "stdint.h"
#include "usart.h"
#define BIT16		short
#define  LAN16            10	
#define BIT32		float
#define  LAN32           11	

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void ANO_TC_To_Send(void);
void Usart_Send_To_Show32(UART_HandleTypeDef* huart, float *data_ts);

#endif

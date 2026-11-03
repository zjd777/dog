/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：Corgi_for_ROBOCON/corgi/My/ANO_TC.c
 * 作用：匿名上位机波形输出。
 * 用途：封装浮点波形并经串口发送；原通道数必须与实际数组一致，本次不改算法。
 * 主要符号（节选）：Usart_Send_To_Show32。
 * 依赖：ANO_TC.h、usart.h、string.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
/*
 * 文件：ANO_TC.c
 * 用途：匿名上位机波形通信。
 * 说明：将浮点数组封装成匿名上位机数据帧，通过串口 DMA 发送，用于曲线观察与调试。
 */
#include "ANO_TC.h"
#include "usart.h"
#include "string.h"

__attribute__((section(".AXI_SRAM")))
static uint8_t data[4 + LAN32 * 4 + 1];
void Usart_Send_To_Show32(UART_HandleTypeDef *huart, float *data_ts)
{
    uint8_t i, sum = 0;
    uint8_t data_head[4] = {0xAA, 0xAA, 0xF1, LAN32 * 4};
    memcpy(data, data_head, sizeof(uint8_t) * 4);
    for (i = 0; i < LAN32; i++)
    {
        uint8_t data_head[4] = {BYTE3(data_ts[i]), BYTE2(data_ts[i]), BYTE1(data_ts[i]), BYTE0(data_ts[i])};
        memcpy(data + 4 * (i + 1), data_head, sizeof(char) * 4);
        sum += BYTE3(data_ts[i]) + BYTE2(data_ts[i]) + BYTE1(data_ts[i]) + BYTE0(data_ts[i]);
    }
    sum += 0xAA + 0xAA + 0XF1 + LAN32 * 4;

    memcpy(data + 4 + LAN32 * 4, &sum, sizeof(char) * 1);
    HAL_UART_Transmit_DMA(huart, data, 4 + LAN32 * 4 + 1);
//		HAL_UART_Transmit(huart, data, 4 + LAN32 * 4 + 1,0xfff);
}

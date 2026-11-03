/*
 * bsp_uart.h
 *
 *  Created on: Mar 4, 2022
 *      Author: Administrator
 */

#ifndef BSP_UART_H_
#define BSP_UART_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t RxTemp_2;

#ifdef __cplusplus
}
#endif

void USART_Init(void);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);


#endif /* BSP_UART_H_ */

/*
 * bsp_sbus.h
 *
 *  Created on: Mar 8, 2022
 *      Author: Administrator
 */

#ifndef BSP_SBUS_H_
#define BSP_SBUS_H_

#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS       0x00

#ifdef __cplusplus
extern "C" {
#endif
// CH1～CH10为换算后的控制值，中值1500。
extern int16_t g_sbus_channels[18];
extern volatile uint32_t sbus_uart_error_count;
extern volatile uint32_t sbus_uart_last_error;
void SBUS_Reveive(uint8_t data);
void SBUS_Handle(void);
uint8_t SBUS_IsOnline(void);

#ifdef __cplusplus
}

#endif

#endif /* BSP_SBUS_H_ */

/*
 * 工程文件说明（注释整理，不修改程序逻辑）
 * 文件：DM02_control-main/dm02_motor/User_File/Driver/remote_control/remote_control.h
 * 作用：大疆 DBUS 遥控解码。
 * 用途：接收并解码 18 字节通道/开关/鼠标/键盘数据；原函数名带 SBUS，但协议不是标准 25 字节 SBUS。
 * 主要符号（节选）：remote_control_init。
 * 依赖：struct_typedef.h、UART.h。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"

#include "UART.h"

#define RC_CH_VALUE_OFFSET		1024U

struct Remote_Info_Typedef
{
    struct
	{
		int16_t ch[5];
		uint8_t s[2];
    } rc;
	struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    struct
    {
        uint16_t v;
    } key;

	bool rc_lost;   
	uint8_t online_cnt;   
} ;

extern Remote_Info_Typedef remote_ctrl;

void remote_control_init();


#endif


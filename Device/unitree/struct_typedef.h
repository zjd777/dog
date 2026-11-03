/*
 * 文件：DM02_control-main/dm02_motor/User_File/algorithm/struct_typedef.h
 * 作用：工程基础类型。
 * 用途：定义整数和浮点别名，供算法与驱动共享；移植时注意与 stdint.h 的重复定义。
 * 主要符号（节选）：类型、宏或类声明，详见本文件正文。
 * 依赖：无直接 include；由使用方按声明/宏配置引入。
 * 注意：仅统一编码并补注释，不修复原算法、通信、超时或安全联锁问题；移植前按当前硬件检查依赖。
 * 编码：UTF-8；原作者、版权声明和自动生成标记保留。
 */
#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include <stdint.h>


// typedef signed char int8_t; // Use the standard <stdint.h> type.
// typedef signed short int int16_t; // Use the standard <stdint.h> type.
// typedef signed int int32_t; // Use the standard <stdint.h> type.
// typedef signed long long int64_t; // Use the standard <stdint.h> type.

/* exact-width unsigned integer types */
// typedef unsigned char uint8_t; // Use the standard <stdint.h> type.
// typedef unsigned short int uint16_t; // Use the standard <stdint.h> type.
// typedef unsigned int uint32_t; // Use the standard <stdint.h> type.
// typedef unsigned long long uint64_t; // Use the standard <stdint.h> type.
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif




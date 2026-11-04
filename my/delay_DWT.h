#ifndef __DELAY_DWT_H_
#define __DELAY_DWT_H_

#include <stdint.h>
#include "stm32h7xx.h"

/* 系统时钟配置完成后使用；计时期间保持内核频率不变。 */
#define GET_CPU_ClkFreq() (SystemCoreClock)
#define Delay_ms(ms) Delay_us((uint32_t)((ms) * 1000U))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Time
{
    char sign;
    uint32_t Cycle_Times;        /* CYCCNT 自然回绕的累计次数。 */
    float Individual_Cycle_time; /* 当前回绕周期内的秒数。 */
    float Now_Tick;              /* 自首次初始化起的秒数。 */
    uint32_t DWT_tick;
} Time_Param;

extern Time_Param Time_Points_Param;

/* 首次调用启动计数器，后续调用保留计时状态。 */
void DWT_Init(void);
uint32_t CPU_TS_TmrRd(void);
void Delay_us(uint32_t us);

/* 两次更新时间须小于 2^32 / 内核频率秒，480 MHz 下约 8.95 秒。
 * float 秒值的分辨率会随累计运行时间增长而降低。
 */
float Generate_Time_Points(void);
float GetTick(void);

#ifdef __cplusplus
}
#endif

#endif

#include "delay_DWT.h"

Time_Param Time_Points_Param;

/**
 * @brief 启动 DWT 内核周期计数器。
 * @param 无。
 * @retval 无。
 * @note 在系统时钟配置完成后使用；仅首次调用清零，重复调用保留计时状态。
 */
void DWT_Init(void)
{
    /* 延时和动作计时共用 CYCCNT，重复初始化不能清零。 */
    if (Time_Points_Param.sign)
        return;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    Time_Points_Param.sign = 1;
}

/**
 * @brief 读取 DWT 原始周期计数。
 * @param 无。
 * @return 当前 32 位计数值，单位为内核时钟周期。
 * @note 调用前须启动 DWT；此函数不初始化、不换算时间，也不累计回绕次数。
 */
uint32_t CPU_TS_TmrRd(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief 使用 DWT 忙等待指定时间。
 * @param us 等待时间，单位微秒。
 * @retval 无。
 * @note 自动初始化 DWT，不清零已有计数；中断和函数开销可能延长实际等待时间。
 *       本函数不更新动作时间，等待期间仍需满足累计计时的更新间隔约束。
 */
void Delay_us(uint32_t us)
{
    DWT_Init();

    uint64_t ticks = (uint64_t)us * GET_CPU_ClkFreq() / 1000000U;
    uint64_t elapsed = 0U;
    uint32_t last = CPU_TS_TmrRd();

    while (elapsed < ticks)
    {
        uint32_t now = CPU_TS_TmrRd();
        elapsed += (uint32_t)(now - last);
        last = now;
    }
}

/**
 * @brief 统计 DWT 自然回绕并更新累计时间。
 * @param 无。
 * @return 自首次初始化 DWT 起经过的秒数，同时写入 Time_Points_Param.Now_Tick。
 * @note 自动初始化 DWT；两次更新时间须小于 2^32 / SystemCoreClock 秒，
 *       480 MHz 下约为 8.95 秒。计时期间内核频率须保持不变；返回的 float
 *       秒值分辨率会随累计运行时间增长而降低。
 */
float Generate_Time_Points(void)
{
    DWT_Init();

    uint32_t now = CPU_TS_TmrRd();
    if (now < Time_Points_Param.DWT_tick)
        Time_Points_Param.Cycle_Times++;

    Time_Points_Param.DWT_tick = now;
    Time_Points_Param.Individual_Cycle_time = (float)now / GET_CPU_ClkFreq();
    /* 先用 double 换算，避免累计周期数提前转为 float 丢失精度。 */
    Time_Points_Param.Now_Tick = (float)(
        ((double)Time_Points_Param.Cycle_Times * 4294967296.0 + now) /
        GET_CPU_ClkFreq());
    return Time_Points_Param.Now_Tick;
}

/**
 * @brief 为动作和步态提供累计秒数接口。
 * @param 无。
 * @return Generate_Time_Points() 更新后的累计秒数。
 * @note 此函数会更新计时状态，沿用 Generate_Time_Points() 的更新间隔约束；
 *       与返回毫秒数的 HAL_GetTick() 单位不同。
 */
float GetTick(void)
{
    return Generate_Time_Points();
}

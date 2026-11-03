/*
 * 文件：Device/BMI088/BMI088Middleware.c
 * 作用：把 BMI088 驱动的片选、SPI 和延时操作接到 MC02 的 HAL。
 * 来源：达妙 MC02 的 CtrBoard-H7_IMU 示例中间层；保留原有实现。
 * 用法：SPI2 使用 hspi2；加速度计片选 PC0，陀螺仪片选 PC3_C，均低电平有效。
 * 注意：GPIO_init/com_init 当前为空，硬件初始化依靠 main 中的 MX_*；SPI 使用阻塞调用而非 DMA，微秒延时依赖 480 MHz 的 SysTick 计数。
 * 整理：统一 UTF-8，补充中文说明；本次不改变可执行代码。
 */
#include "BMI088Middleware.h"
#include "main.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

/**
************************************************************************
* @brief:      	BMI088_GPIO_init(void)
* @param:       void
* @retval:     	void
* @details:    	GPIO 初始化占位接口；当前函数为空，实际配置由 MX_GPIO_Init 完成
************************************************************************
**/
void BMI088_GPIO_init(void)
{

}
/**
************************************************************************
* @brief:      	BMI088_com_init(void)
* @param:       void
* @retval:     	void
* @details:    	通信初始化占位接口；当前函数为空，SPI2 由 MX_SPI2_Init 完成
************************************************************************
**/
void BMI088_com_init(void)
{


}
/**
************************************************************************
* @brief:      	BMI088_delay_ms(uint16_t ms)
* @param:       ms - 要延迟的毫秒数
* @retval:     	void
* @details:    	延迟指定毫秒数的函数，基于微秒延迟实现
************************************************************************
**/
void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}
/**
************************************************************************
* @brief:      	BMI088_delay_us(uint16_t us)
* @param:       us - 要延迟的微秒数
* @retval:     	void
* @details:    	使用 SysTick 忙等待；us * 480 依赖计数频率为 480 MHz，修改时钟后需要同步调整
************************************************************************
**/
void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 480;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088加速度计片选信号置低，使其处于选中状态
************************************************************************
**/
void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_ACCEL_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088加速度计片选信号置高，使其处于非选中状态
************************************************************************
**/
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_L(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088陀螺仪片选信号置低，使其处于选中状态
************************************************************************
**/
void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}
/**
************************************************************************
* @brief:      	BMI088_GYRO_NS_H(void)
* @param:       void
* @retval:     	void
* @details:    	将BMI088陀螺仪片选信号置高，使其处于非选中状态
************************************************************************
**/
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}
/**
************************************************************************
* @brief:      	BMI088_read_write_byte(uint8_t txdata)
* @param:       txdata - 要发送的数据
* @retval:     	uint8_t - 接收到的数据
* @details:    	通过 SPI2 阻塞交换一个字节，HAL 超时参数为 1000 ms；原实现没有检查 HAL 返回状态
************************************************************************
**/
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}



#include "bsp_sbus.h"
#include "bsp.h"
#include "string.h"


#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00

// Parameters related to receiving data  接收数据相关参数
uint8_t sbus_start = 0;
uint8_t sbus_buf_index = 0;
uint8_t sbus_new_cmd = 0;

// data-caching mechanism  数据缓存
uint8_t inBuffer[SBUS_RECV_MAX] = {0};
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t sbus_data[SBUS_RECV_MAX] = {0};
int16_t g_sbus_channels[18] = {0};
//最近完整帧的接收时间
static volatile uint32_t sbus_last_rx_tick = 0;
static volatile uint8_t sbus_frame_ok = 0;   //帧是否报告正常
volatile uint32_t sbus_uart_error_count = 0U;
volatile uint32_t sbus_uart_last_error = 0U;

// Parses SBUS data into channel values  解析SBUS的数据，转化成通道数值。
static int SBUS_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    g_sbus_channels[5]  = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
    g_sbus_channels[6]  = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif

    // SBUS原始值192/992/1792统一换算为控制端使用的1000/1500/2000。
    //原来的没有进行值的换算
    for (uint8_t i = 0U; i < 10U; ++i)
        g_sbus_channels[i] = g_sbus_channels[i] * 5 / 8 + 880;

    // 安全检测，检测是否失联或者数据错误
    // Security detection to check for lost connections or data errors
    failsafe_status = SBUS_SIGNAL_OK;
    if (sbus_data[23] & (1 << 2))
    {
        failsafe_status = SBUS_SIGNAL_LOST;
        printf("SBUS_SIGNAL_LOST\n");
        // lost contact errors  遥控器失联错误
    }
    else if (sbus_data[23] & (1 << 3))
    {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
        printf("SBUS_SIGNAL_FAILSAFE\n");
        // data loss error  数据丢失错误
    }
    return failsafe_status;
}

// Receives SBUS cache data  接收SBUS的缓存数据
//每次接受一次字节直到接受完完整一帧，复制完整帧到sbus_data[]，复制完整帧到sbus_data[]
//在中断里被调用，就先使用IT把
void SBUS_Reveive(uint8_t data)
{
    // If the protocol start flag is met, data is received  如果符合协议开始标志，则开始接收数据
    if (sbus_start == 0 && data == SBUS_START)
    {
        sbus_start = 1;
        sbus_new_cmd = 0;
        sbus_buf_index = 0;
        inBuffer[sbus_buf_index] = data;
        inBuffer[SBUS_RECV_MAX - 1] = 0xff;
    }
    else if (sbus_start)
    {
        sbus_buf_index++;
        inBuffer[sbus_buf_index] = data;
    }

    // Finish receiving a frame of data  完成接收一帧数据
    if (sbus_start & (sbus_buf_index >= (SBUS_RECV_MAX - 1)))
    {
        sbus_start = 0;
        if (inBuffer[SBUS_RECV_MAX - 1] == SBUS_END)
        {
            memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
            sbus_new_cmd = 1;
            sbus_last_rx_tick = HAL_GetTick();
            sbus_frame_ok =((inBuffer[23] & ((1U << 2) | (1U << 3))) == 0U);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart5)
        return;

    sbus_uart_error_count++;
    sbus_uart_last_error = huart->ErrorCode;

    sbus_start = 0U;
    sbus_buf_index = 0U;
    sbus_new_cmd = 0U;
    sbus_frame_ok = 0U;
    failsafe_status = SBUS_SIGNAL_LOST;

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                               UART_CLEAR_NEF | UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    // ORE时HAL已结束接收；非阻塞错误仍在接收时沿用原接收任务。
    if (huart->RxState == HAL_UART_STATE_READY)
        (void)HAL_UART_Receive_IT(huart, &RxTemp_2, 1U);
}

// SBUS receives and processes data handle  SBUS接收处理数据句柄
//放到单独的一个线程里去处理接受一个帧的数据
void SBUS_Handle(void)
{
    if (sbus_new_cmd)
    {
        //解析通道数据
        int res = SBUS_Parse_Data();
        sbus_new_cmd = 0;
        if (res) return;
        #if SBUS_ALL_CHANNELS
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
			   g_sbus_channels[3], g_sbus_channels[4], g_sbus_channels[5],
			   g_sbus_channels[6], g_sbus_channels[7], g_sbus_channels[8],
			   g_sbus_channels[9], g_sbus_channels[10], g_sbus_channels[11],
			   g_sbus_channels[12], g_sbus_channels[13], g_sbus_channels[14],
			   g_sbus_channels[15]);
        #else
        //这个是后面的函数，把他传给控制电机的函数，控制电机的函数里也有imu的
        // printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        //        g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
		// 	   g_sbus_channels[3], g_sbus_channels[4],g_sbus_channels[5],
		// 	   g_sbus_channels[6], g_sbus_channels[7],g_sbus_channels[8],
		// 	   g_sbus_channels[9]);
        
        #endif
    }
}

uint8_t SBUS_IsOnline(void)
{
    if (!sbus_frame_ok)
        return 0U;

    // 100ms 为示例超时，后续根据接收周期确定。
    if ((uint32_t)(HAL_GetTick() - sbus_last_rx_tick) >= 100U)
        return 0U;

    return 1U;
}

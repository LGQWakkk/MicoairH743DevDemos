// 20250919 Wakkk
#include "system.h"
#include "spl06.h"

#define DEBUG_UART_HANDLE huart1
uint8_t log_buffer[128];
void slog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    debugUartTx(log_buffer, strlen((char*)log_buffer));
}
void slogDma(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char*)log_buffer, sizeof(log_buffer), fmt, args);
    va_end(args);
    debugUartTxDma(log_buffer, strlen((char*)log_buffer));
}
void debugUartTx(uint8_t *buffer, uint32_t len)
{
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, buffer, len, 1000);
}
void debugUartTxDma(uint8_t *buffer, uint32_t len)
{
    HAL_UART_Transmit_DMA(&DEBUG_UART_HANDLE, buffer, len);
}

float pressure, temperature;

void systemInit(void)
{
    HAL_Delay(800);
    slog("System Init\r\n");
    slog("SPL06 Init\r\n");
    spl06Init();

    // spl06Update(&pressure, &temperature);
    // slogDma("pressure: %f, temperature: %f\r\n", pressure, temperature);
    HAL_Delay(10);
}

uint32_t lastMs = 0;
uint32_t currentMs = 0;
uint8_t intStatus = 0;

bool spl06DrFlag = false; // 下降沿中断标志位

void systemLoop(void)
{
    // 使用轮询DR引脚进行读取
    // if(spl06ExtiLow()){ // 中断引脚低电平
    //     LEDR_ON();
    //     currentMs = HAL_GetTick();
    //     uint32_t dtMs = currentMs - lastMs;
    //     lastMs = currentMs;
    //     slogDma("dtMs: %d\r\n", dtMs);
    //     intStatus = spl06GetIntStatus();
    //     if(intStatus){
    //         // 读取数据
    //         spl06Update(&pressure, &temperature);
    //     }
    // }else{
    //     LEDR_OFF();
    // }

    // 使用EXTI中断方式进行读取
    if(spl06DrFlag){ // 触发中断
        spl06DrFlag = false;
        currentMs = HAL_GetTick();
        uint32_t dtMs = currentMs - lastMs;
        lastMs = currentMs;
        intStatus = spl06GetIntStatus();
        if(intStatus){ // 读取数据
            spl06Update(&pressure, &temperature);
            slogDma("dt:%d pressure:%f temperature:%f\r\n", dtMs, pressure, temperature);
        }
    }
}

// 处理SPL06 DR中断
// EXTI 上拉输入 下降沿触发中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SPL06_DR_Pin){
        spl06DrFlag = true;
    }
}

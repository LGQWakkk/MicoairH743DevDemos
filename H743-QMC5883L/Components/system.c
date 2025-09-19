// 20250919 Wakkk
#include "system.h"
#include "qmc5883l.h"

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

void systemInit(void)
{
    HAL_Delay(800);
    slog("System Init\r\n");
    slog("QMC5883L Init\r\n");
    qmc5883lInit();
    HAL_Delay(10);
}

uint32_t lastMs = 0;
uint32_t currentMs = 0;

qmc5883l_data_t magData;

void systemLoop(void)
{
    if(qmc5883lGetState() & QMC5883L_STATUS_DRDY){ // 数据有效标志位
        currentMs = HAL_GetTick();
        uint32_t dtMs = currentMs - lastMs;
        lastMs = currentMs;
        qmc5883lUpdateData(&magData);
        slogDma("mx: %f my: %f mz: %f temp: %f\r\n", magData.mx, magData.my, magData.mz, magData.temp);
        // qmc5883lDebugData(); // 直接输出相关寄存器内容
        HAL_Delay(1); // 确保中断标志位复位 实际可能不需要
    }
}

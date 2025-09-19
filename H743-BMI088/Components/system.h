#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "stdint.h"
#include "string.h"
#include "stddef.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include "main.h"
#include "usart.h"
#include "gpio.h"

#include "bmi08x.h"
#include "spi.h"

#define LEDR_ON()  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET)
#define LEDR_OFF() HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET)
#define LEDR_TOGGLE() HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin)
#define LEDG_ON()  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET)
#define LEDG_OFF() HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET)
#define LEDG_TOGGLE() HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin)
#define LEDB_ON()  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET)
#define LEDB_OFF() HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET)
#define LEDB_TOGGLE() HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin)

void slog(const char *fmt, ...);
void slogDma(const char *fmt, ...);
void debugUartTx(uint8_t *buffer, uint32_t len);
void debugUartTxDma(uint8_t *buffer, uint32_t len);
void systemInit(void);
void systemLoop(void);

#define BMI088_SPI_HANDLE hspi2
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmi08_delay_us(uint32_t period, void *intf_ptr);

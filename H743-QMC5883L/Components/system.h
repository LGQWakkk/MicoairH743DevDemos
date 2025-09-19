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

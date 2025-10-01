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

#include "bmi270.h"
#include "spi.h"

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)
/*! Macros to select the sensors */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

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

#define BMI270_SPI_HANDLE hspi3
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

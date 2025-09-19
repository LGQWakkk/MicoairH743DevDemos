// 20250918 STM32H743
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "system.h"
#include "i2c.h"
#include "gpio.h"

// I2C地址: 0X76
// 中断引脚连接到PD0
#define SPL06_I2C_HANDLE hi2c2 // Micoair H743
#define DETECTION_MAX_RETRY_COUNT 5
#define SEA_LEVEL_PRESSURE 1013.25f // 海平面气压

typedef struct{
	float temperature;	// deg
	float pressure;		// Pa
}spl06_data_t;

typedef struct {
	int16_t c0;
	int16_t c1;
	int32_t c00;
	int32_t c10;
	int16_t c01;
	int16_t c11;
	int16_t c20;
	int16_t c21;
	int16_t c30;
}spl06_coeffs_t;

// 20250919 经过实际测试 SPL06的原始I2C地址是0x77
#define SPL06_I2C_ADDR                         (0x77<<1)
#define SPL06_DEFAULT_CHIP_ID                  0x10

#define SPL06_PRESSURE_START_REG               0x00
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_PRESSURE_B2_REG                  0x00    // Pressure MSB Register
#define SPL06_PRESSURE_B1_REG                  0x01    // Pressure middle byte Register
#define SPL06_PRESSURE_B0_REG                  0x02    // Pressure LSB Register
#define SPL06_TEMPERATURE_START_REG            0x03
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_TEMPERATURE_B2_REG               0x03    // Temperature MSB Register
#define SPL06_TEMPERATURE_B1_REG               0x04    // Temperature middle byte Register
#define SPL06_TEMPERATURE_B0_REG               0x05    // Temperature LSB Register
#define SPL06_PRESSURE_CFG_REG                 0x06    // Pressure config
#define SPL06_TEMPERATURE_CFG_REG              0x07    // Temperature config
#define SPL06_MODE_AND_STATUS_REG              0x08    // Mode and status
#define SPL06_INT_AND_FIFO_CFG_REG             0x09    // Interrupt and FIFO config
#define SPL06_INT_STATUS_REG                   0x0A    // Interrupt and FIFO config
#define SPL06_FIFO_STATUS_REG                  0x0B    // Interrupt and FIFO config
#define SPL06_RST_REG                          0x0C    // Softreset Register
#define SPL06_CHIP_ID_REG                      0x0D    // Chip ID Register
#define SPL06_CALIB_COEFFS_START               0x10
#define SPL06_CALIB_COEFFS_END                 0x21

#define SPL06_CALIB_COEFFS_LEN                 (SPL06_CALIB_COEFFS_END - SPL06_CALIB_COEFFS_START + 1)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  	// measure pressure 单独压力测量
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  	// measure temperature 单独温度测量
#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2) 	// 持续测量标志位
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4) 	// 压力数据有效
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5) 	// 温度数据有效
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6) 	// 传感器初始化完毕
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7) 	// 校准系数有效标志位

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8
#define SPL06_PRESSURE_OVERSAMPLING            8
#define SPL06_TEMPERATURE_OVERSAMPLING         8
#define SPL06_INT_ACTIVE_HIGH (0x80)
#define SPL06_INT_ACTIVE_LOW  (0x00)
#define SPL06_INT_PRS_ENABLE  (0x20) // 压力数据中断使能
#define SPL06_INT_TMP_ENABLE  (0x10) // 温度数据中断使能

// 中断标志位
#define SPL06_INT_FIFO_FULL (1<<2)
#define SPL06_INT_TMP 		(1<<1)
#define SPL06_INT_PRS 		(1<<0)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 压力传感器采样频率(measurements per second)
// PM_RATE[6:4]
#define PM_RATE_1MPS   (0<<4)
#define PM_RATE_2MPS   (1<<4)
#define PM_RATE_4MPS   (2<<4)
#define PM_RATE_8MPS   (3<<4)
#define PM_RATE_16MPS  (4<<4)
#define PM_RATE_32MPS  (5<<4)
#define PM_RATE_64MPS  (6<<4)
#define PM_RATE_128MPS (7<<4)

// 压力传感器过采样次数
// PM_PRC[3:0]
#define PM_PRC_1X      (0<<0) // 3.6ms 5PaRMS
#define PM_PRC_2X      (1<<0) // 5.2ms
#define PM_PRC_4X      (2<<0) // 8.4ms 2.5PaRMS
#define PM_PRC_8X      (3<<0) // 14.8ms
#define PM_PRC_16X     (4<<0) // 27.6ms 1.2PaRMS
#define PM_PRC_32X     (5<<0) // 53.2ms 0.9PaRMS
#define PM_PRC_64X     (6<<0) // 104.4ms 0.5PaRMS
#define PM_PRC_128X    (7<<0) // 206.8ms

// 温度传感器采样频率(measurements per second)
// TMP_RATE[6:4]
#define TMP_RATE_1MPS   (0<<4)
#define TMP_RATE_2MPS   (1<<4)
#define TMP_RATE_4MPS   (2<<4)
#define TMP_RATE_8MPS   (3<<4)
#define TMP_RATE_16MPS  (4<<4)
#define TMP_RATE_32MPS  (5<<4)
#define TMP_RATE_64MPS  (6<<4)
#define TMP_RATE_128MPS (7<<4)

// 温度传感器过采样次数
// TMP_PRC[2:0]
#define TMP_PRC_1X      (0<<0) // 3.6ms
#define TMP_PRC_2X      (1<<0)
#define TMP_PRC_4X      (2<<0)
#define TMP_PRC_8X      (3<<0)
#define TMP_PRC_16X     (4<<0)
#define TMP_PRC_32X     (5<<0)
#define TMP_PRC_64X     (6<<0)
#define TMP_PRC_128X    (7<<0)

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void spl06DmaUpdateStart(void);
void spl06DmaUpdateCallbackISR(void);
bool spl06Probe(void);
bool spl06Init(void);
float spl06GetAltitude(float pressure_Pa);
bool spl06ExtiLow(void);
void spl06Update(float* pressure, float* temperature);
uint8_t spl06GetIntStatus(void);

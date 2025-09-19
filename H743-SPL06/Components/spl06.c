// 20250918 STM32H743
// 裸机SPL06-001测试
#include "spl06.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// I2C DMA 接收缓冲区 6字节数据
// 从0x00开始的6个字节: PSR_B2 PSR_B1 PSR_B0 TMP_B2 TMP_B1 TMP_B0
#define SPL06_DATA_BUFFER_SIZE 6
static uint8_t spl06_buf[SPL06_DATA_BUFFER_SIZE]; // DMA 接收缓存
static spl06_coeffs_t spl06_cal; // 校准系数
static int32_t  spl06_pressure_raw;      // 原始压力数据
static int32_t  spl06_temperature_raw;   // 原始温度数据
static float    spl06_pressure = 0;        // 校准后压力数据
static float    spl06_temperature = 0;     // 校准后温度数据
//////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t spl06ReadByte(uint8_t reg);
static void spl06WriteByte(uint8_t reg, uint8_t data);
static void spl06ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len);
static int8_t spl06Samples2ConfigRegValue(uint8_t sample_rate);
static int32_t spl06RawValueScaleFactor(uint8_t oversampling_rate);
static void spl06StartMeasurement(void);
static void spl06StartTemperatureMeasurement(void);
static void spl06StartPressureMeasurement(void);
static void spl06ReadTemperatureRaw(void);
static void spl06ReadPressureRaw(void);
static float spl06CompensateTemperature(int32_t temperature_raw);
static float spl06CompensatePressure(int32_t pressure_raw, int32_t temperature_raw);
static void spl06Calculate(float* pressure, float* temperature);
static bool spl06ReadCalibrationCoefficients(void);
static void spl06ConfigureMeasurements(void);
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// 寄存器读取一个字节 非DMA方式 只能用于初始化阶段
static uint8_t spl06ReadByte(uint8_t reg)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&SPL06_I2C_HANDLE, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
	return data;
}

// 寄存器写入一个字节 非DMA方式 只能用于初始化阶段
static void spl06WriteByte(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&SPL06_I2C_HANDLE, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

// 寄存器读取多个字节 非DMA方式 只能用于初始化阶段
static void spl06ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&SPL06_I2C_HANDLE, SPL06_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
}

static int8_t spl06Samples2ConfigRegValue(uint8_t sample_rate)
{
    switch(sample_rate){
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        case 64: return 6;
        case 128: return 7;
        default: return -1; // invalid
    }
}

static int32_t spl06RawValueScaleFactor(uint8_t oversampling_rate)
{
    switch(oversampling_rate){
        case 1: return 524288;
        case 2: return 1572864;
        case 4: return 3670016;
        case 8: return 7864320;
        case 16: return 253952;
        case 32: return 516096;
        case 64: return 1040384;
        case 128: return 2088960;
        default: return -1; // invalid
    }
}

// 开启温度压力测量(连续模式)
static void spl06StartMeasurement(void)
{
    spl06WriteByte(SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_CFG_CONTINUOUS | SPL06_MEAS_TEMPERATURE | SPL06_MEAS_PRESSURE);
}

// 开始温度测量(连续模式)
static void spl06StartTemperatureMeasurement(void)
{
    spl06WriteByte(SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_CFG_CONTINUOUS | SPL06_MEAS_TEMPERATURE);
}

// 开始气压测量(连续模式)
static void spl06StartPressureMeasurement(void)
{
    spl06WriteByte(SPL06_MODE_AND_STATUS_REG, SPL06_MEAS_CFG_CONTINUOUS | SPL06_MEAS_PRESSURE);
}

// 更新原始温度数据(非DMA方式)
static void spl06ReadTemperatureRaw(void)
{
    uint8_t data[SPL06_TEMPERATURE_LEN];
    spl06ReadBuf(SPL06_TEMPERATURE_START_REG, data, SPL06_TEMPERATURE_LEN);
    spl06_temperature_raw = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
}

// 更新原始气压数据(非DMA方式)
static void spl06ReadPressureRaw(void)
{
    uint8_t data[SPL06_PRESSURE_LEN];
    spl06ReadBuf(SPL06_PRESSURE_START_REG, data, SPL06_PRESSURE_LEN);
    spl06_pressure_raw = (int32_t)((data[0] & 0x80 ? 0xFF000000 : 0) | (((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
}

// Returns temperature in degrees centigrade
static float spl06CompensateTemperature(int32_t temperature_raw)
{
    const float t_raw_sc = (float)temperature_raw / spl06RawValueScaleFactor(SPL06_TEMPERATURE_OVERSAMPLING);
    const float temp_comp = (float)spl06_cal.c0 / 2 + t_raw_sc * spl06_cal.c1;
    return temp_comp;
}

// Returns pressure in Pascal
static float spl06CompensatePressure(int32_t pressure_raw, int32_t temperature_raw)
{
    const float p_raw_sc = (float)pressure_raw / spl06RawValueScaleFactor(SPL06_PRESSURE_OVERSAMPLING);
    const float t_raw_sc = (float)temperature_raw / spl06RawValueScaleFactor(SPL06_TEMPERATURE_OVERSAMPLING);
    const float pressure_cal = (float)spl06_cal.c00 + p_raw_sc * ((float)spl06_cal.c10 + p_raw_sc * ((float)spl06_cal.c20 + p_raw_sc * spl06_cal.c30));
    const float p_temp_comp = t_raw_sc * ((float)spl06_cal.c01 + p_raw_sc * ((float)spl06_cal.c11 + p_raw_sc * spl06_cal.c21));
    return pressure_cal + p_temp_comp;
}

// SPL06 根据原始数值输出补偿后数据
static void spl06Calculate(float* pressure, float* temperature)
{
    if(pressure){
        *pressure = spl06CompensatePressure(spl06_pressure_raw, spl06_temperature_raw);
    }
    if(temperature){
        *temperature = spl06CompensateTemperature(spl06_temperature_raw);
    }
}

void spl06Update(float* pressure, float* temperature)
{
    spl06ReadPressureRaw();
    spl06ReadTemperatureRaw();
    spl06Calculate(pressure, temperature);
}

// SPL06 读取内部校准系数 (非DMA方式)
static bool spl06ReadCalibrationCoefficients(void)
{
    uint8_t _status;
    _status = spl06ReadByte(SPL06_MODE_AND_STATUS_REG);
    if(!(_status & SPL06_MEAS_CFG_COEFFS_RDY)){
        return false;   // error reading status or coefficients not ready
    }
    uint8_t caldata[SPL06_CALIB_COEFFS_LEN];
    spl06ReadBuf(SPL06_CALIB_COEFFS_START, caldata, SPL06_CALIB_COEFFS_LEN);
    spl06_cal.c0 = (caldata[0] & 0x80 ? 0xF000 : 0) | ((uint16_t)caldata[0] << 4) | (((uint16_t)caldata[1] & 0xF0) >> 4);
    spl06_cal.c1 = ((caldata[1] & 0x8 ? 0xF000 : 0) | ((uint16_t)caldata[1] & 0x0F) << 8) | (uint16_t)caldata[2];
    spl06_cal.c00 = (caldata[3] & 0x80 ? 0xFFF00000 : 0) | ((uint32_t)caldata[3] << 12) | ((uint32_t)caldata[4] << 4) | (((uint32_t)caldata[5] & 0xF0) >> 4);
    spl06_cal.c10 = (caldata[5] & 0x8 ? 0xFFF00000 : 0) | (((uint32_t)caldata[5] & 0x0F) << 16) | ((uint32_t)caldata[6] << 8) | (uint32_t)caldata[7];
    spl06_cal.c01 = ((uint16_t)caldata[8] << 8) | ((uint16_t)caldata[9]);
    spl06_cal.c11 = ((uint16_t)caldata[10] << 8) | (uint16_t)caldata[11];
    spl06_cal.c20 = ((uint16_t)caldata[12] << 8) | (uint16_t)caldata[13];
    spl06_cal.c21 = ((uint16_t)caldata[14] << 8) | (uint16_t)caldata[15];
    spl06_cal.c30 = ((uint16_t)caldata[16] << 8) | (uint16_t)caldata[17];
    return true;
}

// SPL06 配置测量参数
static void spl06ConfigureMeasurements(void)
{
    uint8_t reg_value;

    // 温度测量配置
    // reg_value = SPL06_TEMP_USE_EXT_SENSOR | spl06Samples2ConfigRegValue(SPL06_TEMPERATURE_OVERSAMPLING);
    reg_value = SPL06_TEMP_USE_EXT_SENSOR | TMP_RATE_32MPS | TMP_PRC_8X; // 3.6ms * 32 = 115.2ms
    spl06WriteByte(SPL06_TEMPERATURE_CFG_REG, reg_value);

    // 压力测量配置
    // reg_value = spl06Samples2ConfigRegValue(SPL06_PRESSURE_OVERSAMPLING);
    reg_value = PM_RATE_32MPS | PM_PRC_8X; // 14.8ms * 32 = 473.6ms
    spl06WriteByte(SPL06_PRESSURE_CFG_REG, reg_value);

    // 过采样特殊情况处理
    reg_value = 0;
    if(SPL06_TEMPERATURE_OVERSAMPLING > 8){reg_value |= SPL06_TEMPERATURE_RESULT_BIT_SHIFT;}
    if(SPL06_PRESSURE_OVERSAMPLING > 8){reg_value |= SPL06_PRESSURE_RESULT_BIT_SHIFT;}

    // 配置SPL06中断输出
    // 根据数据手册说明 因为SPL06中断引脚和SDO以及I2C地址共享引脚
    // 并且SDO引脚一般会存在弱上拉电阻或者是设计者添加的下拉电阻
    // 所以中断输出的有效电平要和对应常规电平相反
    // 例如此引脚含有上拉电阻(一般情况)则有效电平应该为低电平
    reg_value |= SPL06_INT_ACTIVE_LOW; // 中断输出有效电平为低电平
    reg_value |= SPL06_INT_PRS_ENABLE; // 压力数据中断使能
    spl06WriteByte(SPL06_INT_AND_FIFO_CFG_REG, reg_value);

    spl06StartMeasurement(); // 启动测量
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

// DMA 非阻塞更新磁力计数据
void spl06DmaUpdateStart(void)
{
    HAL_I2C_Mem_Read_DMA(
        &SPL06_I2C_HANDLE, SPL06_I2C_ADDR, 
        SPL06_PRESSURE_B2_REG, I2C_MEMADD_SIZE_8BIT, spl06_buf, SPL06_DATA_BUFFER_SIZE);
}

// DMA 非阻塞更新完毕回调函数 ISR
void spl06DmaUpdateCallbackISR(void)
{
    spl06_data_t _data;

    spl06_pressure_raw      = (int32_t)((spl06_buf[0]&0x80?0xFF000000:0)|(((uint32_t)(spl06_buf[0])) << 16) | (((uint32_t)(spl06_buf[1])) << 8) | ((uint32_t)spl06_buf[2]));
    spl06_temperature_raw   = (int32_t)((spl06_buf[3]&0x80?0xFF000000:0)|(((uint32_t)(spl06_buf[3])) << 16) | (((uint32_t)(spl06_buf[4])) << 8) | ((uint32_t)spl06_buf[5]));
    spl06_pressure      = spl06CompensatePressure(spl06_pressure_raw, spl06_temperature_raw);
    spl06_temperature   = spl06CompensateTemperature(spl06_temperature_raw);

    _data.pressure = spl06_pressure;
    _data.temperature = spl06_temperature;
    // 这里将_data数据放入队列中
}

bool spl06Probe(void)
{
	uint8_t id = spl06ReadByte(SPL06_CHIP_ID_REG);
	if(id == SPL06_DEFAULT_CHIP_ID){
		slog("SPL06 Device id:0x%x\n", id);
		return true;
	}else{
		slog("SPL06 Wrong Device id:0x%x\n", id);
		return false;
	}
}

bool spl06Init(void)
{
    if(!spl06Probe()){
		slog("Error: SPL06 Not Found\r\n");
		return false;
	}
    if(!spl06ReadCalibrationCoefficients()){
        slog("Error: SPL06 Read Calibration Coefficients Failed\r\n");
        return false;
    }
    spl06ConfigureMeasurements();
    return true;
}

// 根据气压计压强计算海拔高度
float spl06GetAltitude(float pressure_Pa)
{
    float pressure_hPa = pressure_Pa / 100.0f; // 转换为百帕
    float altitude = 44330 * (1.0 - powf(pressure_hPa / SEA_LEVEL_PRESSURE, 0.1903));
    return altitude;
}

// 获取中断引脚电平
bool spl06ExtiLow(void)
{
    return (!HAL_GPIO_ReadPin(SPL06_DR_GPIO_Port, SPL06_DR_Pin));
}

// 查询中断状态寄存器
uint8_t spl06GetIntStatus(void)
{
    return spl06ReadByte(SPL06_INT_STATUS_REG);
}

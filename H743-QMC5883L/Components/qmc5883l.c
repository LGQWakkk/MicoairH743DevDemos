// 20250919 Wakkk
// STM32H743 QMC5883L TEST
#include "qmc5883l.h"

// 由于大多数方案都没有使用QMC5883L的中断触发读取方案 MicoairH743同样
// 大多数方案直接使用定期读取数据的方案

static uint8_t qmc5883lReadByte(uint8_t reg);
static void qmc5883lReadBuf(uint8_t reg, uint8_t *buf, uint8_t len);
static void qmc5883lWriteByte(uint8_t reg, uint8_t data);
static bool qmc5883lProbe(void);

static bool radioSendMagDataFlag = false; // 读取数据时是否发送磁力计数据
static qmc5883l_calib_data_t calib_data_static; // 保存校准数据

// DMA RX BUFFER
#define QMC5883L_DATA_BUFFER_SIZE 9
static uint8_t qmc5883l_data_buffer[QMC5883L_DATA_BUFFER_SIZE]; // mag + temp

//////////////////////////////////////////////////////////////////////////////////////////////

// DMA 非阻塞更新磁力计数据
void qmc5883lDmaUpdateStart(void)
{
    HAL_I2C_Mem_Read_DMA(
        &QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, 
        REG_DATA_OUT_X_L, I2C_MEMADD_SIZE_8BIT, qmc5883l_data_buffer, QMC5883L_DATA_BUFFER_SIZE);
}

// DMA 非阻塞更新完毕回调函数 ISR
void qmc5883lDmaUpdateCallbackISR(void)
{
    qmc5883l_data_t _data;
    _data.mx = (float)((int16_t)(((uint16_t)qmc5883l_data_buffer[1] << 8) | qmc5883l_data_buffer[0])) / QMC5883L_8G_SCALE;
    _data.my = (float)((int16_t)(((uint16_t)qmc5883l_data_buffer[3] << 8) | qmc5883l_data_buffer[2])) / QMC5883L_8G_SCALE;
    _data.mz = (float)((int16_t)(((uint16_t)qmc5883l_data_buffer[5] << 8) | qmc5883l_data_buffer[4])) / QMC5883L_8G_SCALE;
    int16_t temp = (int16_t)(((uint16_t)qmc5883l_data_buffer[8] << 8) | qmc5883l_data_buffer[7]);
    _data.temp = (float)temp / QMC5883L_TMP_SCALE;
    // 这里可以将_data存入队列
}

// I2C 直接读取数据
void qmc5883lUpdateData(qmc5883l_data_t *data)
{
    qmc5883lReadBuf(REG_DATA_OUT_X_L, qmc5883l_data_buffer, QMC5883L_DATA_BUFFER_SIZE);
    data->mx = (float)((int16_t)(((uint16_t)qmc5883l_data_buffer[1] << 8) | qmc5883l_data_buffer[0])) / QMC5883L_8G_SCALE;
    data->my = (float)((int16_t)(((uint16_t)qmc5883l_data_buffer[3] << 8) | qmc5883l_data_buffer[2])) / QMC5883L_8G_SCALE;
    data->mz = (float)((int16_t)(((uint16_t)qmc5883l_data_buffer[5] << 8) | qmc5883l_data_buffer[4])) / QMC5883L_8G_SCALE;
    // 注意温度数据仅仅是相对温度 不代表实际绝对温度数值
    int16_t temp = (int16_t)(((uint16_t)qmc5883l_data_buffer[8] << 8) | qmc5883l_data_buffer[7]);
    data->temp = (float)temp / QMC5883L_TMP_SCALE;
}

void qmc5883lDebugData(void)
{
    qmc5883lReadBuf(REG_DATA_OUT_X_L, qmc5883l_data_buffer, QMC5883L_DATA_BUFFER_SIZE);
    slogDma("debug: %x %x %x %x %x %x %x %x %x\r\n", 
        qmc5883l_data_buffer[0], qmc5883l_data_buffer[1], qmc5883l_data_buffer[2],
        qmc5883l_data_buffer[3], qmc5883l_data_buffer[4], qmc5883l_data_buffer[5],
        qmc5883l_data_buffer[6], qmc5883l_data_buffer[7], qmc5883l_data_buffer[8]);
}

// 初始化QMC5883L
bool qmc5883lInit(void)
{
    bool ret = qmc5883lProbe();
    if(!ret){
        slog("Error: QMC5883L Not Found\r\n");
        return false;
    }
    qmc5883lWriteByte(REG_CONTROL2, 0x80);  // 软件复位
    qmc5883lWriteByte(REG_PERIOD, 0x01);    // 设置为连续测量模式
    uint8_t config = QMC5883L_MODE_CONTINUOUS | QMC5883L_ODR_100HZ | QMC5883L_RNG_8G | QMC5883L_OSR_512;
    qmc5883lWriteByte(REG_CONTROL1, config); // 配置
    return true;
}

// 对磁力计施加校准数据 返回值为校准之后的模值(Gauss)
float qmc5883lCalibData(qmc5883l_data_t *data, qmc5883l_calib_data_t *calib_data)
{
    float mx_corr = (data->mx - calib_data->x_bias) / calib_data->x_scale;
    float my_corr = (data->my - calib_data->y_bias) / calib_data->y_scale;
    float mz_corr = (data->mz - calib_data->z_bias) / calib_data->z_scale;
    float norm_corr = calib_data->average_scale * sqrt(mx_corr*mx_corr + my_corr*my_corr + mz_corr*mz_corr);
    // 更新校准之后的数值 Gauss
    data->mx = mx_corr * calib_data->average_scale;
    data->my = my_corr * calib_data->average_scale;
    data->mz = mz_corr * calib_data->average_scale;
    return norm_corr;
}

// 对磁力计施加校准数据 返回值为校准之后的模值(Gauss)
float qmc5883lCalibDataStatic(qmc5883l_data_t *data)
{
    float mx_corr = (data->mx - calib_data_static.x_bias) / calib_data_static.x_scale;
    float my_corr = (data->my - calib_data_static.y_bias) / calib_data_static.y_scale;
    float mz_corr = (data->mz - calib_data_static.z_bias) / calib_data_static.z_scale;
    float norm_corr = calib_data_static.average_scale * sqrt(mx_corr*mx_corr + my_corr*my_corr + mz_corr*mz_corr);
    // 更新校准之后的数值 Gauss
    data->mx = mx_corr * calib_data_static.average_scale;
    data->my = my_corr * calib_data_static.average_scale;
    data->mz = mz_corr * calib_data_static.average_scale;
    return norm_corr;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// 寄存器读取一个字节 非DMA方式 只能用于初始化阶段
static uint8_t qmc5883lReadByte(uint8_t reg)
{
	uint8_t data;
	HAL_I2C_Mem_Read(&QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    return data;
}
// 寄存器读取多个字节 非DMA方式 只能用于初始化阶段
static void qmc5883lReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
}
// 寄存器写入一个字节 非DMA方式 只能用于初始化阶段
static void qmc5883lWriteByte(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&QMC5883L_I2C_HANDLE, QMC5883L_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}
// 检测QMC5883L是否位于总线上
static bool qmc5883lProbe(void)
{
    uint8_t id = qmc5883lReadByte(REG_CHIP_ID);
    if(id == 0xFF){
        slog("QMC5883L Device id:0x%x\n", id);
        return true;
    }else{
        slog("QMC5883L Wrong Device id:0x%x\n", id);
        return false;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////

uint8_t qmc5883lGetState(void)
{
    return qmc5883lReadByte(REG_STATUS);
}

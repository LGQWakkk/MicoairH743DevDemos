// 20250919 Wakkk
#include "system.h"

// BMI088 CS 相关定义
#define CS_ACC_LOW() HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET)
#define CS_ACC_HIGH() HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET)
#define CS_GYRO_LOW() HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET)
#define CS_GYRO_HIGH() HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET)

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

struct bmi08_dev bmi08dev; /*! @brief This structure containing relevant bmi08 info */
struct bmi08_accel_int_channel_cfg accel_int_config; /*! bmi08 accel int config */
struct bmi08_gyro_int_channel_cfg gyro_int_config; /*! bmi08 gyro int config */
uint8_t bmi088_spi_gyro_cs;
uint8_t bmi088_spi_accel_cs;

// SPI 读取指定长度数据
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if(intf_ptr == NULL)return BMI08_E_NULL_PTR;
    if(intf_ptr == &bmi088_spi_accel_cs){
        CS_ACC_LOW();
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg_addr, 1, 1000);
        HAL_SPI_Receive(&BMI088_SPI_HANDLE, reg_data, len, 1000);
        CS_ACC_HIGH();
        return BMI08_INTF_RET_SUCCESS;
    }else if(intf_ptr == &bmi088_spi_gyro_cs){
        CS_GYRO_LOW();
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg_addr, 1, 1000);
        HAL_SPI_Receive(&BMI088_SPI_HANDLE, reg_data, len, 1000);
        CS_GYRO_HIGH();
        return BMI08_INTF_RET_SUCCESS;
    }else{
        return BMI08_E_NULL_PTR;
    }
}

// SPI 写入指定长度数据
BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if(intf_ptr == NULL)return BMI08_E_NULL_PTR;
    if(intf_ptr == &bmi088_spi_accel_cs){
        CS_ACC_LOW();
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg_addr, 1, 1000);
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, (uint8_t *)reg_data, len, 1000);
        CS_ACC_HIGH();
        return BMI08_INTF_RET_SUCCESS;
    }else if(intf_ptr == &bmi088_spi_gyro_cs){
        CS_GYRO_LOW();
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, &reg_addr, 1, 1000);
        HAL_SPI_Transmit(&BMI088_SPI_HANDLE, (uint8_t *)reg_data, len, 1000);
        CS_GYRO_HIGH();
        return BMI08_INTF_RET_SUCCESS;
    }else{
        return BMI08_E_NULL_PTR;
    }
}

// 使用ms替代方案
void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if(period < 1000){
        HAL_Delay(1);
    }else{
        HAL_Delay(period/1000);
    }
}

// 使能数据有效中断
// BMI088中断设置为推挽输出模式
// 均为高电平有效
// 因为是推挽输出模式 所以MCU不需要上下拉
// MCU配置为上升沿触发中断
static int8_t enable_bmi08_interrupt(void)
{
    int8_t rslt;
    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1; // INT1引脚
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi08a_set_int_config fail\r\n");
    }

    /* Set gyro interrupt pin configuration */
    gyro_int_config.int_channel = BMI08_INT_CHANNEL_3; // INT3引脚
    gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
    gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
    /* Enable gyro data ready interrupt channel */
    rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi08g_set_int_config fail\r\n");
    }

    return rslt;
}

// 初始化函数接口
int8_t bmi08_interface_init(struct bmi08_dev *bmi08dev)
{
    int8_t rslt = BMI08_OK;
    if(bmi08dev != NULL){
        bmi08dev->write = bmi08_spi_write;
        bmi08dev->read = bmi08_spi_read;
        bmi08dev->intf = BMI08_SPI_INTF;
        bmi08dev->variant = BMI088_VARIANT;
        bmi08dev->delay_us = bmi08_delay_us;
        bmi08dev->read_write_len = 1024;
        bmi08dev->intf_ptr_accel = &bmi088_spi_accel_cs;
        bmi08dev->intf_ptr_gyro = &bmi088_spi_gyro_cs;
    }else{
        rslt = BMI08_E_NULL_PTR;
    }
    return rslt;
}

// BMI088 初始化函数
static int8_t init_bmi08(void)
{
    int8_t rslt;
    rslt = bmi08xa_init(&bmi08dev); // 初始化ACC
    if(rslt != BMI08_OK){
        slog("bmi088 acc init failed\r\n");
        return rslt;
    }

    rslt = bmi08g_init(&bmi08dev); // 初始化GYRO
    if(rslt != BMI08_OK){
        slog("bmi088 gyro init failed\r\n");
        return rslt;
    }

    slog("Uploading config file\n");
    rslt = bmi08a_load_config_file(&bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi088 load config file fail\r\n");
        return rslt;
    }

    slog("Accel chip ID - 0x%x\n", bmi08dev.accel_chip_id); // 0x1E
    slog("Gyro chip ID - 0x%x\n", bmi08dev.gyro_chip_id); // 0x0F

    bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;
    bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;

    rslt = bmi08a_set_power_mode(&bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi088 set accel power mode fail\r\n");
        return rslt;
    }

    rslt = bmi08xa_set_meas_conf(&bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi088 set accel meas conf fail\r\n");
        return rslt;
    }

    bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;
    bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_250_DPS;
    bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;
    bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    rslt = bmi08g_set_power_mode(&bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi088 set gyro power mode fail\r\n");
        return rslt;
    }

    rslt = bmi08g_set_meas_conf(&bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi088 set gyro meas conf fail\r\n");
        return rslt;
    }

    if ((rslt == BMI08_OK) &&
        (bmi08dev.accel_cfg.power == BMI08_ACCEL_PM_SUSPEND &&
        (bmi08dev.gyro_cfg.power == BMI08_GYRO_PM_SUSPEND ||
        bmi08dev.gyro_cfg.power == BMI08_GYRO_PM_DEEP_SUSPEND))){
        slog("Accel and gyro sensors are in suspend mode\n Use them in active/normal mode !!");
    }
    return rslt;
}

void systemInit(void)
{
    slog("System Init\r\n");
    HAL_Delay(500);

    int8_t rslt;
    rslt = bmi08_interface_init(&bmi08dev);
    if(rslt != BMI08_OK){
        slog("bmi08 interface init fail\r\n");
    }
    // BMI088 初始化
    rslt = init_bmi08();
    if(rslt != BMI08_OK){
        slog("bmi08 init fail\r\n");
    }
    // 使能数据有效中断
    rslt = enable_bmi08_interrupt();
    if(rslt != BMI08_OK){
        slog("enable bmi08 interrupt fail\r\n");
    }
    slog("BMI088 Init Done\r\n");

}

// 中断次数计数
uint32_t accIntCount = 0;
uint32_t gyroIntCount = 0;
bool accIntFlag = false;
bool gyroIntFlag = false;

void systemLoop(void)
{
    slogDma("accIntCount:%d gyroIntCount:%d\r\n", accIntCount, gyroIntCount);
    HAL_Delay(100);
}

// BMI088 EXTI 中断回调处理
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BMI088_ACCEL_DR_Pin){ // ACC 数据有效中断
        accIntFlag = true;
        accIntCount++;
    }else if(GPIO_Pin == BMI088_GYRO_DR_Pin){ // GYRO 数据有效中断
        gyroIntFlag = true;
        gyroIntCount++;
    }
}

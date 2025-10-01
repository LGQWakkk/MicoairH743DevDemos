// 20251001 Wakkk
// BMI270 1600Hz ACC GYRO 中断读取
// BMI270 SPI3

#include "system.h"

volatile uint8_t drdy_int_status = 0; // BMI270 中断有效标志位
volatile uint32_t drdy_int_cnt = 0; // BMI270 中断计数器

// BMI270 EXTI 中断回调处理
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == BMI270_DR_Pin){ // ACC 数据有效中断
        drdy_int_status = 1; // 设置中断有效标志位
        drdy_int_cnt += 1;   // 计数器加1
    }
}

// BMI270 CS 相关定义
#define CS_LOW() HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(BMI270_CS_GPIO_Port, BMI270_CS_Pin, GPIO_PIN_SET)

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;
    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));
    return (dps / (half_scale)) * (val);
}

// 配置 ACC GYRO 相关参数
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    int8_t rslt;
    struct bmi2_int_pin_config pin_config = { 0 }; // 中断引脚配置结构体
    struct bmi2_sens_config config[2]; // 定义ACC GYRO的配置结构体

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configuration for hardware Interrupt */
    rslt = bmi2_get_int_pin_config(&pin_config, bmi);
    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi); // 设置数据有效中断引脚

    if(rslt == BMI2_OK){
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ; // 设置ACC数据输出速率
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_8G; // 设置ACC量程

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ; // 设置GYRO数据输出速率
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000; // 设置GYRO量程

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE; // BMI2_GYR_OSR2_MODE BMI2_GYR_OSR4_MODE

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        // 中断引脚配置
        pin_config.pin_type = BMI2_INT1;
        pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE; // 禁止输入
        pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW; // 低电平有效
        pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL; // 推挽输出
        pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE; // 使能输出
        pin_config.int_latch = BMI2_INT_NON_LATCH;
        rslt = bmi2_set_int_pin_config(&pin_config, bmi); // 设置中断引脚配置
        rslt = bmi2_set_sensor_config(config, 2, bmi); // 配置ACC与GYRO
    }
    return rslt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////////////////////////////

// SPI 读取指定长度数据
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    CS_LOW();
    HAL_SPI_Transmit(&BMI270_SPI_HANDLE, &reg_addr, 1, 1000);
    HAL_SPI_Receive(&BMI270_SPI_HANDLE, reg_data, len, 1000);
    CS_HIGH();
    return BMI2_INTF_RET_SUCCESS;
}

// SPI 写入指定长度数据
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    CS_LOW();
    HAL_SPI_Transmit(&BMI270_SPI_HANDLE, &reg_addr, 1, 1000);
    HAL_SPI_Transmit(&BMI270_SPI_HANDLE, (uint8_t *)reg_data, len, 1000);
    CS_HIGH();
    return BMI2_INTF_RET_SUCCESS;
}

// 使用ms替代方案
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if(period < 1000){
        HAL_Delay(1);
    }else{
        HAL_Delay(period/1000);
    }
}

// BMI270 初始化函数接口
int8_t bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{
    int8_t rslt = BMI2_OK;
    if(bmi != NULL){
        bmi->write = bmi2_spi_write;
        bmi->read = bmi2_spi_read;
        bmi->intf = BMI2_SPI_INTF;
        bmi->delay_us = bmi2_delay_us;
        bmi->read_write_len = 1024;
        bmi->config_file_ptr = NULL; /* Assign to NULL to load the default config file. */
    }else{
        rslt = NULL;
    }
    return rslt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// 20251001 相关测试数据结构
int8_t rslt;
uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO }; /* Assign accel and gyro sensor to variable. */
struct bmi2_dev bmi; // 传感器初始化配置
/* Structure to define type of sensor and their respective data. */
struct bmi2_sens_data sensor_data = { { 0 } };
float acc_x = 0, acc_y = 0, acc_z = 0;
float gyr_x = 0, gyr_y = 0, gyr_z = 0;

void systemInit(void)
{
    slog("System Init\r\n");
    HAL_Delay(500);

    rslt = bmi2_interface_init(&bmi, BMI2_SPI_INTF); // 初始化函数接口
    if(rslt != BMI2_OK){
        slog("bmi2 interface init fail\r\n");
        return;
    }
    rslt = bmi270_init(&bmi); // 传感器初始化
    if(rslt != BMI2_OK){
        slog("bmi270 init fail\r\n");
        return;
    }
    rslt = set_accel_gyro_config(&bmi); // 配置传感器
    if(rslt != BMI2_OK){
        slog("bmi270 set accel gyro config fail\r\n");
        return;
    }
    rslt = bmi2_sensor_enable(sensor_list, 2, &bmi); // Accel and Gyro enable must be done after setting configurations
    if(rslt != BMI2_OK){
        slog("bmi270 sensor enable fail\r\n");
        return;
    }
    slog("System Init Done\r\n");
}

// 20251001 自行实现的读取函数 避免无效延时
void bmi2_get_regs_my(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi2_dev *dev)
{
    uint16_t index = 0;
    uint8_t temp_buf[BMI2_MAX_LEN];
    /* Configuring reg_addr for SPI Interface */
    if(dev->intf == BMI2_SPI_INTF){
        reg_addr = (reg_addr | BMI2_SPI_RD_MASK);
    }
    dev->intf_rslt = dev->read(reg_addr, temp_buf, (len + dev->dummy_byte), dev->intf_ptr);
    if(dev->intf_rslt == BMI2_INTF_RET_SUCCESS){
        /* Read the data from the position next to dummy byte */
        while(index < len){
            data[index] = temp_buf[index + dev->dummy_byte];
            index++;
        }
    }
}

// 20251001 自行实现的读取函数 避免无效延时
void bmi2_get_data(struct bmi2_sens_data *data, struct bmi2_dev *dev)
{
    uint8_t sensor_data[BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES];
    bmi2_get_regs_my(BMI2_STATUS_ADDR, sensor_data, BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES, dev);
    bmi2_parse_sensor_data(sensor_data, data, dev);
}

// Data set, Accel Range, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyro_DPS_X, Gyro_DPS_Y, Gyro_DPS_Z
void systemLoop(void)
{
    // HAL_Delay(1000);
    // slogDma("count: %d\n", drdy_int_cnt); // 20251001 经过实际检测 中断次数符合1600HZ
    // return;

    if(drdy_int_status == 1){
        drdy_int_status = 0;

        // rslt = bmi2_get_sensor_data(&sensor_data, &bmi);

        bmi2_get_data(&sensor_data, &bmi); // 读取传感器数据 自行实现

        // 20251001 使用下面方式读取 实际测试输出频率为500Hz
        // if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) &&
        //     (sensor_data.status & BMI2_DRDY_GYR))
        // {
        //     acc_x = lsb_to_mps2(sensor_data.acc.x, (float)8, bmi.resolution);
        //     acc_y = lsb_to_mps2(sensor_data.acc.y, (float)8, bmi.resolution);
        //     acc_z = lsb_to_mps2(sensor_data.acc.z, (float)8, bmi.resolution);
        //     gyr_x = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi.resolution);
        //     gyr_y = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi.resolution);
        //     gyr_z = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi.resolution);
        //     // slogDma("%f,%f,%f,%f,%f,%f\n",acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z);
        //     slogDma("%f,%f,%f\n",gyr_x,gyr_y,gyr_z);
        // }

        // 20251001 使用下面方式读取 实际测试输出频率仍为500Hz 说明不是数据有效性问题
        acc_x = lsb_to_mps2(sensor_data.acc.x, (float)8, bmi.resolution);
        acc_y = lsb_to_mps2(sensor_data.acc.y, (float)8, bmi.resolution);
        acc_z = lsb_to_mps2(sensor_data.acc.z, (float)8, bmi.resolution);
        gyr_x = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi.resolution);
        gyr_y = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi.resolution);
        gyr_z = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi.resolution);
        slogDma("%f,%f,%f\n",gyr_x,gyr_y,gyr_z);
    }
}

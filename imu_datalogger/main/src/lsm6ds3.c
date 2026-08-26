#include "lsm6ds3.h"

static uint8_t data[12];
float gyro_fs_cf, accel_fs_cf;

/**
 * @brief Read a sequence of bytes from a LSM6DS3 sensor registers
 *
 * @param adev_handle i2c handle of LSM6DS3
 * @param reg_addr memory register address
 * @param data pointer to destination
 * @param len number of bytes to read
 */
esp_err_t lsm6ds3_register_read(i2c_master_dev_handle_t dev_handle,
                                uint8_t reg_addr,
                                uint8_t *data,
                                size_t len)
{
    /* transmit one byte (the register address on the LSM6DS3) and receive multiple bytes back from the LSM6DS3. */
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a LSM6DS3 sensor register
 *
 * @param adev_handle i2c handle of LSM6DS3
 * @param reg_addr memory register address
 * @param data pointer to source
 */
esp_err_t lsm6ds3_register_write_byte(i2c_master_dev_handle_t dev_handle,
                                      uint8_t reg_addr,
                                      uint8_t data)
{
    /* write the register address first, then the data to write to the LSM6DS3. */
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief initialize LSM6DS3 accelerometer
 */
void lsm6ds3_init_accel(i2c_master_dev_handle_t dev_handle, uint16_t odr, uint16_t xl_fs)
{
    /* set accelerometer power to normal power */
    lsm6ds3_set_accel_power(dev_handle, LSM6DS3_XL_HM_MODE_HP);

    /* set accelerometer FS */
    switch (xl_fs)
    {
    case 2:
        lsm6ds3_set_accel_fs(dev_handle, LSM6DS3_XL_FS_2G);
        break;
    case 4:
        lsm6ds3_set_accel_fs(dev_handle, LSM6DS3_XL_FS_4G);
        break;
    case 8:
        lsm6ds3_set_accel_fs(dev_handle, LSM6DS3_XL_FS_8G);
        break;
    case 16:
        lsm6ds3_set_accel_fs(dev_handle, LSM6DS3_XL_FS_16G);
        break;
    default:
        lsm6ds3_set_accel_fs(dev_handle, LSM6DS3_DEFAULT_XL_FS);
    }

    /* set accelerometer ODR */
    switch (odr)
    {
    case 12:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_12_5HZ);
        break;
    case 26:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_26HZ);
        break;
    case 52:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_52HZ);
        break;
    case 104:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_104HZ);
        break;
    case 208:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_208HZ);
        break;
    case 416:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_416HZ);
        break;
    case 833:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_ODR_833HZ);
        break;
    default:
        lsm6ds3_set_accel_odr(dev_handle, LSM6DS3_DEFAULT_ODR);
    }
}

/**
 * @brief initialize LSM6DS3 gyroscope
 */
void lsm6ds3_init_gyro(i2c_master_dev_handle_t dev_handle, uint16_t odr, uint16_t g_fs)
{
    /* set gyroscope power to normal power */
    lsm6ds3_set_gyro_power(dev_handle, LSM6DS3_G_HM_MODE_HP);

    /* set gyroscope FS */
    switch (g_fs)
    {
    case 245:
        lsm6ds3_set_gyro_fs(dev_handle, LSM6DS3_G_FS_245DPS);
        break;
    case 500:
        lsm6ds3_set_gyro_fs(dev_handle, LSM6DS3_G_FS_500DPS);
        break;
    case 1000:
        lsm6ds3_set_gyro_fs(dev_handle, LSM6DS3_G_FS_1000DPS);
        break;
    case 2000:
        lsm6ds3_set_gyro_fs(dev_handle, LSM6DS3_G_FS_2000DPS);
        break;
    default:
        lsm6ds3_set_gyro_fs(dev_handle, LSM6DS3_DEFAULT_G_FS);
    }

    /* set gyroscope ODR */
    switch (odr)
    {
    case 12:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_12_5HZ);
        break;
    case 26:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_26HZ);
        break;
    case 52:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_52HZ);
        break;
    case 104:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_104HZ);
        break;
    case 208:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_208HZ);
        break;
    case 416:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_416HZ);
        break;
    case 833:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_ODR_833HZ);
        break;
    default:
        lsm6ds3_set_gyro_odr(dev_handle, LSM6DS3_DEFAULT_ODR);
    }
}

/**
 * @brief initialize LSM6DS3
 */
void lsm6ds3_init_all(i2c_master_dev_handle_t dev_handle, uint8_t odr, uint16_t xl_fs, uint16_t g_fs)
{
    /* reset lsm6ds3 */
    lsm6ds3_reset(dev_handle);

    /* read the WHO_AM_I register fromthe LSM6DS3
    WHO_AM_I must be 0x69 */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(LSM6DS3_TAG, "WHO_AM_I = %X", data[0]);

    /* initialize accelerometer */
    lsm6ds3_init_accel(dev_handle, odr, xl_fs);
    /* initialize gyroscope */
    lsm6ds3_init_gyro(dev_handle, odr, g_fs);

    /* enable block data update (BDU) */
    lsm6ds3_enable_bdu(dev_handle);
}

/**
 * @brief reset LSM6DS3
 */
void lsm6ds3_reset(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle,
                                                LSM6DS3_CTRL3_C_REG_ADDR,
                                                LSM6DS3_CTRL3_C_SW_RESET));
}

/**
 * @brief configure LSM6DS3 accelerometer power mode
 */
void lsm6ds3_set_accel_power(i2c_master_dev_handle_t dev_handle, lsm6ds3_xl_hm xl_hm)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL6_C_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(CTRL6_C_XL_HM_MODE, CTRL6_C_XL_HM_MODE, xl_hm, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL6_C_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 accelerometer full scale (FS)
 *
 * @param
 */
void lsm6ds3_set_accel_fs(i2c_master_dev_handle_t dev_handle, lsm6ds3_xl_fs xl_fs)
{
    switch (xl_fs)
    {
    case LSM6DS3_XL_FS_2G:
        accel_fs_cf = 2;
        break;
    case LSM6DS3_XL_FS_4G:
        accel_fs_cf = 4;
        break;
    case LSM6DS3_XL_FS_8G:
        accel_fs_cf = 8;
        break;
    case LSM6DS3_XL_FS_16G:
        accel_fs_cf = 16;
        break;
    default:
        accel_fs_cf = 1;
    }
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(CTRL1_XL_FS_XL1, CTRL1_XL_FS_XL0, xl_fs, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 accelerometer output data rate (ODR)
 *
 * @param
 */
void lsm6ds3_set_accel_odr(i2c_master_dev_handle_t dev_handle, lsm6ds3_odr xl_odr)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(CTRL1_XL_ODR_XL3, CTRL1_XL_ODR_XL0, xl_odr, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL1_XL_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 gyroscope power mode
 */
void lsm6ds3_set_gyro_power(i2c_master_dev_handle_t dev_handle, lsm6ds3_g_hm g_hm)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL7_G_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(CTRL7_G_G_HM_MODE, CTRL7_G_G_HM_MODE, g_hm, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL7_G_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 gyroscope full scale (FS)
 *
 * @param
 */
void lsm6ds3_set_gyro_fs(i2c_master_dev_handle_t dev_handle, lsm6ds3_g_fs g_fs)
{
    switch (g_fs)
    {
    case LSM6DS3_G_FS_245DPS:
        gyro_fs_cf = 245;
        break;
    case LSM6DS3_G_FS_500DPS:
        gyro_fs_cf = 500;
        break;
    case LSM6DS3_G_FS_1000DPS:
        gyro_fs_cf = 1000;
        break;
    case LSM6DS3_G_FS_2000DPS:
        gyro_fs_cf = 2000;
        break;
    default:
        gyro_fs_cf = 1;
    }
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(CTRL2_G_FS_G1, CTRL2_G_FS_G0, g_fs, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 gyroscope output data rate (ODR)
 */
void lsm6ds3_set_gyro_odr(i2c_master_dev_handle_t dev_handle, lsm6ds3_odr g_odr)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(CTRL2_G_ODR_G3, CTRL2_G_ODR_G0, g_odr, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL2_G_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 FIFO threshold value
 */
void lsm6ds3_set_fifo_thresh(i2c_master_dev_handle_t dev_handle, uint16_t fifo_thresh)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    /* modify */
    read_modify_write(FIFO_CTRL1_FTH_7, FIFO_CTRL1_FTH_0, fifo_thresh & 0xFF, &data[0]);
    read_modify_write(FIFO_CTRL2_FTH_10, FIFO_CTRL2_FTH_8, (fifo_thresh >> 8) & 0x7, &data[1]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, data[0]));
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, data[1]));
}

/**
 * @brief lsm6ds3 reset FIFO contents
 */
void lsm6ds3_fifo_reset(i2c_master_dev_handle_t dev_handle)
{
    lsm6ds3_set_fifo_operation_mode(dev_handle, LSM6DS3_FIFO_MODE_BYPASS);
}

/**
 * @brief configure LSM6DS3 output data rate (ODR)
 */
void lsm6ds3_set_fifo_odr(i2c_master_dev_handle_t dev_handle, lsm6ds3_odr fifo_odr)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(FIFO_CTRL5_ODR_3, FIFO_CTRL5_ODR_0, fifo_odr, &data[0]);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data[0]));
}

/**
 * @brief configure LSM6DS3 FIFO operation mode
 */
void lsm6ds3_set_fifo_operation_mode(i2c_master_dev_handle_t dev_handle, lsm6ds3_fifo_mode fifo_mode)
{
    /* read */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data, 1));
    /* modify */
    read_modify_write(FIFO_CTRL5_MODE_2, FIFO_CTRL5_MODE_0, fifo_mode, data);
    /* write */
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, data[0]));
}

/**
 * @brief check if new temperature data is available from the LSM6DS3
 */
uint8_t lsm6ds3_check_temp_data_available(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_STATUS_REG_ADDR, data, 1));
    return read_bit(data[0], STATUS_REG_TDA);
}

/**
 * @brief check if new gyroscope data is available from the LSM6DS3
 */
uint8_t lsm6ds3_check_gyro_data_available(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_STATUS_REG_ADDR, data, 1));
    return read_bit(data[0], STATUS_REG_GDA);
}

/**
 * @brief check if new accelerometer data is available from the LSM6DS3
 */
uint8_t lsm6ds3_check_accel_data_available(i2c_master_dev_handle_t dev_handle)
{
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_STATUS_REG_ADDR, data, 1));
    return read_bit(data[0], STATUS_REG_XLDA);
}

/**
 * @brief read temperature from LSM6DS3
 */
void lsm6ds3_read_temperature(i2c_master_dev_handle_t dev_handle, float *temp)
{
    int16_t temp_raw;
    float temp1;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_OUT_TEMP_L_REG_ADDR, data, 2));
    temp_raw = (data[1] << 8) | data[0];
    ESP_LOGI(LSM6DS3_TAG, "temp_raw = %d", temp_raw);
    temp1 = temp_raw / 256.0 + 25.0;
    ESP_LOGI(LSM6DS3_TAG, "temp1 = %f", temp1);
    *temp = temp_raw / 256.0 + 25.0;
}

/**
 * @brief read gyroscope from LSM6DS3
 */
void lsm6ds3_read_gyroscope(i2c_master_dev_handle_t dev_handle, float g[3])
{
    int16_t g_raw[3];
    g_raw[0] = 0;
    g_raw[1] = 0;
    g_raw[2] = 0;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_OUTX_L_G_REG_ADDR, data, 6));
    g_raw[0] = (data[1] << 8) | data[0];
    g_raw[1] = (data[3] << 8) | data[2];
    g_raw[2] = (data[5] << 8) | data[4];
    g[0] = g_raw[0] / 32768.0 * gyro_fs_cf;
    g[1] = g_raw[1] / 32768.0 * gyro_fs_cf;
    g[2] = g_raw[2] / 32768.0 * gyro_fs_cf;
}

/**
 * @brief read gyroscope from LSM6DS3
 */
void lsm6ds3_read_accelerometer(i2c_master_dev_handle_t dev_handle, float a[3])
{
    int16_t a_raw[3];
    a_raw[0] = 0;
    a_raw[1] = 0;
    a_raw[2] = 0;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_OUTX_L_XL_REG_ADDR, data, 6));
    a_raw[0] = (data[1] << 8) | data[0];
    a_raw[1] = (data[3] << 8) | data[2];
    a_raw[2] = (data[5] << 8) | data[4];
    a[0] = a_raw[0] / 32768.0 * accel_fs_cf;
    a[1] = a_raw[1] / 32768.0 * accel_fs_cf;
    a[2] = a_raw[2] / 32768.0 * accel_fs_cf;
}

/**
 * @brief read raw temperature, gyroscope, and accelerometer values from LSM6DS3
 */
void lsm6ds3_read_raw_data(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *data)
{
    /* read temperature if available*/
    if (lsm6ds3_check_temp_data_available(dev_handle))
    {
        lsm6ds3_read_temperature(dev_handle, &data->temp);
    }
    /* read gyroscope if new data available */
    if (lsm6ds3_check_gyro_data_available(dev_handle))
    {
        lsm6ds3_read_gyroscope(dev_handle, data->gyro);
    }

    /* read gyroscope if new data available */
    if (lsm6ds3_check_accel_data_available(dev_handle))
    {
        lsm6ds3_read_accelerometer(dev_handle, data->accel);
    }
}

/**
 * @brief LSM6DS3 enable block data update
 */
void lsm6ds3_enable_bdu(i2c_master_dev_handle_t dev_handle)
{
    /* enable block data update (BDU) */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_CTRL3_C_REG_ADDR, data, 1));
    read_modify_write(CTRL3_C_BDU, CTRL3_C_BDU, 1, data);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_CTRL3_C_REG_ADDR, data[0]));
}

/**
 * @brief set up the FIFO of the LSM6DS3
 */
void lsm6ds3_fifo_init(i2c_master_dev_handle_t dev_handle, uint16_t odr)
{
    /* set FIFO threshold */
    uint16_t fifo_threshold = 200;
    lsm6ds3_set_fifo_thresh(dev_handle, fifo_threshold);

    /* include temperature in FIFO */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    read_modify_write(FIFO_CTRL2_TEMP_EN, FIFO_CTRL2_TEMP_EN, 0x1, &data[1]);
    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, data[1]));

    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, data, 2));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL1_REG_ADDR = %X", data[0]);
    ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL2_REG_ADDR = %X", data[1]);

    /* set gyroscope decimation rate
    no decimation */
    data[0] = 0;
    read_modify_write(FIFO_CTRL3_DEC_GYRO2, FIFO_CTRL3_DEC_GYRO0, 0x1, &data[0]);

    /* set accelerometer decimation rate */
    read_modify_write(FIFO_CTRL3_DEC_ACCEL2, FIFO_CTRL3_DEC_ACCEL0, 0x1, &data[0]);

    ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL3_REG_ADDR, data[0]));

    /* disable 3rd FIFO data set and enable 4th FIFO data set (for temperature) */
    // data[0] = 0;
    // read_modify_write(FIFO_CTRL4_STOP_ON_FTH, FIFO_CTRL4_STOP_ON_FTH, 0x1, &data[0]);
    // read_modify_write(FIFO_CTRL4_DEC_DS4_FIFO2, FIFO_CTRL4_DEC_DS4_FIFO0, 0x1, &data[0]);
    // read_modify_write(FIFO_CTRL4_DEC_DS3_FIFO2, FIFO_CTRL4_DEC_DS3_FIFO0, 0x1, &data[0]);

    // ESP_ERROR_CHECK(lsm6ds3_register_write_byte(dev_handle, LSM6DS3_FIFO_CTRL4_REG_ADDR, data[0]));

    /* Set FIFO ODR */
    switch (odr)
    {
    case 12:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_12_5HZ);
        break;
    case 26:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_26HZ);
        break;
    case 52:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_52HZ);
        break;
    case 104:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_104HZ);
        break;
    case 208:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_208HZ);
        break;
    case 416:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_416HZ);
        break;
    case 833:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_ODR_833HZ);
        break;
    default:
        lsm6ds3_set_fifo_odr(dev_handle, LSM6DS3_DEFAULT_ODR);
        break;
    }
}

/**
 * @brief start recording data samples to the FIFO of the LSM6DS3
 */
void lsm6ds3_fifo_reset_start(i2c_master_dev_handle_t dev_handle)
{
    /* read FIFO CTRL */
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL1_REG_ADDR = %X", data[0]);
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL2_REG_ADDR = %X", data[1]);

    /* reset FIFO */
    lsm6ds3_fifo_reset(dev_handle);

    /* start FIFO in FIFO mode
    FIFO mode = 0b001 for FIFO mode*/
    lsm6ds3_set_fifo_operation_mode(dev_handle, LSM6DS3_FIFO_MODE_FIFO);
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL5_REG_ADDR, &data[0], 1));
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL5_REG_ADDR = %X", data[0]);
}

/**
 * @brief get number of samples in the LSM6DS3 FIFO buffer
 */
uint16_t lsm6ds3_fifo_get_num_samples(i2c_master_dev_handle_t dev_handle)
{
    uint16_t num_fifo_samples;
    /* get number of FIFO samples */
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS1_REG_ADDR, data, 1));
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS2_REG_ADDR, &data[1], 1));
    num_fifo_samples = ((data[1] & 0x0F) << 8 | data[0]);

    return num_fifo_samples;
}

/**
 * @brief read the LSM6DS3 FIFO pattern register
 */
uint16_t lsm6ds3_fifo_get_pattern(i2c_master_dev_handle_t dev_handle)
{
    uint8_t temp[2];
    uint16_t data;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS3_REG_ADDR, temp, 2));
    data = ((temp[1] & 0x3) << 8 | temp[0]);
    ESP_LOGI(LSM6DS3_TAG, "fifo_pattern = %X", data);
    return data;
}

/**
 * @brief read one word from the LSM6DS3 FIFO buffer
 */
uint16_t lsm6ds3_fifo_read_word_from_fifo(i2c_master_dev_handle_t dev_handle)
{
    uint8_t temp[2];
    uint16_t data;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, temp, 2));
    data = ((temp[1] << 8) | temp[0]);
    ESP_LOGI(LSM6DS3_TAG, "fifo_data = %X", data);
    return data;
}

/**
 * @brief LSM6DS3 read and interpret FIFO STATUS 2 register
 */
void lsm6ds3_read_fifo_status2_reg(i2c_master_dev_handle_t dev_handle)
{
    uint8_t temp;
    uint8_t waterm = 0;
    uint8_t overrun = 0;
    uint8_t full_smart = 0;
    uint8_t empty = 0;
    ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_STATUS2_REG_ADDR, &temp, 1));
    if (temp & FIFO_STATUS2_WATERM)
        waterm = 1;
    if (temp & FIFO_STATUS2_OVER_RUN)
        overrun = 1;
    if (temp & FIFO_STATUS2_FIFO_FULL_SMART)
        full_smart = 1;
    if (temp & FIFO_STATUS2_FIFO_EMPTY)
        empty = 1;
    ESP_LOGI(LSM6DS3_TAG, "FIFO watermark = %d, overrun = %d, full_smart = %d, empty = %d", waterm, overrun, full_smart, empty);
}

/**
 * @brief read data from the FIFO of the LSM6DS3 at a fixed frequency
 *
 * @param dev_handle LSM6DS3 i2c handle
 * @param lsm6ds3_fifo_buffer pointer to the memory buffer
 * @param num_timesteps number of timesteps to read
 *
 * @return number of timesteps read from the LSM6DS3 FIFO
 */
uint16_t lsm6ds3_fifo_read(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, uint16_t num_timesteps)
{
    uint16_t num_fifo_samples;
    uint16_t fifo_pattern;
    uint16_t num_timesteps_read = 0;

    /**
     * The accelerometer and gyroscope both log at 104 Hz.
     * The temperature sensor logs at 52 Hz.
     * Each group is 6 bytes
     *
     * 1st 6-byte group is gyroscope
     * 2nd group is accelerometer
     *
     * t = 0,
     */
    // /* read FIFO CTRL again */
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL1_REG_ADDR, &data[0], 1));
    // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_CTRL2_REG_ADDR, &data[1], 1));
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL1_REG_ADDR again = %X", data[0]);
    // ESP_LOGI(LSM6DS3_TAG, "FIFO_CTRL2_REG_ADDR again = %X", data[1]);

    // do
    // {
    // num_fifo_samples = lsm6ds3_fifo_get_num_samples(dev_handle);
    // } while (num_fifo_samples < 40);

    // // grab 40 samples from the FIFO and print out the FIFO pattern
    // while (num_fifo_samples > 0)
    // {
    //     /* get FIFO pattern */
    //     lsm6ds3_fifo_get_pattern(dev_handle);

    //     /* read 16 bits from FIFO data out H and L */
    //     uint16_t fifo_data = lsm6ds3_fifo_read_word_from_fifo(dev_handle);

    //     num_fifo_samples--;
    // }

    num_fifo_samples = lsm6ds3_fifo_get_num_samples(dev_handle);

    /* 6 axes */
    if (num_fifo_samples > num_timesteps * 6)
    {
        for (int i = 0; i < num_timesteps; i++)
        {
            // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            // lsm6ds3_fifo_buffer[i].gyro[0] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * gyro_fs_cf;
            // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            // lsm6ds3_fifo_buffer[i].gyro[1] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * gyro_fs_cf;
            // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            // lsm6ds3_fifo_buffer[i].gyro[2] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * gyro_fs_cf;

            // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            // lsm6ds3_fifo_buffer[i].accel[0] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * accel_fs_cf;
            // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            // lsm6ds3_fifo_buffer[i].accel[1] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * accel_fs_cf;
            // ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 2));
            // lsm6ds3_fifo_buffer[i].accel[2] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * accel_fs_cf;

            ESP_ERROR_CHECK(lsm6ds3_register_read(dev_handle, LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR, data, 12));
            lsm6ds3_fifo_buffer[i].gyro[0] = ((int16_t)(data[1] << 8) | data[0]) / 32768.0 * gyro_fs_cf;
            lsm6ds3_fifo_buffer[i].gyro[1] = ((int16_t)(data[3] << 8) | data[2]) / 32768.0 * gyro_fs_cf;
            lsm6ds3_fifo_buffer[i].gyro[2] = ((int16_t)(data[5] << 8) | data[4]) / 32768.0 * gyro_fs_cf;
            lsm6ds3_fifo_buffer[i].accel[0] = ((int16_t)(data[7] << 8) | data[6]) / 32768.0 * accel_fs_cf;
            lsm6ds3_fifo_buffer[i].accel[1] = ((int16_t)(data[9] << 8) | data[8]) / 32768.0 * accel_fs_cf;
            lsm6ds3_fifo_buffer[i].accel[2] = ((int16_t)(data[11] << 8) | data[10]) / 32768.0 * accel_fs_cf;

            num_timesteps_read++;

            // vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // ESP_LOGI(LSM6DS3_TAG, "temperature = %f\n", lsm6ds3_data.temp);
        ESP_LOGI(LSM6DS3_TAG, "");
        ESP_LOGI(LSM6DS3_TAG, "gyro = %f %f %f", lsm6ds3_fifo_buffer[num_timesteps - 1].gyro[0], lsm6ds3_fifo_buffer[num_timesteps - 1].gyro[1], lsm6ds3_fifo_buffer[num_timesteps - 1].gyro[2]);
        ESP_LOGI(LSM6DS3_TAG, "accel = %f %f %f", lsm6ds3_fifo_buffer[num_timesteps - 1].accel[0], lsm6ds3_fifo_buffer[num_timesteps - 1].accel[1], lsm6ds3_fifo_buffer[num_timesteps - 1].accel[2]);
    }

    num_fifo_samples = lsm6ds3_fifo_get_num_samples(dev_handle);
    ESP_LOGI(LSM6DS3_TAG, "num_fifo_samples = %d", num_fifo_samples);
    lsm6ds3_read_fifo_status2_reg(dev_handle);

    return num_timesteps_read;
}

/**
 * @brief read an 8-bit register, modify the bit field in the register, then write back to the register.
 *
 * @param bit_start starting bit of bit field in the register
 * @param bit_end ending bit of bit field in the register. bit_end must be equal or lower than bit_start.
 * @param field_value value to write to the register bit field
 * @param reg pointer to the register
 */
void read_modify_write(uint8_t bit_start,
                       uint8_t bit_end,
                       uint8_t field_value,
                       uint8_t *reg)
{
    uint8_t bit_len = bit_start - bit_end + 1;

    // /* create a mask of ones for the field of interest */
    // mask = ((1 << bit_len) - 1) << bit_end;
    // /* zero the field in the value using the mask*/
    // reg &= ~mask;
    // /* set the value of the field in the register */
    // reg |= (field_value & ((1 << bit_len) - 1)) << bit_end;

    /* one-liner */
    *reg = (*reg & ~(((1 << bit_len) - 1) << bit_end)) | ((field_value & ((1 << bit_len) - 1)) << bit_end);
}

uint8_t read_bit(uint8_t reg, uint8_t bit_pos)
{
    return reg & (1 << bit_pos);
}

void write_bit(uint8_t *reg, uint8_t bit_pos, uint8_t value)
{
    /* reset the bit first then set it if true */
    *reg = (*reg & ~(1 << bit_pos)) | (value << bit_pos);
}

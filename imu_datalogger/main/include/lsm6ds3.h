#ifndef LSM6DS3_H
#define LSM6DS3_H

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define I2C_MASTER_TIMEOUT_MS 1000

#define LSM6DS3_TAG "LSM6DS3"

#define LSM6DS3_SENSOR_ADDR 0x6B /*!< Address of the LSM6DS3 sensor */
#define LSM6DS3_WHO_AM_I_REG_ADDR 0x0F
#define LSM6DS3_WHO_AM_I 0x69

/* register addresses*/
/* Device Information & Embedded Functions */
#define LSM6DS3_FUNC_CFG_ACCESS_REG_ADDR 0x01
#define LSM6DS3_SENSOR_SYNC_TIME_FRAME_REG_ADDR 0x04
#define LSM6DS3_FIFO_CTRL1_REG_ADDR 0x06
#define LSM6DS3_FIFO_CTRL2_REG_ADDR 0x07
#define LSM6DS3_FIFO_CTRL3_REG_ADDR 0x08
#define LSM6DS3_FIFO_CTRL4_REG_ADDR 0x09
#define LSM6DS3_FIFO_CTRL5_REG_ADDR 0x0A
#define LSM6DS3_ORIENT_CFG_G_REG_ADDR 0x0B
#define LSM6DS3_INT1_CTRL_REG_ADDR 0x0D
#define LSM6DS3_INT2_CTRL_REG_ADDR 0x0E

/* Primary Control Registers */
#define LSM6DS3_CTRL1_XL_REG_ADDR 0x10
#define LSM6DS3_CTRL2_G_REG_ADDR 0x11
#define LSM6DS3_CTRL3_C_REG_ADDR 0x12
#define LSM6DS3_CTRL4_C_REG_ADDR 0x13
#define LSM6DS3_CTRL5_C_REG_ADDR 0x14
#define LSM6DS3_CTRL6_C_REG_ADDR 0x15
#define LSM6DS3_CTRL7_G_REG_ADDR 0x16
#define LSM6DS3_CTRL8_XL_REG_ADDR 0x17
#define LSM6DS3_CTRL9_XL_REG_ADDR 0x18
#define LSM6DS3_CTRL10_C_REG_ADDR 0x19

/* Interrupts & Status Registers */
#define LSM6DS3_WAKE_UP_SRC_REG_ADDR 0x1B
#define LSM6DS3_TAP_SRC_REG_ADDR 0x1C
#define LSM6DS3_D6D_SRC_REG_ADDR 0x1D
#define LSM6DS3_STATUS_REG_ADDR 0x1E

/* Sensor Data Output Registers */
#define LSM6DS3_OUT_TEMP_L_REG_ADDR 0x20
#define LSM6DS3_OUT_TEMP_H_REG_ADDR 0x21
#define LSM6DS3_OUTX_L_G_REG_ADDR 0x22
#define LSM6DS3_OUTX_H_G_REG_ADDR 0x23
#define LSM6DS3_OUTY_L_G_REG_ADDR 0x24
#define LSM6DS3_OUTY_H_G_REG_ADDR 0x25
#define LSM6DS3_OUTZ_L_G_REG_ADDR 0x26
#define LSM6DS3_OUTZ_H_G_REG_ADDR 0x27
#define LSM6DS3_OUTX_L_XL_REG_ADDR 0x28
#define LSM6DS3_OUTX_H_XL_REG_ADDR 0x29
#define LSM6DS3_OUTY_L_XL_REG_ADDR 0x2A
#define LSM6DS3_OUTY_H_XL_REG_ADDR 0x2B
#define LSM6DS3_OUTZ_L_XL_REG_ADDR 0x2C
#define LSM6DS3_OUTZ_H_XL_REG_ADDR 0x2D

/* FIFO Status & Data Output Registers */
#define LSM6DS3_FIFO_STATUS1_REG_ADDR 0x3A
#define LSM6DS3_FIFO_STATUS2_REG_ADDR 0x3B
#define LSM6DS3_FIFO_STATUS3_REG_ADDR 0x3C
#define LSM6DS3_FIFO_STATUS4_REG_ADDR 0x3D
#define LSM6DS3_FIFO_DATA_OUT_L_REG_ADDR 0x3E
#define LSM6DS3_FIFO_DATA_OUT_H_REG_ADDR 0x3F

/* Hardware Configuration & Embedded Functions Configuration */
#define LSM6DS3_TIMESTAMP0_REG_ADDR 0x40
#define LSM6DS3_TIMESTAMP1_REG_ADDR 0x41
#define LSM6DS3_TIMESTAMP2_REG_ADDR 0x42
#define LSM6DS3_STEP_TIMESTAMP_L_REG_ADDR 0x49
#define LSM6DS3_STEP_TIMESTAMP_H_REG_ADDR 0x4A
#define LSM6DS3_STEP_COUNTER_L_REG_ADDR 0x4B
#define LSM6DS3_STEP_COUNTER_H_REG_ADDR 0x4C

/* Advanced Interrupt Configuration Registers */
#define LSM6DS3_TAP_CFG_REG_ADDR 0x58
#define LSM6DS3_TAP_THS_6D_REG_ADDR 0x59
#define LSM6DS3_INT_DUR2_REG_ADDR 0x5A
#define LSM6DS3_WAKE_UP_THS_REG_ADDR 0x5B
#define LSM6DS3_WAKE_UP_DUR_REG_ADDR 0x5C
#define LSM6DS3_FREE_FALL_REG_ADDR 0x5D
#define LSM6DS3_MD1_CFG_REG_ADDR 0x5E
#define LSM6DS3_MD2_CFG_REG_ADDR 0x5F

/* LSM6DS3 reset bit */
#define LSM6DS3_CTRL3_C_SW_RESET 0x1

/**
 * CTRL1_XL bits
 */
#define CTRL1_XL_ODR_XL3 (7)
#define CTRL1_XL_ODR_XL2 (6)
#define CTRL1_XL_ODR_XL1 (5)
#define CTRL1_XL_ODR_XL0 (4)
#define CTRL1_XL_FS_XL1 (3)
#define CTRL1_XL_FS_XL0 (2)
#define CTRL1_XL_LPF1_BW_SEL (1)
#define CTRL1_XL_BW0_XL (0)

/* accelerometer power modes */
typedef enum
{
    LSM6DS3_XL_HM_MODE_HP = 0,
    LSM6DS3_XL_HM_MODE_LP = 1,
} lsm6ds3_xl_hm;

/* accelerometer full scale ranges */
typedef enum
{
    LSM6DS3_XL_FS_2G = 0x0,
    LSM6DS3_XL_FS_16G = 0x1,
    LSM6DS3_XL_FS_4G = 0x2,
    LSM6DS3_XL_FS_8G = 0x3,
} lsm6ds3_xl_fs;

/* accelerometer output data rates (with XL_HM_MODE = 0) */
typedef enum
{
    LSM6DS3_ODR_OFF = 0x0,
    LSM6DS3_ODR_12_5HZ = 0x1,
    LSM6DS3_ODR_26HZ = 0x2,
    LSM6DS3_ODR_52HZ = 0x3,
    LSM6DS3_ODR_104HZ = 0x4,
    LSM6DS3_ODR_208HZ = 0x5,
    LSM6DS3_ODR_416HZ = 0x6,
    LSM6DS3_ODR_833HZ = 0x7,
    LSM6DS3_ODR_1166HZ = 0x8,
    LSM6DS3_ODR_3333HZ = 0x9,
    LSM6DS3_ODR_6666HZ = 0xA,
} lsm6ds3_odr;

/**
 * CTRL2_G bits
 */
#define CTRL2_G_ODR_G3 (7)
#define CTRL2_G_ODR_G2 (6)
#define CTRL2_G_ODR_G1 (5)
#define CTRL2_G_ODR_G0 (4)
#define CTRL2_G_FS_G1 (3)
#define CTRL2_G_FS_G0 (2)
#define CTRL2_G_FS_125 (1)

#define CTRL6_C_XL_HM_MODE (4)
#define CTRL7_G_G_HM_MODE (7)

/* gyroscope power modes */
typedef enum
{
    LSM6DS3_G_HM_MODE_HP = 0,
    LSM6DS3_G_HM_MODE_LP = 1,
} lsm6ds3_g_hm;

/* gyroscope full scale ranges */
typedef enum
{
    LSM6DS3_G_FS_245DPS = 0x0,
    LSM6DS3_G_FS_500DPS = 0x1,
    LSM6DS3_G_FS_2000DPS = 0x3,
    LSM6DS3_G_FS_1000DPS = 0x2,
} lsm6ds3_g_fs;

// /* gyroscope output data rates (with G_HM_MODE = 0) */
// typedef enum
// {
//     LSM6DS3_G_ODR_OFF = 0x0,
//     LSM6DS3_G_ODR_12_5HZ = 0x1,
//     LSM6DS3_G_ODR_26HZ = 0x2,
//     LSM6DS3_G_ODR_52HZ = 0x3,
//     LSM6DS3_G_ODR_104HZ = 0x4,
//     LSM6DS3_G_ODR_208HZ = 0x5,
//     LSM6DS3_G_ODR_416HZ = 0x6,
//     LSM6DS3_G_ODR_833HZ = 0x7,
//     LSM6DS3_G_ODR_1666HZ = 0x8,
//     LSM6DS3_G_ODR_3333HZ = 0x9,
//     LSM6DS3_G_ODR_6666HZ = 0xA,
// } lsm6ds3_odr;

/**
 * CTRL3_C bits
 */
#define CTRL3_C_BOOT (7)
#define CTRL3_C_BDU (6)
#define CTRL3_C_H_LACTIVE (5)
#define CTRL3_C_PP_OD (4)
#define CTRL3_C_IF_INC (3)

/**
 * CTRL4_C bits
 */

/**
 * CTRL5_C bits
 */

/**
 * CTRL6_C bits
 */

/**
 * CTRL7_G bits
 */

/**
 * CTRL8_XL bits
 */

/**
 * CTRL9_XL bits
 */

/**
 * CTRL10_C bits
 */

/**
 * FIFO_CTRL1 bits
 */
#define FIFO_CTRL1_FTH_7 (7)
#define FIFO_CTRL1_FTH_0 (0)

/**
 * FIFO_CTRL2 bits
 */
#define FIFO_CTRL2_TEMP_EN (3)
#define FIFO_CTRL2_FTH_10 (2)
#define FIFO_CTRL2_FTH_8 (0)

/**
 * FIFO_CTRL3 bits
 */
#define FIFO_CTRL3_DEC_GYRO2 (5)
#define FIFO_CTRL3_DEC_GYRO0 (3)
#define FIFO_CTRL3_DEC_ACCEL2 (2)
#define FIFO_CTRL3_DEC_ACCEL0 (0)

/**
 * FIFO_CTRL4 bits
 */
#define FIFO_CTRL4_STOP_ON_FTH (7)
#define FIFO_CTRL4_DEC_DS4_FIFO2 (5)
#define FIFO_CTRL4_DEC_DS4_FIFO0 (3)
#define FIFO_CTRL4_DEC_DS3_FIFO2 (2)
#define FIFO_CTRL4_DEC_DS3_FIFO0 (0)

/**
 * FIFO_CTRL5 bits
 */
#define FIFO_CTRL5_ODR_3 (6)
#define FIFO_CTRL5_ODR_0 (3)
#define FIFO_CTRL5_MODE_2 (2)
#define FIFO_CTRL5_MODE_0 (0)

// typedef enum
// {
//     LSM6DS3_FIFO_ODR_OFF = 0x0,
//     LSM6DS3_FIFO_ODR_12_5HZ = 0x1,
//     LSM6DS3_FIFO_ODR_26HZ = 0x2,
//     LSM6DS3_FIFO_ODR_52HZ = 0x3,
//     LSM6DS3_FIFO_ODR_104HZ = 0x4,
//     LSM6DS3_FIFO_ODR_208HZ = 0x5,
//     LSM6DS3_FIFO_ODR_416HZ = 0x6,
//     LSM6DS3_FIFO_ODR_833HZ = 0x7,
// } lsm6ds3_fifo_odr;

typedef enum
{
    LSM6DS3_FIFO_MODE_BYPASS = 0x0,
    LSM6DS3_FIFO_MODE_FIFO = 0x1,
    LSM6DS3_FIFO_CONTINUOUS_THEN_FIFO = 0x3,
    LSM6DS3_FIFO_BYPASS_CONTINUOUS = 0x4,
    LSM6DS3_FIFO_MODE_CONTINUOUS = 0x6,
} lsm6ds3_fifo_mode;

/**
 * FIFO_STATUS1 bits
 */
#define FIFO_STATUS1_DIFF_FIFO_7 (7)
#define FIFO_STATUS1_DIFF_FIFO_0 (0)

/**
 * FIFO_STATUS2 bits
 */
#define FIFO_STATUS2_WATERM (7)
#define FIFO_STATUS2_OVER_RUN (6)
#define FIFO_STATUS2_FIFO_FULL_SMART (5)
#define FIFO_STATUS2_FIFO_EMPTY (4)
#define FIFO_STATUS2_DIFF_FIFO_10 (2)
#define FIFO_STATUS2_DIFF_FIFO_8 (0)

/**
 * FIFO_STATUS3 bits
 */
#define FIFO_STATUS3_FIFO_PATTERN_7 (7)
#define FIFO_STATUS3_FIFO_PATTERN_0 (0)

/**
 * FIFO_STATUS4 bits
 */
#define FIFO_STATUS4_FIFO_PATTERN_9 (1)
#define FIFO_STATUS4_FIFO_PATTERN_8 (0)

/**
 * STATUS_REG bits
 */
#define STATUS_REG_TDA (2)
#define STATUS_REG_GDA (1)
#define STATUS_REG_XLDA (0)

/**
 * default full scale values and output data rates
 */
#define LSM6DS3_DEFAULT_XL_FS LSM6DS3_XL_FS_4G
#define LSM6DS3_DEFAULT_G_FS LSM6DS3_G_FS_500DPS
#define LSM6DS3_DEFAULT_ODR LSM6DS3_ODR_52HZ

typedef struct
{
    float temp;
    float gyro[3];
    float accel[3];

} lsm6ds3_data_t;
esp_err_t lsm6ds3_register_read(i2c_master_dev_handle_t dev_handle,
                                uint8_t reg_addr,
                                uint8_t *data,
                                size_t len);
esp_err_t lsm6ds3_register_write_byte(i2c_master_dev_handle_t dev_handle,
                                      uint8_t reg_addr,
                                      uint8_t data);

void lsm6ds3_init_accel(i2c_master_dev_handle_t dev_handle, uint16_t odr, uint16_t xl_fs);
void lsm6ds3_init_gyro(i2c_master_dev_handle_t dev_handle, uint16_t odr, uint16_t g_fs);
void lsm6ds3_init_all(i2c_master_dev_handle_t dev_handle, uint8_t odr, uint16_t xl_fs, uint16_t g_fs);

void lsm6ds3_reset(i2c_master_dev_handle_t dev_handle);

void lsm6ds3_set_accel_power(i2c_master_dev_handle_t dev_handle, lsm6ds3_xl_hm xl_hm);
void lsm6ds3_set_accel_fs(i2c_master_dev_handle_t dev_handle, lsm6ds3_xl_fs xl_fs);
void lsm6ds3_set_accel_odr(i2c_master_dev_handle_t dev_handle, lsm6ds3_odr xl_odr);

void lsm6ds3_set_gyro_power(i2c_master_dev_handle_t dev_handle, lsm6ds3_g_hm g_hm);
void lsm6ds3_set_gyro_fs(i2c_master_dev_handle_t dev_handle, lsm6ds3_g_fs g_fs);
void lsm6ds3_set_gyro_odr(i2c_master_dev_handle_t dev_handle, lsm6ds3_odr g_odr);

void lsm6ds3_enable_bdu(i2c_master_dev_handle_t dev_handle);

void lsm6ds3_set_fifo_thresh(i2c_master_dev_handle_t dev_handle, uint16_t fifo_thresh);
void lsm6ds3_set_fifo_odr(i2c_master_dev_handle_t dev_handle, lsm6ds3_odr fifo_odr);
void lsm6ds3_set_fifo_operation_mode(i2c_master_dev_handle_t dev_handle, lsm6ds3_fifo_mode fifo_mode);

uint16_t lsm6ds3_fifo_get_num_samples(i2c_master_dev_handle_t dev_handle);
uint16_t lsm6ds3_fifo_get_pattern(i2c_master_dev_handle_t dev_handle);
uint16_t lsm6ds3_fifo_read_word_from_fifo(i2c_master_dev_handle_t dev_handle);
void lsm6ds3_read_fifo_status2_reg(i2c_master_dev_handle_t dev_handle);

uint8_t lsm6ds3_check_temp_data_available(i2c_master_dev_handle_t dev_handle);
uint8_t lsm6ds3_check_gyro_data_available(i2c_master_dev_handle_t dev_handle);
uint8_t lsm6ds3_check_accel_data_available(i2c_master_dev_handle_t dev_handle);

void lsm6ds3_read_temperature(i2c_master_dev_handle_t dev_handle, float *temp);
void lsm6ds3_read_gyroscope(i2c_master_dev_handle_t dev_handle, float g[3]);
void lsm6ds3_read_accelerometer(i2c_master_dev_handle_t dev_handle, float a[3]);
void lsm6ds3_read_raw_data(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *data);

void read_modify_write(uint8_t bit_start,
                       uint8_t bit_end,
                       uint8_t field_value,
                       uint8_t *reg);
uint8_t read_bit(uint8_t reg, uint8_t bit_pos);
void write_bit(uint8_t *reg, uint8_t bit_pos, uint8_t value);

void lsm6ds3_fifo_init(i2c_master_dev_handle_t dev_handle, uint16_t odr);
void lsm6ds3_fifo_stop(i2c_master_dev_handle_t dev_handle);
void lsm6ds3_fifo_reset(i2c_master_dev_handle_t dev_handle);
void lsm6ds3_fifo_reset_start(i2c_master_dev_handle_t dev_handle);
uint16_t lsm6ds3_fifo_read(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, uint16_t num_samples);

#endif
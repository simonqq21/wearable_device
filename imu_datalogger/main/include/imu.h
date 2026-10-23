#ifndef IMU_H
#define IMU_H

#include "../include/nvs.h"
#include "../include/uart.h"
#include "common.h"
#include "math.h"
#include "i2c_module.h"
#include "lsm6ds3.h"
#include "nvs.h"

#define IMU_TAG "IMU"

/* IMU data struct */
typedef struct __attribute__((packed))
{
    int64_t timestamp;
    float ax, ay, az; // IMU model-agnostic
    float gx, gy, gz;
    float temp;
} imu_raw_data_t;

/* Euler angle */
typedef struct __attribute__((packed))
{
    int64_t timestamp;
    float x, y, z;
} euler_angle_t;

/* Quaternion */
typedef struct __attribute__((packed))
{
    int64_t timestamp;
    float w, x, y, z;
} quaternion_t;

/* orientation data union */
typedef union __attribute__((packed))
{
    euler_angle_t euler_angle;
    quaternion_t quaternion;
} imu_orientation_data_t;

/**
 * @brief unified raw and processed IMU data struct
 *
 * This struct is used for sending data from the sensor fusion task
 * to the other downstream tasks.
 */
typedef struct __attribute__((packed))
{
    imu_raw_data_t imu_raw_data;
    imu_orientation_data_t imu_orientation_data;
} imu_unified_data_t;

/* IMU logging timedelta in ms */
#define IMU_LOGGING_TIMEDELTA_MS ceil(1000 / IMU_ODR_HZ + 5)

// NVS offset read write function callback typedef
typedef esp_err_t (*nvs_func_t)(imu_calibration_offsets_t *);

/**
 * @brief set the callback function to read the IMU calibration constants
 * from the NVS.
 */
void imu_set_offset_read_cb(nvs_func_t cb);
/**
 * @brief set the callback function to write the IMU calibration constants
 * to the NVS.
 */
void imu_set_offset_write_cb(nvs_func_t cb);
/**
 * @brief calibrate the IMU
 *
 * @param imu_dev_handle i2c handle of the IMU
 */
void imu_calibrate(i2c_master_dev_handle_t imu_dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer);
/**
 * @brief load IMU calibration offsets from the NVS to memory
 */
void imu_load_calibration_offsets(void);
/**
 * @brief initialize and configure IMU
 *
 * @param imu_dev_handle i2c handle of the IMU
 * @param calibrate_imu calibrate the IMU if set
 */
void imu_init(i2c_master_dev_handle_t imu_dev_handle, uint16_t sample_rate, uint16_t xl_fs, uint16_t g_fs, uint8_t calibrate_imu);
/**
 * @brief read a number of samples from the FIFO and apply offsets to all measurements.
 *
 * @param imu_dev_handle i2c handle of the IMU
 * @param imu_data_buffer pointer to IMU data buffer
 * @param num_samples number of samples to read from FIFO
 */
uint16_t imu_read_FIFO_calibrated(i2c_master_dev_handle_t imu_dev_handle,
                                  lsm6ds3_data_t *lsm6ds3_fifo_buffer,
                                  imu_raw_data_t *imu_data_buffer,
                                  uint16_t num_samples);
/**
 * @brief read raw measurements from IMU
 *
 * @param imu_dev_handle i2c handle of the IMU
 * @param data pointer to destination struct
 */
int imu_measure_raw(i2c_master_dev_handle_t imu_dev_handle, imu_raw_data_t *data);
/**
 * @brief convert LSM6DS3 data to IMU agnostic orientation data
 *
 * @param lsm6ds3_data pointer to lsm6ds3 data struct
 * @param imu__data pointer to IMU data struct
 */
void __imu_convert_vals(lsm6ds3_data_t *lsm6ds3_data, imu_raw_data_t *imu_data);
/**
 * @brief apply saved calibration offsets to raw IMU data
 *
 * @param imu__data pointer to IMU data struct
 */
void imu_apply_calibration(imu_raw_data_t *data);
/**
 * @brief read calibration-offset measurements from IMU
 *
 * @param imu_dev_handle i2c handle of the IMU
 * @param imu__data pointer to IMU data struct
 */
esp_err_t imu_measure_calibrated(i2c_master_dev_handle_t imu_dev_handle, imu_raw_data_t *data);

#endif
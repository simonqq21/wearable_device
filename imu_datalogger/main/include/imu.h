#ifndef IMU_H
#define IMU_H

#include "../include/nvs.h"
#include "../include/uart.h"
#include "common.h"
#include "math.h"
#include "i2c_module.h"
#include "lsm6ds3.h"

#define IMU_TAG "IMU"

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
uint16_t imu_read_FIFO_calibrated(i2c_master_dev_handle_t imu_dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, imu_data_t *imu_data_buffer, uint16_t num_samples);
/**
 * @brief read raw measurements from IMU
 *
 * @param imu_dev_handle i2c handle of the IMU
 * @param data pointer to destination struct
 */
int imu_measure_raw(i2c_master_dev_handle_t imu_dev_handle, imu_data_t *data);
/**
 * @brief convert LSM6DS3 data to IMU agnostic orientation data
 *
 * @param lsm6ds3_data pointer to lsm6ds3 data struct
 * @param imu__data pointer to IMU data struct
 */
void __imu_convert_vals(lsm6ds3_data_t *lsm6ds3_data, imu_data_t *imu_data);
/**
 * @brief apply saved calibration offsets to raw IMU data
 *
 * @param imu__data pointer to IMU data struct
 */
void imu_apply_calibration(imu_data_t *data);
/**
 * @brief read calibration-offset measurements from IMU
 *
 * @param imu_dev_handle i2c handle of the IMU
 * @param imu__data pointer to IMU data struct
 */
esp_err_t imu_measure_calibrated(i2c_master_dev_handle_t imu_dev_handle, imu_data_t *data);

#endif
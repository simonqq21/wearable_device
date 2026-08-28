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
#define IMU_LOGGING_TIMEDELTA_MS (1000 / IMU_ODR_HZ)

/**
 * @brief parameters for IMU task
 *
 * @param streambuffer_imu stream buffer going from IMU task to main datalogging task
 * @param task_handle_status_led task handle of status LED controlling task
 * @param dev_handle IMU i2c device handle
 */
typedef struct
{
    TaskHandle_t *task_handle_status_led;
    QueueHandle_t queue_imu;
    StreamBufferHandle_t streambuffer_imu;
    i2c_master_dev_handle_t dev_handle;
} params_task_imu_t;

// NVS offset read write function callback typedef
typedef esp_err_t (*nvs_func_t)(imu_calibration_offsets_t *);

void task_imu(void *params);

void imu_set_offset_read_cb(nvs_func_t cb);
void imu_set_offset_write_cb(nvs_func_t cb);

void imu_calibrate(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer);

void imu_load_calibration_offsets(void);

void imu_init(i2c_master_dev_handle_t dev_handle, uint16_t sample_rate, uint16_t xl_fs, uint16_t g_fs, uint8_t calibrate_imu);

uint16_t imu_read_FIFO_calibrated(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, imu_data_t *imu_data_buffer, uint16_t num_samples);

esp_err_t imu_measure_raw(i2c_master_dev_handle_t dev_handle, imu_data_t *data);
void __imu_convert_vals(lsm6ds3_data_t *lsm6ds3_data, imu_data_t *imu_data);
void imu_apply_calibration(imu_data_t *data);

// void mpu6050_preprocess(imu_data_t *data);
// void task_imu(void *pvParameters);

#endif
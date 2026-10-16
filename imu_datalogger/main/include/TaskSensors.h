#ifndef TASK_SENSORS_H
#define TASK_SENSORS_H

#include "imu.h"
#include "heart_rate.h"

/**
 * @brief parameters for IMU task
 *
 * @param streambuffer_imu stream buffer going from IMU task to main datalogging task
 * @param task_handle_GPIO task handle of status LED controlling task
 * @param imu_dev_handle IMU i2c device handle
 */
typedef struct
{
    QueueHandle_t queue_imu;
    // StreamBufferHandle_t streambuffer_imu;
    i2c_master_dev_handle_t imu_dev_handle;
    QueueHandle_t queue_heart_rate;
    // StreamBufferHandle_t streambuffer_imu;
    i2c_master_dev_handle_t heart_rate_dev_handle;
} params_task_sensors_t;

/**
 * @brief i2c sensors reading task
 *
 * @param params type params_task_sensors_t
 */
void task_sensors(void *params);

#endif

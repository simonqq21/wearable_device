#ifndef TASK_SENSORS_H
#define TASK_SENSORS_H

#include "common.h"
#include "imu.h"
#include "heart_rate.h"

/**
 * @brief parameters for IMU task
 *
 * @param queue_imu_sensor outgoing queue for IMU sensor data
 * @param queue_heart_rate_sensor outgoing queue for heart rate sensor data
 * @param imu_dev_handle IMU i2c device handle
 * @param heart_rate_dev_handle heart rate i2c device handle
 * @param sensors_cmds struct containing sensor command
 */
typedef struct
{
    QueueHandle_t queue_imu_sensor;
    QueueHandle_t queue_heart_rate_sensor;
    i2c_master_dev_handle_t imu_dev_handle;
    i2c_master_dev_handle_t heart_rate_dev_handle;
    sensors_cmd_struct_t *sensors_cmds;
} params_task_sensors_t;

/**
 * @brief i2c sensors reading task
 *
 * @param params type params_task_sensors_t
 */
void task_sensors(void *params);

#endif

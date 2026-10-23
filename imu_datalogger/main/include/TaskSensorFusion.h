#ifndef TASK_SENSOR_FUSION_H
#define TASK_SENSOR_FUSION_H

#include "datalogging.h"

/**
 * @brief datalogging task params
 *
 * @param queue_imu_sensor incoming queue for IMU sensor data
 * @param queue_heart_rate_sensor incoming queue for heart rate sensor data
 * @param queue_imu_sdcard outgoing queue for IMU sensor data to the sdcard task
 * @param queue_heart_rate_sdcard outgoing queue for IMU sensor data to the sdcard task
 * @param queue_imu_UART outgoing queue for IMU sensor data to the UART task
 * @param queue_heart_rate_UART outgoing queue for IMU sensor data to the UART task
 * @param queue_imu_BLE outgoing queue for IMU sensor data to the BLE task
 * @param queue_heart_rate_BLE outgoing queue for IMU sensor data to the BLE task

 */
typedef struct
{
    QueueHandle_t queue_imu_sensor;
    QueueHandle_t queue_heart_rate_sensor;

    QueueHandle_t queue_imu_sdcard;
    QueueHandle_t queue_heart_rate_sdcard;
    QueueHandle_t queue_imu_UART;
    QueueHandle_t queue_heart_rate_UART;
    QueueHandle_t queue_imu_BLE;
    QueueHandle_t queue_heart_rate_BLE;
} params_task_sensor_fusion_t;

/**
 * @brief
 */
void task_sensor_fusion(void *params);

#endif
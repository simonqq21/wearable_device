#ifndef TASK_BLE_H
#define TASK_BLE_H

#include "ble.h"
#include "haptics_lib.h"
#include "imu.h"

/**
 * @brief parameters for BLE FreeRTOS task
 *
 * @param queue_imu_BLE incoming queue of unified IMU values
 * @param queue_heart_rate_BLE incoming queue of unified heart rate values
 * @param queue_haptics_BLE queue to send haptics play
 *      built-in sequences commands to the haptics module
 */
typedef struct
{
    QueueHandle_t queue_imu_BLE;
    QueueHandle_t queue_heart_rate_BLE;

    QueueHandle_t queue_haptics_BLE;

    /* pointer to battery voltage */
    float *battery_voltage;

} params_task_ble_t;

void task_ble_streaming(void *params);

#endif
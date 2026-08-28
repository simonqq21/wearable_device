#ifndef DATALOGGING_H
#define DATALOGGING_H

#include "common.h"

/**
 * @brief datalogging task params
 *
 * @param queue_imu
 * @param streambuffer_imu
 *
 */
typedef struct
{
    QueueHandle_t queue_imu;

    StreamBufferHandle_t streambuffer_imu;

    QueueHandle_t queue_raw_sdcard;
    QueueHandle_t queue_orientation_sdcard;
    QueueHandle_t queue_orientation_UART;
    QueueHandle_t queue_orientation_BLE;
    StreamBufferHandle_t streambuffer_sd;

} params_task_main_datalogging_t;

void task_main_datalogging(void *params);

#endif
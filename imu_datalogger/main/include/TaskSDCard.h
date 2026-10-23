#ifndef TASK_SD_CARD_H
#define TASK_SD_CARD_H

#include "sd_card.h"

/**
 * @brief parameters for the SD card datalogging task
 *
 * @param queue_imu_sdcard incoming queue of unified IMU values
 * @param queue_heart_rate_sdcard incoming queue of unified heart rate values
 */
typedef struct
{
    QueueHandle_t queue_imu_sdcard;
    QueueHandle_t queue_heart_rate_sdcard;
} params_task_SD_card_datalogger_t;

void task_SD_card_datalogger(void *params);

#endif
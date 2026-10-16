#ifndef HEART_RATE_H
#define HEART_RATE_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdatomic.h>
#include "esp_log.h"
// #include "esp_board_manager_includes.h"
#include "esp_max30102.h"
#include "esp_health_hr.h"
#include "hr_ppg_msg.h"
#include "hr_example_win.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define HEART_RATE_TAG HEART_RATE

typedef struct
{
    uint32_t samples[SAMPLE_RATE];
    int kind; /* HR_STREAM_* */
    int n;    /* real samples in this item; 0 if kind is not SAMPLES */
} hr_ppg_msg_t;

/**
 * @brief
 */
void heart_rate_init(i2c_master_bus_handle_t bus_handle,
                     i2c_master_dev_handle_t heart_rate_dev_handle);

#endif
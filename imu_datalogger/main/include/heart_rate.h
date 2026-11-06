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

#define HEART_RATE_TAG "HEART_RATE"
#define PPG_SAMPLE_RATE 100

/**
 * @brief PPG heart rate raw data struct
 *
 * @param timestamp timestamp
 * @param red_val measured PPG red value
 * @param ir_val measured PPG IR value
 *
 */
typedef struct
{
    int64_t timestamp;
    uint32_t red_val;
    uint32_t ir_val;
    // uint32_t samples[PPG_SAMPLE_RATE];
    // int kind; /* HR_STREAM_* */
    // int n;    /* real samples in this item; 0 if kind is not SAMPLES */
} heart_rate_raw_data_t;

/**
 * @brief heart rate and spo2 data struct
 *
 * @param timestamp
 * @param heart_rate
 * @param spo2
 */
typedef struct
{
    int64_t timestamp;
    uint8_t heart_rate;
    float spo2;
} heart_rate_hr_spo2_data_t;

/**
 * @brief unified PPG heart rate data
 *
 * @param heart_rate_raw_data
 * @param heart_rate_hr_spo2_data
 */
typedef struct
{
    heart_rate_raw_data_t heart_rate_raw_data;
    heart_rate_hr_spo2_data_t heart_rate_hr_spo2_data;
} heart_rate_unified_data_t;

/**
 * @brief
 */
void heart_rate_init(i2c_master_bus_handle_t bus_handle,
                     i2c_master_dev_handle_t heart_rate_dev_handle);

#endif
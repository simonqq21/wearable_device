#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <math.h>
#include <dirent.h>
#include <inttypes.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdatomic.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include "freertos/semphr.h"

#include <esp_err.h>
#include <esp_log.h>
#include "esp_timer.h"
#include "esp_system.h"

#include "iot_button.h"
#include "button_gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "config.h"
#include "pins.h"

/* IMU task notification queue values */
typedef enum
{
    CMD_IMU_STOP,
    CMD_IMU_CALIBRATE,
    CMD_IMU_START,
} cmd_imu_t;

/* heart rate command values */
typedef enum
{
    CMD_HEART_RATE_STOP = 0,
    CMD_HEART_RATE_START,
} cmd_heart_rate_t;

/* Datalogging task notification queue values */
typedef enum
{
    CMD_DATALOGGING_STOP,
    CMD_DATALOGGING_GO,
} cmd_task_datalogging_t;

/* SD card task notification command values */
typedef enum
{
    CMD_SD_CARD_STOP,
    CMD_SD_CARD_START,
} cmd_task_sd_card_datalogging_t;

/* Status LED task notification queue values */
typedef enum
{
    STATUS_LED_OFF,
    STATUS_LED_ON,
    STATUS_LED_BLINK,
    STATUS_LED_FAST_BLINK,
} cmd_task_status_led_t;

/**
 * @brief struct for individual sensor toggles
 *
 * @param imu_cmd imu is enabled and collecting data
 * @param heart_rate_cmd heart rate sensor is enabled and collecting data
 *
 */
typedef struct
{
    atomic_int imu_cmd;
    atomic_int heart_rate_cmd;
} sensors_cmd_struct_t;

#endif
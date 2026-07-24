#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <dirent.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include "freertos/semphr.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include <esp_err.h>
#include <esp_log.h>
#include "esp_system.h"

#include "iot_button.h"
#include "button_gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include <mpu6050.h>

// #include "button_module.h"
// #include "imu.h"
// #include "led_module.h"
// #include "nvs.h"
// #include "sd_card.h"
// #include "uart.h"

/* IMU task notification queue values */
typedef enum
{
    IMU_STOP,
    IMU_CALIBRATE,
    IMU_READ_LOOP,
} cmd_imu_task_t;

/* IMU calibration offsets struct */
typedef struct
{
    float oAx, oAy, oAz;
    float oGx, oGy, oGz;
} imu_calibration_offsets_t;

/* IMU data struct */
typedef struct
{
    uint32_t timestamp;
    float ax, ay, az; // IMU model-agnostic
    float gx, gy, gz;
    float temp;
} imu_data_t;

/* Datalogging task notification queue values */
typedef enum
{
    DATALOGGING_STOP,
    DATALOGGING_GO,
} cmd_task_datalogging_t;

/* SD card task notification command values */
typedef enum
{
    SD_CARD_STOP,
    SD_CARD_START,
} cmd_task_sd_card_datalogging_t;

/* Status LED task notification queue values */
typedef enum
{
    STATUS_LED_OFF,
    STATUS_LED_ON,
    STATUS_LED_BLINK,
    STATUS_LED_FAST_BLINK,
} cmd_task_status_led_t;

// ESP32 SD card config
#define MOUNT_POINT "/sdcard"

#endif
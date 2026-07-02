#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "freertos/semphr.h"

#include <esp_err.h>
#include <esp_log.h>
#include "esp_system.h"

#include "iot_button.h"
#include "button_gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include <mpu6050.h>

/* Status LED task notification queue values */
typedef enum
{
    STATUS_LED_OFF,
    STATUS_LED_ON,
    STATUS_LED_BLINK,
    STATUS_LED_FADE,
} status_led_task_notif_t;

/* IMU task notification queue values */
typedef enum
{
    IMU_CALIBRATE,
    IMU_READ_LOOP,
} imu_task_notif_t;

/* Datalogging task notification queue values */
typedef enum
{
    DATALOGGING_STOP,
    DATALOGGING_START, 
} datalogging_task_notif_t;

/* IMU calibration offsets struct */
typedef struct
{
    float oAx;
    float oAy;
    float oAz;
    float oGx;
    float oGy;
    float oGz;
} imu_calibration_offsets_t;

/* IMU data struct */
typedef struct
{
    mpu6050_acceleration_t acc;
    mpu6050_rotation_t rot;
    float temp;
} imu_data_t;

// ESP32 pins config
#define LED1_PIN 2 // power/status LED
#define LED2_PIN 4 // recording LED
#define BTN1_PIN 5
#define SDA_PIN 21
#define SCL_PIN 22

// ESP32 NVS config
#define IMU_OFFSETS_NVS_KEY "imu_offsets"
#define NVS_NAMESPACE "NVS"

// ESP32 UART config
#define UART_PORT_NUM 0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 128

// ESP32 IMU MPU6050 config
// #ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
// #else
// #define ADDR MPU6050_I2C_ADDRESS_HIGH
// #endif

// FreeRTOS primitives config
#define IMU_QUEUE_LEN 30
#define IMU_SAMPLE_RATE_HZ 10

#endif
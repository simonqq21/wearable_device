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

typedef enum
{
    STATUS_LED_OFF,
    STATUS_LED_ON,
    STATUS_LED_BLINK,
    STATUS_LED_FADE,
} status_led_state_t;

typedef struct
{
    float oAx;
    float oAy;
    float oAz;
    float oGx;
    float oGy;
    float oGz;
} imu_calibration_offsets_t;

#endif
#ifndef BLE_H
#define BLE_H

#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

const char device_name[] = "ITLAB BLE IMU";

#define PROFILE_NUM 1
#define APP_ID 0
// 32816f9c-482f-43fb-9e58-5f35d8d5a8e0
const uint8_t ORIENTATION_SERVICE_UUID[] =
    {
        0x32,
        0x81,
        0x6f,
        0x9c,
        0x48,
        0x2f,
        0x43,
        0xfb,
        0x9e,
        0x58,
        0x5f,
        0x35,
        0xd8,
        0xd5,
        0xa8,
        0xe0,
};

// 653f0660-4e9d-46e5-9881-14fe894e2edd
const uint8_t ORIENTATION_CHARACTERISTIC_UUID[] = {
    0x65,
    0x3f,
    0x06,
    0x60,
    0x4e,
    0x9d,
    0x46,
    0xe5,
    0x98,
    0x81,
    0x14,
    0xfe,
    0x89,
    0x4e,
    0x2e,
    0xdd,
};

#define ORIENTATION_NUM_HANDLE 6

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/**
 * @brief parameters for BLE FreeRTOS task
 *
 * @param queue_orientation_BLE IMU orientation queue sent to BLE
 */
typedef struct
{
    QueueHandle_t queue_orientation_BLE;
} params_task_ble_t;

void ble_configure(void);
void task_ble_streaming(void *params);

#endif
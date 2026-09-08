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

#define BLE_TAG "BLE"

/* BLE app profile */
#define PROFILE_NUM 1
#define APP_ID 0
#define SVC_INST_ID 0

#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

/* Attributes State Machine */
enum
{
    ORIENTATION_IDX_SVC, // orientation service index

    ORIENTATION_IDX_CHAR,    // orientation characteristic index
    ORIENTATION_IDX_VAL,     // orientation value index
    ORIENTATION_IDX_NTF_CFG, // POT1 notification configuration index

    // LED1_IDX_CHAR, // LED 1
    // LED1_IDX_VAL,  // LED 1 v

    // LED2_IDX_CHAR, // LED 2 characteristic index
    // LED2_IDX_VAL,  // LED 2 value index

    // BTN1_IDX_CHAR,	  // BTN1 characteristic index
    // BTN1_IDX_VAL,	  // BTN1 value index
    // BTN1_IDX_NTF_CFG, // BTN1 notification configuration index

    // POT1_IDX_CHAR,	  // POT1 characteristic index
    // POT1_IDX_VAL,	  // POT1 value index
    // POT1_IDX_NTF_CFG, // POT1 notification configuration index

    GATT_IDX_NB, // number of elements
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

/**
 * @brief this struct acts as an application-level state container for a single GATT profile.
 * It stores the stack handles, identifiers, and configuration metadata needed to manage a GATT
 * service, its characteristic, and its descriptor throughout their lifecycle.
 */
struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;       // Profile-specific event callback
    uint16_t gatts_if;             // GATT Interface ID
    uint16_t app_id;               // Application ID
    uint16_t conn_id;              // Connection ID
    uint16_t service_handle;       // Service handle
    esp_gatt_srvc_id_t service_id; // Service ID

    uint16_t char_handle;    // Characteristic handle
    esp_bt_uuid_t char_uuid; // Characteristic UUID

    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property; // Characteristic property (Read, Write, Notify, Indicate)
    uint16_t descr_handle;         // Characteristic descriptor handle
    esp_bt_uuid_t descr_uuid;      // Characteristic descriptor UUID
};

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
void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param);

#endif
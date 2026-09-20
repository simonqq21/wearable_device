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

#include "haptics_lib.h"

#define BLE_TAG "BLE"

/* BLE app profile */
#define PROFILE_NUM 1
#define APP_ID 0
#define SVC_INST_ID 0

#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

// #define HEART_RATE_SERVICE_UUID 0x180D
// #define HEART_RATE_CHARACTERISTIC_UUID 0x2A37
// #define BODY_SENSOR_LOCATION_CHARACTERISTIC_UUID 0x2A38

/* Attributes State Machine */
enum
{
    HEART_RATE_IDX_SVC, // heart rate service

    HEART_RATE_IDX_CHAR, // Heart Rate Measurement characteristic index
    HEART_RATE_IDX_VAL,
    HEART_RATE_IDX_NTF_CFG,

    BODY_SENSOR_LOCATION_IDX_CHAR, // Body Sensor Location characteristic index
    BODY_SENSOR_LOCATION_IDX_VAL,

    GATT_HEART_RATE_IDX_NB, // number of elements
};

enum
{
    ORIENTATION_IDX_SVC, // orientation service index

    ORIENTATION_IDX_CHAR,    // orientation characteristic index
    ORIENTATION_IDX_VAL,     // orientation value index
    ORIENTATION_IDX_NTF_CFG, // orientation notification configuration index
    /* notification configuration is only used for notify and
    indicate transactions */

    GATT_ORIENTATION_IDX_NB, // number of elements
};

enum
{
    HAPTICS_IDX_SVC, // haptic service index

    /* haptics command to play built-in sequence characteristic index */
    HAPTICS_COMMAND_PLAY_BUILTIN_SEQUENCE_IDX_CHAR,
    /* haptics command to play built-in sequence value index */
    HAPTICS_COMMAND_PLAY_BUILTIN_SEQUENCE_IDX_VAL, //

    /* haptics command to play custom sequence characteristic index */
    HAPTICS_COMMAND_PLAY_CUSTOM_SEQUENCE_IDX_CHAR,
    /* haptics command to play custom sequence value index */
    HAPTICS_COMMAND_PLAY_CUSTOM_SEQUENCE_IDX_VAL,

    // /* haptics configuration characteristic index */
    // HAPTICS_CONFIG_IDX_CHAR,
    // /* haptics configuration value index */
    // HAPTICS_CONFIG_IDX_VAL,

    /* haptics status characteristic index */
    HAPTICS_STATUS_IDX_CHAR,
    /* haptics status value index */
    HAPTICS_STATUS_IDX_VAL,
    /* haptics status notification configuration index */
    HAPTICS_STATUS_IDX_NTF_CFG,

    GATT_HAPTICS_IDX_NB, // number of elements
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
 * @param queue_orientation_BLE IMU orientation queue sent via BLE
 * @param queue_heart_rate_BLE heart rate queue sent via BLE
 * @param queue_haptics_command_play_builtin_BLE queue to send haptics play
 *      built-in sequences commands to the haptics module
 * @param queue_haptics_command_play_custom_BLE queue to send haptics play
 *      custom sequences commands to the haptics module
 *
 */
typedef struct
{
    QueueHandle_t queue_orientation_BLE;
    QueueHandle_t queue_heart_rate_BLE;
    QueueHandle_t queue_haptics_command_play_builtin_BLE;
    QueueHandle_t queue_haptics_command_play_custom_BLE;

    /* pointer to haptics configuration struct */
    haptics_command_config_t *haptics_command_config;

    /* pointer to haptics status struct */
    haptics_status_t *haptics_status;

    /* pointer to battery voltage */
    float *battery_voltage;

} params_task_ble_t;

void ble_configure(void);
void task_ble_streaming(void *params);
void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param);

#endif
/**
 * BLE Quick Information
 *
 * GAP (Generic Access Profile)
 *  GAP controls connections and advertising in Bluetooth.
 *  A GAP can either be a Central Device or a Peripheral Device.
 *  Peripheral
 *  Central devices are usually the device that connects to the Peripheral
 *  devices.
 * Advertising Data Payload
 * Scan Response Request
 *
 * Broadcast Network Topology
 *
 * Connected Network Topology
 *
 *
 * GATT (Generic Attribute Profile)
 *  GATT defines ther way two BLE devices transfer data back and forth using Services
 * and Characteristics.
 * Attribute Protocol (ATT) is used to store Services and Characteristics in a lookup table
 * using 16-bit IDs for each entry in the table.
 * GATT comes into play after a BLE connection is established between two devices.
 *
 * BLE peripheral can only be connected to one central device at a time, but one Central device
 * can be connected to multiple Peripheral devices.
 *
 * Peripheral device is the GATT server, while Central device is the GATT client.
 * GATT client sends GATT requests to GATT server, and the GATT server sends GATT
 * responses to the GATT client.
 *
 * Profiles -> Services -> Characteristics
 * Profile contains a collection of related Services specified either by Bluetooth SIG or the
 * peripheral designers.
 *
 * Services break up data into logical entities. Services contain specific chunks of data called
 * Characteristics. Each Service can have one or more Characteristics, and has a unique UUID that
 * is either 16-bit or 128-bit. Bluetooth SIG has many predefined services for many kinds of data.
 *
 * Characteristics represent a single data point. Characteristics also have a pre-defined 16-bit
 * or 128-bit unique UUID. Characteristics can be read from or written to by the Central device.
 */

/**
 * BLE services and characteristics:
 *
 */

#include "../include/ble.h"
#include "../include/haptics_lib.h"

static const char device_name[] = "ITLAB BLE WEARABLE";

static uint16_t HEART_RATE_SERVICE_UUID = 0x180D;
static uint16_t HEART_RATE_CHARACTERISTIC_UUID = 0x2A37;
static uint16_t BODY_SENSOR_LOCATION_CHARACTERISTIC_UUID = 0x2A38;

// 32816f9c-482f-43fb-9e58-5f35d8d5a8e0
static uint8_t ORIENTATION_SERVICE_UUID[] = {
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
static uint8_t ORIENTATION_CHARACTERISTIC_UUID[] = {
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

/* haptic actuators control service
UUID = 3b2e4236-a101-47a2-99cd-cbc128b57126 */
static uint8_t HAPTICS_SERVICE_UUID[] = {
    0x26,
    0x71,
    0xb5,
    0x28,
    0xc1,
    0xcb,
    0xcd,
    0x99,
    0xa2,
    0x47,
    0x01,
    0xa1,
    0x36,
    0x42,
    0x2e,
    0x3b,
};

/* haptics play built in sequence characteristic
UUID = 651539c8-b216-4ede-a768-bee272c47d42 */
static uint8_t HAPTICS_COMMAND_PLAY_BUILTIN_SEQUENCE_CHARACTERISTIC_UUID[] = {
    0x42,
    0x7d,
    0xc4,
    0x72,
    0xe2,
    0xbe,
    0x68,
    0xa7,
    0xde,
    0x4e,
    0x16,
    0xb2,
    0xc8,
    0x39,
    0x15,
    0x65,
};

/* haptics play custom sequence characteristic
UUID = 7af554d5-7c73-4fc8-b32c-d1ae5bc18837 */
static uint8_t HAPTICS_COMMAND_PLAY_CUSTOM_SEQUENCE_CHARACTERISTIC_UUID[] = {
    0x37,
    0x88,
    0xc1,
    0x5b,
    0xae,
    0xd1,
    0x2c,
    0xb3,
    0xc8,
    0x4f,
    0x73,
    0x7c,
    0xd5,
    0x54,
    0xf5,
    0x7a,
};

/* haptics channel configuration characteristic
UUID = 74ea415e-44fc-47c6-a8f9-ab63f0d34913 */
static uint8_t HAPTICS_CONFIG_CHARACTERISTIC_UUID[] = {
    0x13,
    0x49,
    0xd3,
    0xf0,
    0x63,
    0xab,
    0xf9,
    0xa8,
    0xc6,
    0x47,
    0xfc,
    0x44,
    0x5e,
    0x41,
    0xea,
    0x74,
};

/* haptics status characteristic
UUID = c5fd745b-bd37-417f-a28f-22c384a412e7 */
static uint8_t HAPTICS_STATUS_CHARACTERISTIC_UUID[] = {
    0xe7,
    0x12,
    0xa4,
    0x84,
    0xc3,
    0x22,
    0x8f,
    0xa2,
    0x7f,
    0x41,
    0x37,
    0xbd,
    0x5b,
    0x74,
    0xfd,
    0xc5,
};

/* orientation value */
static orientation_data_t orientation_data;
/* heart rate value */
static uint8_t heart_rate = 0;
/* heart rate body sensor location value */
static uint8_t body_sensor_location = 0;
/* haptic actuators play builtin command struct */
static haptics_command_play_builtin_sequence_t haptics_command_builtin;
/* haptic actuators play custom sequence struct */
static haptics_command_play_custom_sequence_t haptics_command_custom;

/* BLE queues struct */
params_task_ble_t ble_values;

/* indicate enabled */
uint8_t notify_enabled[GATT_ORIENTATION_IDX_NB];
/* advertising configuration done */
static uint8_t adv_config_done = 0;
/* service handle integer storage */
uint16_t orientation_service_handle_table[GATT_ORIENTATION_IDX_NB];
uint16_t heart_rate_service_handle_table[GATT_HEART_RATE_IDX_NB];
uint16_t haptics_service_handle_table[GATT_HAPTICS_IDX_NB];
static prepare_type_env_t prepare_write_env;

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = ESP_BLE_GAP_CONN_ITVL_MS(7.5), // slave connection min interval
    .max_interval = ESP_BLE_GAP_CONN_ITVL_MS(20),  // slave connection max interval
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, // test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(ORIENTATION_SERVICE_UUID),
    .p_service_uuid = ORIENTATION_SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .adv_int_max = ESP_BLE_GAP_ADV_ITVL_MS(40),
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = ESP_BLE_GAP_CONN_ITVL_MS(7.5),
    .max_interval = ESP_BLE_GAP_CONN_ITVL_MS(20),
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(ORIENTATION_SERVICE_UUID),
    .p_service_uuid = ORIENTATION_SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* BLE GATT profile instances
 One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if
 returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [APP_ID] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

// /* update orientation data with timestamp and dummy orientation values */
// void update_orientation(orientation_data_t *orientation_data)
// {
//     // orientation_data->euler_angle.timestamp = 100;
//     // orientation_data->euler_angle.x = 30;
//     // orientation_data->euler_angle.y = 40;
//     // orientation_data->euler_angle.z = 50;

//     uint32_t random_number = esp_random() % 361;
//     orientation_data->euler_angle.timestamp = esp_timer_get_time() / 1000;
//     orientation_data->euler_angle.x = -180.0 + random_number;
//     // vTaskDelay(3 / portTICK_PERIOD_MS);
//     random_number = esp_random() % 361;
//     orientation_data->euler_angle.y = -180.0 + random_number;
//     // vTaskDelay(3 / portTICK_PERIOD_MS);
//     random_number = esp_random() % 361;
//     orientation_data->euler_angle.z = -180.0 + random_number;
// }

/* UUID for defining a primary GATT service in the GATT DB  */
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
/* UUID for declaring a GATT characteristic in the GATT DB */
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
/* UUID for declaring a GATT characteristic configuration in the GATT DB */
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

/*
BLE GATT characteristics properties
Read, Write, Notify, or Indicate
*/
/* read-only characteristic property */
static uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
/* write-only characteristic property */
static uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
/* read-write characteristic property */
static uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
/* read-write-notify characteristic property */
static uint8_t char_prop_read_write_indicate = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;
static uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
/*
Client Characteristic Configuration (CCC) descriptor is an additional attribute that describes if the characteristic has notifications enabled.

Notifications for changing orientation values
*/
const uint8_t orientation_ccc[2] = {0x00, 0x00};
const uint8_t heart_rate_ccc[2] = {0x00, 0x00};
const uint8_t haptics_status_ccc[2] = {0x00, 0x00};

/* Full Database Description - Used to add attributes into the database */
static uint8_t s_table_index = 0;
esp_gatts_attr_db_t gatt_db_heart_rate[GATT_HEART_RATE_IDX_NB] = {
    // static const
    /*  heart rate service declaration  */
    [HEART_RATE_IDX_SVC] =
        {
            {ESP_GATT_AUTO_RSP},                    // Auto respond configuration, set to respond automatically by the stack.
            {ESP_UUID_LEN_16,                       // define primary GATT service
             (uint8_t *)&primary_service_uuid,      // define primary GATT service
             ESP_GATT_PERM_READ,                    // read permission
             sizeof(HEART_RATE_SERVICE_UUID),       // max UUID16
             sizeof(HEART_RATE_SERVICE_UUID),       // UUID16
             (uint8_t *)&HEART_RATE_SERVICE_UUID}}, // GATT service UUID

    /* heart rate measurement characteristic declaration */
    [HEART_RATE_IDX_CHAR] = // Named or designated initializer in the enum table.
    {
        {ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
        {ESP_UUID_LEN_16,                        // define a GATT characteristic
         (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
         ESP_GATT_PERM_READ,                     // read permission
         CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
         CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
         (uint8_t *)&char_prop_read_notify}},    // Characteristic is read-notify

    /* heart rate measurement value declaration */
    [HEART_RATE_IDX_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16,                            // UUID128
          (uint8_t *)&HEART_RATE_CHARACTERISTIC_UUID, // GATT characteristic UUID
          ESP_GATT_PERM_READ,                         // read-write permission
          sizeof(heart_rate),                         // max size of value
          sizeof(heart_rate),                         // cur size of value
          (uint8_t *)&heart_rate}},                   // pointer to characteristic value

    /* heart rate measurement notification config declaration */
    [HEART_RATE_IDX_NTF_CFG] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16,                          // UUID16
          (uint8_t *)&character_client_config_uuid, // define a client config UUID
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // read-write permission
          sizeof(uint16_t),                         // max size of value
          sizeof(uint16_t),                         // cur size of value
          (uint8_t *)&heart_rate_ccc}},             // pointer to characteristic value

    /* body sensor location characteristic declaration */
    [BODY_SENSOR_LOCATION_IDX_CHAR] =
        {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
         {ESP_UUID_LEN_16,                        // define a GATT characteristic
          (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
          ESP_GATT_PERM_READ,                     // read permission
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          (uint8_t *)&char_prop_read_notify}},    // Characteristic is read-notify

    /* body sensor location value declaration */
    [BODY_SENSOR_LOCATION_IDX_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_16,                                      // UUID128
          (uint8_t *)&BODY_SENSOR_LOCATION_CHARACTERISTIC_UUID, // GATT characteristic UUID
          ESP_GATT_PERM_READ,                                   // read-write permission
          sizeof(body_sensor_location),                         // max size of value
          sizeof(body_sensor_location),                         // cur size of value
          (uint8_t *)&body_sensor_location}},                   // pointer to characteristic value // replace with heart rate
};

static const esp_gatts_attr_db_t gatt_db_orientation[GATT_ORIENTATION_IDX_NB] =
    {

        /* Orientation Service Declaration */
        [ORIENTATION_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
             {ESP_UUID_LEN_16,                        // define primary GATT service
              (uint8_t *)&primary_service_uuid,       // define primary GATT service
              ESP_GATT_PERM_READ,                     // read permission
              sizeof(ORIENTATION_SERVICE_UUID),       // UUID128
              sizeof(ORIENTATION_SERVICE_UUID),       // UUID128
              (uint8_t *)&ORIENTATION_SERVICE_UUID}}, // GATT service UUID

        /* orientation characteristic declaration */
        [ORIENTATION_IDX_CHAR] =                  // Named or designated initializer in the enum table.
        {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
         {ESP_UUID_LEN_16,                        // define a GATT characteristic
          (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
          ESP_GATT_PERM_READ,                     // read permission
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          (uint8_t *)&char_prop_read_notify}},    // Characteristic is read-write

        /* orientation value declaration */
        [ORIENTATION_IDX_VAL] =
            {{ESP_GATT_AUTO_RSP},
             {ESP_UUID_LEN_128,                           // UUID128
              (uint8_t *)ORIENTATION_CHARACTERISTIC_UUID, // GATT characteristic UUID
              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,   // read-write permission
              500,                                        // max size of value
              sizeof(orientation_data),                   // cur size of value
              (uint8_t *)&orientation_data}},             // pointer to characteristic value

        /* orientation notification configuration declaration */
        [ORIENTATION_IDX_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP},
             {ESP_UUID_LEN_16,                          // UUID128
              (uint8_t *)&character_client_config_uuid, // GATT characteristic UUID
              ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, // read-write permission
              sizeof(uint16_t),                         // max size of value
              sizeof(uint16_t),                         // cur size of value
              (uint8_t *)&orientation_ccc}},            // pointer to characteristic value

};
static const esp_gatts_attr_db_t gatt_db_haptics[GATT_HAPTICS_IDX_NB] = {
    /* Haptics service declaration */
    [HAPTICS_IDX_SVC] =
        {{ESP_GATT_AUTO_RSP},                 // Auto respond configuration, set to respond automatically by the stack.
         {ESP_UUID_LEN_16,                    // define primary GATT service
          (uint8_t *)&primary_service_uuid,   // define primary GATT service
          ESP_GATT_PERM_READ,                 // read permission
          sizeof(HAPTICS_SERVICE_UUID),       // UUID128
          sizeof(HAPTICS_SERVICE_UUID),       // UUID128
          (uint8_t *)&HAPTICS_SERVICE_UUID}}, // GATT service UUID

    /* Haptics command play builtin sequence characteristic declaration */
    [HAPTICS_COMMAND_PLAY_BUILTIN_SEQUENCE_IDX_CHAR] =
        {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
         {ESP_UUID_LEN_16,                        // define a GATT characteristic
          (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
          ESP_GATT_PERM_READ,                     // read permission
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          (uint8_t *)&char_prop_write}},          // Characteristic is read-notify

    /* Haptics command play builtin sequence value declaration */
    [HAPTICS_COMMAND_PLAY_BUILTIN_SEQUENCE_IDX_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_128,                           // UUID128
          (uint8_t *)ORIENTATION_CHARACTERISTIC_UUID, // GATT characteristic UUID
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,   // read-write permission
          sizeof(haptics_command_builtin),            // max size of value
          sizeof(haptics_command_builtin),            // cur size of value
          (uint8_t *)&haptics_command_builtin}},      // pointer to characteristic value

    /* Haptics command play custom sequence characteristic declaration */
    [HAPTICS_COMMAND_PLAY_CUSTOM_SEQUENCE_IDX_CHAR] =
        {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
         {ESP_UUID_LEN_16,                        // define a GATT characteristic
          (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
          ESP_GATT_PERM_READ,                     // read permission
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          (uint8_t *)&char_prop_write}},          // Characteristic is read

    /* Haptics command play custom sequence value declaration */
    [HAPTICS_COMMAND_PLAY_CUSTOM_SEQUENCE_IDX_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_128,                           // UUID128
          (uint8_t *)ORIENTATION_CHARACTERISTIC_UUID, // GATT characteristic UUID
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,   // read-write permission
          sizeof(haptics_command_custom),             // max size of value
          sizeof(haptics_command_custom),             // cur size of value
          (uint8_t *)&haptics_command_custom}},       // pointer to characteristic value

    // /* Haptics configuration characteristic declaration */
    // [HAPTICS_CONFIG_IDX_CHAR] =
    //     {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
    //      {ESP_UUID_LEN_16,                        // define a GATT characteristic
    //       (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
    //       ESP_GATT_PERM_READ,                     // read permission
    //       CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
    //       CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
    //       (uint8_t *)&char_prop_read_notify}},    // Characteristic is read-notify

    // /* Haptics configuration value declaration */
    // [HAPTICS_CONFIG_IDX_VAL] =
    //     {{ESP_GATT_AUTO_RSP},
    //      {ESP_UUID_LEN_128,                                // UUID128
    //       (uint8_t *)ORIENTATION_CHARACTERISTIC_UUID,      // GATT characteristic UUID
    //       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,        // read-write permission
    //       sizeof(haptics_command_config_t),                // max size of value
    //       sizeof(haptics_command_config_t),                // cur size of value
    //       (uint8_t *)&ble_values.haptics_command_config}}, // pointer to characteristic value

    /* Haptics status characteristic declaration */
    [HAPTICS_STATUS_IDX_CHAR] =
        {{ESP_GATT_AUTO_RSP},                     // Auto respond configuration, set to respond automatically by the stack.
         {ESP_UUID_LEN_16,                        // define a GATT characteristic
          (uint8_t *)&character_declaration_uuid, // define a GATT characteristic
          ESP_GATT_PERM_READ,                     // read permission
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          CHAR_DECLARATION_SIZE,                  // characteristic declaration size (uint8_t)
          (uint8_t *)&char_prop_read_notify}},    // Characteristic is read-notify

    /* Haptics status value declaration */
    [HAPTICS_STATUS_IDX_VAL] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_128,                           // UUID128
          (uint8_t *)ORIENTATION_CHARACTERISTIC_UUID, // GATT characteristic UUID
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,   // read-write permission
          sizeof(haptics_status_t),                   // max size of value
          sizeof(haptics_status_t),                   // cur size of value
          (uint8_t *)&ble_values.haptics_status}},    // pointer to characteristic value

    /* Haptics status notification configuration declaration */
    [HAPTICS_STATUS_IDX_NTF_CFG] =
        {{ESP_GATT_AUTO_RSP},
         {ESP_UUID_LEN_128,                           // UUID128
          (uint8_t *)ORIENTATION_CHARACTERISTIC_UUID, // GATT characteristic UUID
          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,   // read-write permission
          sizeof(haptics_status_ccc),                 // max size of value
          sizeof(haptics_status_ccc),                 // cur size of value
          (uint8_t *)&haptics_status_ccc}},           // pointer to characteristic value
};

/**
 * @brief handles different BLE events such as connect, disconnect,
 *
 *
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
        /* BLE GAP advertising data set completed */
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(BLE_TAG, "Advertising data set, status %d", param->adv_data_cmpl.status);
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        /* BLE GAP scan response data set completed */
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(BLE_TAG, "Scan response data set, status %d", param->scan_rsp_data_cmpl.status);
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        /* BLE GAP advertising started */
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(BLE_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(BLE_TAG, "Advertising start successfully");
        break;
        /* BLE GAP update connection params completed */
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(BLE_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
        /* BLE GAP set packet length complete */
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(BLE_TAG, "Packet length update, status %d, rx %d, tx %d",
                 param->pkt_data_length_cmpl.status,
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(BLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.offset > PREPARE_BUF_MAX_SIZE)
    {
        status = ESP_GATT_INVALID_OFFSET;
    }
    else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
    {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }
    if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(BLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(BLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(BLE_TAG, "%s, malloc failed", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        ESP_LOG_BUFFER_HEX(BLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(BLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        /* This event is triggered when a GATT Server application is registered using esp_ble_gatts_app_register */
    case ESP_GATTS_REG_EVT:
    {
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(device_name);
        if (set_dev_name_ret)
        {
            ESP_LOGE(BLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        // ESP_LOGI(BLE_TAG, "before adv");

        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(BLE_TAG, "config adv data failed, error code = %x", ret);
        }

        adv_config_done |= ADV_CONFIG_FLAG;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(BLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;

        esp_err_t create_attr_ret;
        create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_orientation, gatts_if, GATT_ORIENTATION_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(BLE_TAG, "create orientation attr table failed, error code = %x", create_attr_ret);
        }
        create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_heart_rate, gatts_if, GATT_HEART_RATE_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(BLE_TAG, "create heart rate attr table failed, error code = %x", create_attr_ret);
        }
        create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_haptics, gatts_if, GATT_HAPTICS_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(BLE_TAG, "create haptics attr table failed, error code = %x", create_attr_ret);
        }
    }
    break;

    /* This event is triggered when the read request from the Client is received.  */
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_READ_EVT handle %d", param->read.handle);
        break;

        /* This event is triggered when the write request from the Client is received. */
    case ESP_GATTS_WRITE_EVT:
        /* write.is_prep indicates the write operation is a prepared write operation */
        if (!param->write.is_prep)
        {
            // GATTS_DEMO_CHAR_VAL_LEN_MAX
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(BLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            ESP_LOG_BUFFER_HEX(BLE_TAG, param->write.value, param->write.len);

            /*
            receive haptics play builtin sequences commands from phone
            */
            // if (param->write.handle == orientation_service_handle_table[HAPTICS_COMMAND_PLAY_BUILTIN_SEQUENCE_IDX_VAL])
            // {
            //     ESP_LOGI(BLE_TAG, "haptics command builtin write");
            //     memcpy(&haptics_command_builtin, param->write.value, sizeof(haptics_command_play_builtin_sequence_t));
            //     /* push to haptics command play builtin queue */
            // }

            // /*
            // receive haptics custom sequences from phone
            // */
            // if (param->write.handle == orientation_service_handle_table[HAPTICS_COMMAND_PLAY_CUSTOM_SEQUENCE_IDX_VAL])
            // {
            //     ESP_LOGI(BLE_TAG, "haptics command custom write");
            //     memcpy(&haptics_command_custom, param->write.value, sizeof(haptics_command_play_custom_sequence_t));
            //     /* push to haptics command play custom queue */
            // }

            /* toggle notify and indicate in orientation characteristic CCC */
            if (orientation_service_handle_table[ORIENTATION_IDX_NTF_CFG] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                /* */
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(BLE_TAG, "notify enable");

                    /* set notify_enabled for BTN1 characteristic to 1 */
                    notify_enabled[ORIENTATION_IDX_VAL] = 1;
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(BLE_TAG, "indicate enable");

                    /* set notify_enabled for BTN1 characteristic to 1 */
                    notify_enabled[ORIENTATION_IDX_VAL] = 1;
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(BLE_TAG, "notify/indicate disable ");

                    /* set notify_enabled for this characteristic to 0 */
                    notify_enabled[ORIENTATION_IDX_VAL] = 0;
                }
                else
                {
                    ESP_LOGE(BLE_TAG, "unknown descr value");
                    ESP_LOG_BUFFER_HEX(BLE_TAG, param->write.value, param->write.len);
                }
            }

            /* */

            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }

        else
        {
            /* handle prepare write */
            example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;

        /* */
    case ESP_GATTS_EXEC_WRITE_EVT:
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(BLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;

        /**/
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;

        /* This event is triggered when the confirmation from the Client is received. */
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;

        /* This event is triggered when the service is started using esp_ble_gatts_start_service */
    case ESP_GATTS_START_EVT:
        ESP_LOGI(BLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;

        /* This event is triggered when a physical connection is set up. */
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        ESP_LOG_BUFFER_HEX(BLE_TAG, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;

        /* This event is triggered when a physical connection is terminated. */
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(BLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;

    /* This event is triggered when a service attribute table is created using esp_ble_gatts_create_attr_tab */
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        switch (s_table_index)
        {
            /* orientation GATT table */
        case 0:
            // ESP_LOGI(BLE_TAG, "%d %d",
            //          param->add_attr_tab.svc_uuid,
            //          param->add_attr_tab.svc_inst_id);
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(BLE_TAG, "create orientation attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }

            else if (param->add_attr_tab.num_handle != GATT_ORIENTATION_IDX_NB)
            {
                ESP_LOGE(BLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to GATT_ORIENTATION_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, GATT_ORIENTATION_IDX_NB);
            }
            /* populate the uint16_t service handle table */
            else
            {
                ESP_LOGI(BLE_TAG, "create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
                memcpy(orientation_service_handle_table, param->add_attr_tab.handles, sizeof(orientation_service_handle_table));
                esp_ble_gatts_start_service(orientation_service_handle_table[ORIENTATION_IDX_SVC]);
            }
            break;
        /* heart rate GATT table */
        case 1:
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(BLE_TAG, "create heart rate attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }

            else if (param->add_attr_tab.num_handle != GATT_HEART_RATE_IDX_NB)
            {
                ESP_LOGE(BLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to GATT_HEART_RATE_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, GATT_HEART_RATE_IDX_NB);
            }
            /* populate the uint16_t service handle table */
            else
            {
                ESP_LOGI(BLE_TAG, "create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
                memcpy(heart_rate_service_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_service_handle_table));
                esp_ble_gatts_start_service(heart_rate_service_handle_table[HEART_RATE_IDX_SVC]);
            }
            break;
        /* haptics GATT table */
        case 2:
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(BLE_TAG, "create haptics attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }

            else if (param->add_attr_tab.num_handle != GATT_HAPTICS_IDX_NB)
            {
                ESP_LOGE(BLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to GATT_HAPTICS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, GATT_HAPTICS_IDX_NB);
            }
            /* populate the uint16_t service handle table */
            else
            {
                ESP_LOGI(BLE_TAG, "create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
                memcpy(haptics_service_handle_table, param->add_attr_tab.handles, sizeof(haptics_service_handle_table));
                esp_ble_gatts_start_service(haptics_service_handle_table[HAPTICS_IDX_SVC]);
            }
            break;
        }
        s_table_index++;

        break;
    }

    /*  This event is triggered when an attribute value is set using esp_ble_gatts_set_attr_value */
    case ESP_GATTS_SET_ATTR_VAL_EVT:
        // ESP_LOGI(GATTS_TAG, "Attribute value set, status %d, attr_handle %d, srvc_handle %d",
        // 		 param->set_attr_val.status,
        // 		 param->set_attr_val.attr_handle,
        // 		 param->set_attr_val.srvc_handle);
        uint16_t attr_handle = param->set_attr_val.attr_handle;
        uint16_t length = 0;
        const uint8_t *value;

        esp_ble_gatts_get_attr_value(attr_handle, &length, &value);

        /* set orientation attr val */
        if (attr_handle == orientation_service_handle_table[ORIENTATION_IDX_VAL])
        {
            if (notify_enabled[ORIENTATION_IDX_VAL])
            {
                uint8_t indicate_data[length];
                memcpy(indicate_data, value, length);
                esp_ble_gatts_send_indicate(gatts_if, gl_profile_tab[ORIENTATION_IDX_SVC].conn_id, orientation_service_handle_table[ORIENTATION_IDX_VAL],
                                            length, (uint8_t *)indicate_data, false);
            }
        }
        break;

    /* This event is triggered when the service is stopped using esp_ble_gatts_stop_service */
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:

        /* This event is triggered when the service is deleted using esp_ble_gatts_delete_service */
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}

/**
 * @brief BLE GATT event handler
 * The event is captured by the gatts_event_handler() which stores the generated interface in the profile table
 * and then forwards it to the corresponding profile event handler.
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /*
    BLE app register event
    This event is triggered when a GATT Server application is
    registered using esp_ble_gatts_app_register
    */
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(BLE_TAG, "Reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    // gatts_if registered complete, call cb handlers
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    // call the callback
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/**
 * BLE FreeRTOS task
 *
 */
void task_ble_streaming(void *params)
{
    /* orientation data */
    orientation_data_t rcv_orientation_data[15];
    /* orientation data array index */
    uint8_t orientation_arr_idx = 0;
    /* heart rate data */
    uint16_t heart_rate_data;

    ESP_LOGI(BLE_TAG, "ble task started");
    /* UART task parameters */
    params_task_ble_t *params_task_ble = (params_task_ble_t *)params;

    /* orientation data queue */
    QueueHandle_t queue_orientation_BLE = params_task_ble->queue_orientation_BLE;
    ESP_LOGI(BLE_TAG, "orientation queue pointer = %p", queue_orientation_BLE);
    // /* heart rate data queue */
    // QueueHandle_t queue_heart_rate_BLE = params_task_ble->queue_heart_rate_BLE;

    while (1)
    {

        /* receive orientation values from the BLE orientation queue */
        if (queue_orientation_BLE != NULL)
        {
            if (xQueueReceive(queue_orientation_BLE, &rcv_orientation_data[orientation_arr_idx], 100 / portTICK_PERIOD_MS) == pdTRUE)
            {
                /* stream the data out via BLE */
                if (orientation_arr_idx == 14)
                {

                    esp_ble_gatts_set_attr_value(orientation_service_handle_table[ORIENTATION_IDX_VAL],
                                                 sizeof(rcv_orientation_data),
                                                 (uint8_t *)rcv_orientation_data);
                }
                orientation_arr_idx = (orientation_arr_idx + 1) % 15;
            }
        }
        else
        {
            ESP_LOGE(BLE_TAG, "queue_orientation_BLE NULL");
        }

        // /* send heart rate values to phone */
        // if (xQueueReceive(queue_orientation_BLE, &heart_rate_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
        // {
        //     // esp_ble_gatts_set_attr_value(orientation_service_handle_table[HEART_RATE_IDX_VAL],
        //     //                              sizeof(heart_rate_data),
        //     //                              (uint8_t *)&heart_rate_data);
        // }

        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        /*
        send haptics status to phone
        */

        /*
        configure haptics channel configuration from phone
        */
    }
}

/**
 * Configure BLE
 */
void ble_configure(void)
{
    esp_err_t ret;

    /* initialize NVS for BLE stack */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* initialize BT controller */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* enable BT controller */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* initialize Bluedroid stack */
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* enable Bluedroid stack */
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* Set BLE GAP name */
    ret = esp_ble_gap_set_device_name(device_name);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "set device name failed, error code = %x", ret);
        return;
    }

    /*
    register BLE GAP callback handlers
    BLE GAP (generic access profile) governs how devices find and connect to each other.
    Takes effect Pre-connection
    Device roles: Broadcaster, Observer, Peripheral, Central
    */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    /*
    register BLE GATT callback handlers
    BLE GATT (generic attribute profile) governs how structured data is organized and
    exchanged once the connection is established.
    Takes effect post-connection
    Device roles: GATT server, GATT client
    Profiles -> Services -> Characteristics -> Descriptors
    */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    /*
    Register GATT Server application.
    */
    ret = esp_ble_gatts_app_register(APP_ID);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "app register error, error code = %x", ret);
        return;
    }

    /*
    Set BLE MTU (Maximum Transmission Unit)

    BLE MTU stands for Maximum Transmission Unit in Bluetooth Low Energy. It defines the
    maximum size of a single data packet (in bytes) that can be sent between the Client
    and Server at the Attribute Protocol (ATT) layer.
    */
    ret = esp_ble_gatt_set_local_mtu(500);
    if (ret)
    {
        ESP_LOGE(BLE_TAG, "set local  MTU failed, error code = %x", ret);
    }
}
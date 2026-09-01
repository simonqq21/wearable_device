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

/**
 * @brief this struct acts as an application-level state container for a single GATT profile.
 * It stores the stack handles, identifiers, and configuration metadata needed to manage a GATT
 * service, its characteristic, and its descriptor throughout their lifecycle.
 *
 *
 */
struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;       // Profile-specific event callback
    uint16_t gatts_if;             // GATT Interface ID
    uint16_t app_id;               // Application ID
    uint16_t conn_id;              // Connection ID
    uint16_t service_handle;       // Service handle
    esp_gatt_srvc_id_t service_id; // Service ID
    uint16_t char_handle;          // Characteristic handle
    esp_bt_uuid_t char_uuid;       // Characteristic UUID
    esp_gatt_perm_t perm;          //
    esp_gatt_char_prop_t property; // Characteristic property (Read, Write, Notify, Indicate)
    uint16_t descr_handle;         // Characteristic descriptor handle
    esp_bt_uuid_t descr_uuid;      // Characteristic descriptor UUID
};

/**
 * GATT event handlers
 */
static void orientation_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void example_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static const char *GATTS_TAG = "GATTS";
/*
BLE GATT characteristics properties
Read, Write, Notify, or Indicate
*/
/* ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE */
static esp_gatt_char_prop_t orientation_property = 0;
/* ESP_GATT_CHAR_PROP_BIT_WRITE */

/* orientation value */
static orientation_data_t orientation_data;

/* enable BLE indicate */
static bool indicate_enabled = false;
/* orientation create GATT complete */
static bool orientation_create_cmpl = false; // Heart Rate Service
/* advertising configuration done */
static uint8_t adv_config_done = 0;

/* orientation attribute */
esp_attr_value_t orientation_data_attr = {
    .attr_max_len = sizeof(orientation_data_t),
    .attr_len = sizeof(orientation_data_t),
    .attr_value = (uint8_t *)&orientation_data,
};

/* BLE advertising data */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* BLE advertising params */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20, // 20ms
    .adv_int_max = 0x40, // 40ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* BLE GATT profile instances */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [APP_ID] = {
        .gatts_cb = orientation_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

// /* orientation data update and read task */
// static void heart_rate_task(void *param)
// {
//     ESP_LOGI(GATTS_TAG, "Heart Rate Task Start");

//     while (1)
//     {
//         if (orientation_create_cmpl)
//         {
//             // update_heart_rate();
//             // ESP_LOGI(GATTS_TAG, "Heart Rate updated to %d", get_heart_rate());

//             // heart_rate_val[1] = get_heart_rate();
//             esp_ble_gatts_set_attr_value(gl_profile_tab[APP_ID].char_handle, 2, heart_rate_val);
//         }

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

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
        ESP_LOGI(GATTS_TAG, "Advertising data set, status %d", param->adv_data_cmpl.status);
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    /* BLE GAP scan response data set completed */
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Scan response data set, status %d", param->scan_rsp_data_cmpl.status);
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
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    /* BLE GAP update connection params completed */
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    /* BLE GAP set packet length complete */
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                 param->pkt_data_length_cmpl.status,
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

/**
 * @brief orientation data profile event handler
 */
static void orientation_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    /* This event is triggered when a GATT Server application is registered using esp_ble_gatts_app_register */
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[APP_ID].service_id.is_primary = true;
        gl_profile_tab[APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[APP_ID].service_id.id.uuid.uuid.uuid128, ORIENTATION_SERVICE_UUID, ESP_UUID_LEN_128);

        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            break;
        }

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[APP_ID].service_id, ORIENTATION_NUM_HANDLE);
        break;

    /* This event is triggered when a GATT Server service is created using esp_ble_gatts_create_service */
    case ESP_GATTS_CREATE_EVT:
        // service has been created, now add characteristic declaration
        ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[APP_ID].char_uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile_tab[APP_ID].char_uuid.uuid.uuid128, ORIENTATION_CHARACTERISTIC_UUID, ESP_UUID_LEN_128);
        esp_ble_gatts_start_service(gl_profile_tab[APP_ID].service_handle);
        orientation_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_INDICATE;
        ret = esp_ble_gatts_add_char(gl_profile_tab[APP_ID].service_handle, &gl_profile_tab[APP_ID].char_uuid,
                                     ESP_GATT_PERM_READ,
                                     orientation_property,
                                     &orientation_data_attr, NULL);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", ret);
        }
        break;

    /* This event is triggered when a characteristic is added to the service using esp_ble_gatts_add_char */
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, char_uuid %x",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid128);
        gl_profile_tab[APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        ESP_LOGI(GATTS_TAG, "orientation data char handle %d", param->add_char.attr_handle);
        ret = esp_ble_gatts_add_char_descr(gl_profile_tab[APP_ID].service_handle, &gl_profile_tab[APP_ID].descr_uuid,
                                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        break;

    /* This event is triggered when a characteristic descriptor is added to the service using esp_ble_gatts_add_char_descr */
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %u",
                 param->add_char_descr.status, param->add_char_descr.attr_handle);
        gl_profile_tab[APP_ID].descr_handle = param->add_char_descr.attr_handle;
        orientation_create_cmpl = true;
        break;

    /* This event is triggered when the read request from the Client is received.  */
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic read");
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 2;
        memcpy(rsp.attr_value.value, &orientation_data, sizeof(orientation_data));
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    /* This event is triggered when the write request from the Client is received. */
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "Characteristic write, value len %u, value ", param->write.len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

        if (gl_profile_tab[APP_ID].descr_handle == param->write.handle && param->write.len == 2)
        {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0001)
            {
                if (orientation_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                {
                    ESP_LOGI(GATTS_TAG, "Notification enable");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); i++)
                    {
                        notify_data[i] = i % 0xff;
                    }
                    // the size of notify_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                }
            }
            else if (descr_value == 0x0002)
            {
                if (orientation_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                {
                    ESP_LOGI(GATTS_TAG, "Indication enable");
                    indicate_enabled = true;
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); i++)
                    {
                        indicate_data[i] = i % 0xff;
                    }
                    // the size of indicate_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                }
            }
            else if (descr_value == 0x0000)
            {
                indicate_enabled = false;
                ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "Invalid descriptor value");
                ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
            }
        }
        example_write_event_env(gatts_if, param);
        break;

    /* This event is triggered when the service is deleted using esp_ble_gatts_delete_service */
    case ESP_GATTS_DELETE_EVT:
        break;

    /* This event is triggered when the service is started using esp_ble_gatts_start_service */
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;

    /* This event is triggered when the service is stopped using esp_ble_gatts_stop_service */
    case ESP_GATTS_STOP_EVT:
        break;

    /* This event is triggered when a physical connection is set up. */
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
        gl_profile_tab[APP_ID].conn_id = param->connect.conn_id;
        break;

    /* This event is triggered when a physical connection is terminated. */
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        indicate_enabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    /* This event is triggered when the confirmation from the Client is received. */
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;

    /* This event is triggered when an attribute value is set using esp_ble_gatts_set_attr_value */
    case ESP_GATTS_SET_ATTR_VAL_EVT:
        ESP_LOGI(GATTS_TAG, "Attribute value set, status %d", param->set_attr_val.status);
        if (indicate_enabled)
        {
            uint8_t indicate_data[sizeof(orientation_data_t)] = {0};
            memcpy(indicate_data, &orientation_data, sizeof(orientation_data));
            esp_ble_gatts_send_indicate(gatts_if, gl_profile_tab[APP_ID].conn_id, gl_profile_tab[APP_ID].char_handle, sizeof(indicate_data), indicate_data, true);
        }
        break;

    default:
        break;
    }
}

// static void auto_io_gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     switch (event)
//     {
//     case ESP_GATTS_REG_EVT:
//         ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d", param->reg.status, param->reg.app_id);
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.is_primary = true;
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.inst_id = 0x00;
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id.id.uuid.uuid.uuid128 = AUTO_IO_SVC_UUID;
//         esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_id, AUTO_IO_NUM_HANDLE);
//         break;
//     case ESP_GATTS_CREATE_EVT:
//         // service has been created, now add characteristic declaration
//         ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle = param->create.service_handle;
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_128;
//         memcpy(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid.uuid.uuid128, led_chr_uuid, ESP_UUID_LEN_128);

//         esp_ble_gatts_start_service(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle);
//         auto_io_property = ESP_GATT_CHAR_PROP_BIT_WRITE;
//         esp_err_t ret = esp_ble_gatts_add_char(gl_profile_tab[AUTO_IO_PROFILE_APP_ID].service_handle, &gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_uuid,
//                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//                                                auto_io_property,
//                                                &led_status_attr, NULL);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "add char failed, error code = %x", ret);
//         }
//         break;
//     case ESP_GATTS_ADD_CHAR_EVT:
//         ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, char_uuid %x",
//                  param->add_char.status, param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid128);
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
//         break;
//     case ESP_GATTS_ADD_CHAR_DESCR_EVT:
//         ESP_LOGI(GATTS_TAG, "Descriptor add, status %d", param->add_char_descr.status);
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
//         break;
//     case ESP_GATTS_READ_EVT:
//         ESP_LOGI(GATTS_TAG, "Characteristic read");
//         esp_gatt_rsp_t rsp;
//         memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

//         rsp.attr_value.handle = param->read.handle;
//         rsp.attr_value.len = 1;
//         rsp.attr_value.value[0] = 0x02;
//         esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
//         break;
//     case ESP_GATTS_WRITE_EVT:
//         ESP_LOGI(GATTS_TAG, "Characteristic write, value len %u, value ", param->write.len);
//         ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
//         if (param->write.len > 0)
//         {
//             if (param->write.value[0])
//             {
//                 ESP_LOGI(GATTS_TAG, "LED ON!");
//                 // led_on();
//             }
//             else
//             {
//                 ESP_LOGI(GATTS_TAG, "LED OFF!");
//                 // led_off();
//             }
//         }
//         else
//         {
//             ESP_LOGW(GATTS_TAG, "Empty write data received");
//         }
//         example_write_event_env(gatts_if, param);
//         break;
//     case ESP_GATTS_DELETE_EVT:
//         break;
//     case ESP_GATTS_START_EVT:
//         ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d", param->start.status, param->start.service_handle);
//         break;
//     case ESP_GATTS_STOP_EVT:
//         break;
//     case ESP_GATTS_CONNECT_EVT:
//         esp_ble_conn_update_params_t conn_params = {0};
//         memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
//         conn_params.latency = 0;
//         conn_params.max_int = 0x20;
//         conn_params.min_int = 0x10;
//         conn_params.timeout = 400;
//         ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
//                  param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
//         gl_profile_tab[AUTO_IO_PROFILE_APP_ID].conn_id = param->connect.conn_id;
//         esp_ble_gap_update_conn_params(&conn_params);
//         break;
//     case ESP_GATTS_DISCONNECT_EVT:
//         ESP_LOGI(GATTS_TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
//                  ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
//         break;
//     case ESP_GATTS_CONF_EVT:
//         ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
//         if (param->conf.status != ESP_GATT_OK)
//         {
//             ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
//         }
//         break;
//     default:
//         break;
//     }
// }

/**
 * BLE GATT event handler
 *
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /*
    BLE app register event
    This event is triggered when a GATT Server application is
    registered using esp_ble_gatts_app_register
    */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
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
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

// /**
//  * BLE FreeRTOS task
//  *
//  */
// void task_ble_streaming(void *params)
// {
// }

/**
 * @brief send response back to client
 */
void example_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
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
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* initialize BT controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* enable BT controller */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* initialize Bluedroid stack */
    esp_bluedroid_config_t cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* enable Bluedroid stack */
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* Set BLE GAP name */
    ret = esp_ble_gap_set_device_name(device_name);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", ret);
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
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
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
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    /*
    Register GATT Server application.
    */
    ret = esp_ble_gatts_app_register(APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "app register error, error code = %x", ret);
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
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", ret);
    }

    // xTaskCreate(heart_rate_task, "Heart Rate", 2 * 1024, NULL, 5, NULL);
}
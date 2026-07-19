#include "../include/button_module.h"

// button
// ****************************************************************

// extern QueueHandle_t status_led_queue;
extern TaskHandle_t task_handle_status_led;
extern TaskHandle_t task_handle_imu_data;
extern TaskHandle_t task_handle_main_data_logging;
extern TaskHandle_t task_handle_SD_card_datalogger;

uint8_t datalogging;

cmd_task_status_led_t status_led_state_from_button;

/*
notify IMU task to start datalogging
notify IMU task to stop datalogging
*/
void button_toggle_datalogging_cb(void *arg, void *data)
{
    if (datalogging)
    {
        datalogging = false;
        // status_led_state_from_button = STATUS_LED_OFF;
        // xTaskNotifyGiveIndexed(task_handle_status_led, 0);

        // stop datalogging task
        if (task_handle_main_data_logging != NULL)
            xTaskNotify(task_handle_main_data_logging, DATALOGGING_STOP, eSetValueWithOverwrite);
        //  start IMU task
        if (task_handle_imu_data != NULL)
            xTaskNotify(task_handle_imu_data, IMU_STOP, eSetValueWithOverwrite);
        // turn off the status LED
        if (task_handle_status_led != NULL)
            xTaskNotify(task_handle_status_led, STATUS_LED_OFF, eSetValueWithOverwrite);
        // stop SD card datalogging
        if (task_handle_SD_card_datalogger != NULL)
            xTaskNotify(task_handle_SD_card_datalogger, SD_CARD_STOP, eSetValueWithOverwrite);
        ESP_LOGI(BUTTON_TAG, "Datalogging stopped.");
    }
    else
    {
        datalogging = true;
        // status_led_state_from_button = STATUS_LED_BLINK;
        // xTaskNotifyGiveIndexed(task_handle_status_led, 0);

        // start datalogging task
        if (task_handle_main_data_logging != NULL)
            xTaskNotify(task_handle_main_data_logging, DATALOGGING_GO, eSetValueWithOverwrite);
        //  start IMU task
        if (task_handle_imu_data != NULL)
            xTaskNotify(task_handle_imu_data, IMU_READ_LOOP, eSetValueWithOverwrite);
        // turn on the status LED
        if (task_handle_status_led != NULL)
            xTaskNotify(task_handle_status_led, STATUS_LED_BLINK, eSetValueWithOverwrite);
        // start SD card datalogging
        if (task_handle_SD_card_datalogger != NULL)
            xTaskNotify(task_handle_SD_card_datalogger, SD_CARD_START, eSetValueWithOverwrite);
        ESP_LOGI(BUTTON_TAG, "Datalogging started.");
    }
    // xQueueSend(status_led_queue, &status_led_state_from_button, 10);
}

/*
notify IMU task to pause datalogging and initiate calibration
*/
void button_trigger_calibration_cb(void *arg, void *data)
{
    ESP_LOGI(BUTTON_TAG, "Calibrating IMU...");
    // pause datalogging
    if (task_handle_main_data_logging != NULL)
        xTaskNotify(task_handle_main_data_logging, DATALOGGING_STOP, eSetValueWithOverwrite);
    // pause IMU reading task to perform calibration
    if (task_handle_imu_data != NULL)
        xTaskNotify(task_handle_imu_data, IMU_CALIBRATE, eSetValueWithOverwrite);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // xTaskNotify(task_handle_imu, IMU_STOP, eSetValueWithOverwrite);
}

// initialize button
esp_err_t buttons_init(void)
{
    // configure button
    const button_config_t btn_cfg = {
        long_press_time : 2000,
        short_press_time : 60,
    };
    const button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = BTN1_PIN,
        .active_level = BUTTON_INACTIVE,
        .enable_power_save = false,

    };

    // initialize buttons
    button_handle_t btn = NULL;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn);
    if (ret != ESP_OK)
    {
        return ret;
    }

    /*
    Button actions:
    short press - toggle datalogging
    long press - calibrate + self-test IMU
    */
    iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_toggle_datalogging_cb, NULL);
    iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_trigger_calibration_cb, NULL);
    return ESP_OK;
}
// ****************************************************************

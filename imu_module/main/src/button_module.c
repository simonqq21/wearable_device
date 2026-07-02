#include "../include/button_module.h"

// button
// ****************************************************************

// extern QueueHandle_t status_led_queue;
extern TaskHandle_t task_handle_status_led;
extern TaskHandle_t task_handle_imu;
extern TaskHandle_t task_handle_data_logging;

extern uint8_t datalogging;

status_led_task_notif_t status_led_state_from_button;

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
        xTaskNotify(task_handle_status_led, STATUS_LED_OFF, eSetValueWithOverwrite);
        ESP_LOGI(BUTTON_TAG, "Datalogging stopped.");
    }
    else
    {
        datalogging = true;
        // status_led_state_from_button = STATUS_LED_BLINK;
        // xTaskNotifyGiveIndexed(task_handle_status_led, 0);
        xTaskNotify(task_handle_status_led, STATUS_LED_BLINK, eSetValueWithOverwrite);
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
    // pause IMU reading task

    // perform calibration
    xTaskNotify(task_handle_imu, 1, eSetValueWithOverwrite);

    // resume datalogging
    // resume IMU reading task
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

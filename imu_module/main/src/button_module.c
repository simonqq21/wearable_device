#include "../include/button_module.h"

// button
// ****************************************************************

extern QueueHandle_t status_led_queue;
extern uint8_t datalogging;

status_led_state_t status_led_state_from_button;
// button callback format
void button_toggle_datalogging_cb(void *arg, void *data)
{
    if (datalogging)
    {
        datalogging = false;
        status_led_state_from_button = STATUS_LED_OFF;
        ESP_LOGI(BUTTON_TAG, "Datalogging stopped.");
    }
    else
    {
        datalogging = true;
        status_led_state_from_button = STATUS_LED_BLINK;
        ESP_LOGI(BUTTON_TAG, "Datalogging started.");
    }
    xQueueSend(status_led_queue, &status_led_state_from_button, 10);
}

void button_trigger_calibration_cb(void *arg, void *data)
{
    ESP_LOGI(BUTTON_TAG, "Calibrating IMU...");
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

#include "../include/gpio_module.h"

// task handles for GPIO, sensors, sensor fusion, and SD card datalogging
extern TaskHandle_t task_handle_GPIO;
extern TaskHandle_t task_handle_sensors;
extern TaskHandle_t task_handle_sensor_fusion;
extern TaskHandle_t task_handle_SD_card_datalogger;

uint8_t btn_datalogging;

extern button_handle_t btn1;

cmd_task_status_led_t status_led_state_from_button;

void gpio_init()
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    gpio_config(&io_conf);

    buttons_init();
}
// gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
// gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);

esp_err_t buttons_init()
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
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn1);
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

void buttons_set_callbacks(void)
{
    /*
    Button actions:
    short press - toggle datalogging
    long press - calibrate + self-test IMU
    */
    iot_button_register_cb(btn1, BUTTON_SINGLE_CLICK, NULL, button_toggle_datalogging_cb, NULL);
    iot_button_register_cb(btn1, BUTTON_LONG_PRESS_START, NULL, button_trigger_calibration_cb, NULL);
}

void set_datalogging_led(uint8_t led_value)
{
    gpio_set_level(LED1_PIN, led_value);
}

void button_trigger_calibration_cb(void *arg, void *data)
{
    ESP_LOGI(BUTTON_TAG, "Calibrating IMU...");
    // pause datalogging
    if (task_handle_sensor_fusion != NULL)
        xTaskNotify(task_handle_sensor_fusion, CMD_DATALOGGING_STOP, eSetValueWithOverwrite);
    // pause IMU reading task to perform calibration
    if (task_handle_sensors != NULL)
        xTaskNotify(task_handle_sensors, CMD_IMU_CALIBRATE, eSetValueWithOverwrite);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // xTaskNotify(task_handle_imu, CMD_IMU_STOP, eSetValueWithOverwrite);
}

void button_toggle_datalogging_cb(void *arg, void *data)
{
    if (btn_datalogging)
    {
        btn_datalogging = false;
        // status_led_state_from_button = STATUS_LED_OFF;
        // xTaskNotifyGiveIndexed(task_handle_GPIO, 0);

        // stop datalogging task
        if (task_handle_sensor_fusion != NULL)
            xTaskNotify(task_handle_sensor_fusion, CMD_DATALOGGING_STOP, eSetValueWithOverwrite);
        //  start IMU task
        if (task_handle_sensors != NULL)
            xTaskNotify(task_handle_sensors, CMD_IMU_STOP, eSetValueWithOverwrite);
        // turn off the status LED
        if (task_handle_GPIO != NULL)
            xTaskNotify(task_handle_GPIO, STATUS_LED_OFF, eSetValueWithOverwrite);
        // stop SD card datalogging
        if (task_handle_SD_card_datalogger != NULL)
            xTaskNotify(task_handle_SD_card_datalogger, CMD_SD_CARD_STOP, eSetValueWithOverwrite);
        ESP_LOGI(BUTTON_TAG, "Datalogging stopped.");
    }
    else
    {
        btn_datalogging = true;
        // status_led_state_from_button = STATUS_LED_BLINK;
        // xTaskNotifyGiveIndexed(task_handle_GPIO, 0);

        // start datalogging task
        if (task_handle_sensor_fusion != NULL)
            xTaskNotify(task_handle_sensor_fusion, CMD_DATALOGGING_GO, eSetValueWithOverwrite);
        //  start IMU task
        if (task_handle_sensors != NULL)
            xTaskNotify(task_handle_sensors, CMD_IMU_READ_LOOP, eSetValueWithOverwrite);
        // turn on the status LED
        if (task_handle_GPIO != NULL)
            xTaskNotify(task_handle_GPIO, STATUS_LED_BLINK, eSetValueWithOverwrite);
        // start SD card datalogging
        if (task_handle_SD_card_datalogger != NULL)
            xTaskNotify(task_handle_SD_card_datalogger, CMD_SD_CARD_START, eSetValueWithOverwrite);
        ESP_LOGI(BUTTON_TAG, "Datalogging started.");
    }
}

// void set_gpio_task_handles(TaskHandle_t ptask_handle_GPIO,
//                            TaskHandle_t ptask_handle_sensors,
//                            TaskHandle_t ptask_handle_main_datalogging,
//                            TaskHandle_t ptask_handle_SD_card_datalogger)
// {
//     task_handle_GPIO = ptask_handle_GPIO;
//     task_handle_sensors = ptask_handle_sensors;
//     task_handle_sensor_fusion = ptask_handle_main_datalogging;
//     task_handle_SD_card_datalogger = ptask_handle_SD_card_datalogger;
// }

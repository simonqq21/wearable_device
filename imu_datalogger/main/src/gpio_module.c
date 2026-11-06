#include "../include/gpio_module.h"

/* structs for atomic task commands structs */
extern sensors_cmd_struct_t sensors_cmd_struct;
extern sensor_fusion_cmd_struct_t sensor_fusion_cmd_struct;
extern gpio_cmd_struct_t gpio_cmd_struct;
extern sd_card_cmd_struct_t sd_card_cmd_struct;

atomic_bool btn_datalogging;

extern button_handle_t btn1;

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

/**
 * @brief
 * Button actions:
 * short press - toggle datalogging
 * long press - calibrate + self-test IMU
 *
 */
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

void set_datalogging_led(uint8_t led_value)
{
    gpio_set_level(LED1_PIN, led_value);
}
void button_trigger_calibration_cb(void *arg, void *data)
{
    ESP_LOGI(BUTTON_TAG, "Calibrating IMU...");
    /* stop datalogging */
    atomic_store(&sensor_fusion_cmd_struct.imu_sensor_fusion_cmd, CMD_IMU_SENSOR_FUSION_STOP);
    atomic_store(&sensor_fusion_cmd_struct.heart_rate_sensor_fusion_cmd, CMD_HEART_RATE_SENSOR_FUSION_STOP);

    /* stop all sensors */
    atomic_store(&sensors_cmd_struct.imu_cmd, CMD_IMU_STOP);
    atomic_store(&sensors_cmd_struct.heart_rate_cmd, CMD_HEART_RATE_STOP);

    /* turn off the status LED */
    atomic_store(&gpio_cmd_struct.status_led_cmd, CMD_STATUS_LED_OFF);

    /* run calibration routine */
    atomic_store(&sensors_cmd_struct.imu_cmd, CMD_IMU_CALIBRATE);
}

void button_toggle_datalogging_cb(void *arg, void *data)
{
    bool btn_datalogging_ = atomic_load(&btn_datalogging);
    if (btn_datalogging_)
    {
        /* stop datalogging */
        atomic_store(&btn_datalogging, false);

        /* stop all sensor fusion */
        atomic_store(&sensor_fusion_cmd_struct.imu_sensor_fusion_cmd, CMD_IMU_SENSOR_FUSION_STOP);
        atomic_store(&sensor_fusion_cmd_struct.heart_rate_sensor_fusion_cmd, CMD_HEART_RATE_SENSOR_FUSION_STOP);

        /* stop all sensors */
        atomic_store(&sensors_cmd_struct.imu_cmd, CMD_IMU_STOP);
        atomic_store(&sensors_cmd_struct.heart_rate_cmd, CMD_HEART_RATE_STOP);

        /* turn off the status LED */
        atomic_store(&gpio_cmd_struct.status_led_cmd, CMD_STATUS_LED_OFF);

        /* stop SD card datalogging */
        atomic_store(&sd_card_cmd_struct.sd_card_cmd, CMD_SD_CARD_STOP);

        ESP_LOGI(BUTTON_TAG, "Datalogging stopped.");
    }
    else
    {
        /* start datalogging */
        atomic_store(&btn_datalogging, true);

        /* start all sensor fusion */
        atomic_store(&sensor_fusion_cmd_struct.imu_sensor_fusion_cmd, CMD_IMU_SENSOR_FUSION_GO);
        atomic_store(&sensor_fusion_cmd_struct.heart_rate_sensor_fusion_cmd, CMD_HEART_RATE_SENSOR_FUSION_GO);

        /* start all sensors */
        atomic_store(&sensors_cmd_struct.imu_cmd, CMD_IMU_START);
        atomic_store(&sensors_cmd_struct.heart_rate_cmd, CMD_HEART_RATE_START);

        /* turn on the status LED */
        atomic_store(&gpio_cmd_struct.status_led_cmd, CMD_STATUS_LED_BLINK);

        /* stop SD card datalogging */
        atomic_store(&sd_card_cmd_struct.sd_card_cmd, CMD_SD_CARD_START);
        ESP_LOGI(BUTTON_TAG, "Datalogging started.");
    }
}
void buttons_set_callbacks(void)
{
    iot_button_register_cb(btn1, BUTTON_SINGLE_CLICK, NULL, button_toggle_datalogging_cb, NULL);
    iot_button_register_cb(btn1, BUTTON_LONG_PRESS_START, NULL, button_trigger_calibration_cb, NULL);
}

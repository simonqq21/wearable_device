#include <stdio.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "freertos/semphr.h"

#include <esp_err.h>
#include <esp_log.h>

// #include "esp_flash.h"
// #include "esp_chip_info.h"
// #include "spi_flash_mmap.h"
// #include "esp_littlefs.h"

#include "driver/gpio.h"
#include "iot_button.h"
#include "button_gpio.h"

#include "include/uart.h"
#include "include/nvs.h"
#include "include/imu.h"

#define MAIN_TAG "MAIN"

// LED
// ****************************************************************
#define LED1_PIN 2 // power/status LED
#define LED2_PIN 4 // recording LED
#define GPIO_OUTPUT_PIN_SEL ((1ULL << LED1_PIN) | (1ULL << LED2_PIN))

static void gpio_init(void)
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
}
// gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
// gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
// ****************************************************************

// button
// ****************************************************************
#define BTN1_PIN 5

// button callback format
static void button_event_cb(void *arg, void *data)
{
}

// initialize button
esp_err_t buttons_init(void)
{
    // configure button
    const button_config_t btn_cfg = {0};
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
    // iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb, NULL);
    // iot_button_register_cb(btn, BUTTON_LONG_PRESS, NULL, button_event_cb, NULL);
    return ESP_OK;
}
// ****************************************************************

// littleFS
// ****************************************************************

// ****************************************************************

// tasks
// ****************************************************************
QueueHandle_t imu_data_queue;

// FreeRTOS task to log sensor data
#define DATA_LOGGING_TAG "DATA LOGGING"
static void data_logging_task(void *pvParameters)
{
    uint8_t imu_data_received;
    imu_data_t imu_data;
    while (1)
    {
        // receive IMU data from queue
        imu_data_received = xQueueReceive(imu_data_queue, (void *)&imu_data, 100 / portTICK_PERIOD_MS);

        // receive data from other sensors' queues

        // log data to CSV file.
        if (imu_data_received)
        {
            // log IMU data
            ESP_LOGI(DATA_LOGGING_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_data.acc.x, imu_data.acc.y, imu_data.acc.z);
            ESP_LOGI(DATA_LOGGING_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_data.rot.x, imu_data.rot.y, imu_data.rot.z);
            ESP_LOGI(DATA_LOGGING_TAG, "Temperature:  %.1f\n", imu_data.temp);
        }
    }
}
// ****************************************************************

// main
imu_data_t data;
extern int imu_queue_len;
void app_main(void)
{
    // initialize GPIO

    // initialize i2c
    ESP_ERROR_CHECK(i2cdev_init());
    // initialize NVS
    nvs_init();
    // initialize UART
    uart_configure();
    // initialize MPU6050
    // mpu6050_init(&dev);
    // mpu6050_calibrate(&dev, NULL, NULL);
    // ESP_LOGI(MAIN_TAG, "IMU calibrated");
    mpu6050_init_custom(false);
    // initialize littleFS

    // initialize SD card

    ESP_LOGI(MAIN_TAG, "Starting all tasks!");
    // initialize all queues
    imu_data_queue = xQueueCreate(imu_queue_len, sizeof(imu_data_t));
    // initialize all tasks
    // IMU reading task
    xTaskCreatePinnedToCore(imu_read_task,
                            "imu read task",
                            2048,
                            NULL,
                            1,
                            NULL,
                            0);
    // reading tasks of other sensors

    // sensor data logging task
    xTaskCreatePinnedToCore(data_logging_task,
                            "data logging task",
                            2048,
                            NULL,
                            1,
                            NULL,
                            0);
    while (1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

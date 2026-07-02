#include "common.h"

// #include "esp_flash.h"
// #include "esp_chip_info.h"
// #include "spi_flash_mmap.h"
// #include "esp_littlefs.h"

#include "include/uart.h"
#include "include/nvs.h"
#include "include/led_module.h"
#include "include/button_module.h"
#include "include/imu.h"

#define MAIN_TAG "MAIN"

// task handles
// ****************************************************************
TaskHandle_t task_handle_status_led;
TaskHandle_t task_handle_imu;
TaskHandle_t task_handle_data_logging;

// queue for status LED states
QueueHandle_t status_led_queue;
QueueHandle_t imu_data_queue;

// bool datalogging = false;
datalogging_task_cmd_t datalogging_task_cmd, previous_datalogging_task_cmd;

// FreeRTOS task to log sensor data
#define DATA_LOGGING_TAG "DATA LOGGING"
static void data_logging_task(void *pvParameters)
{
    uint8_t imu_data_received;
    imu_data_t imu_data;
    while (1)
    {
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&datalogging_task_cmd,
                        IMU_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
        switch (datalogging_task_cmd)
        {
        case DATALOGGING_START:
            if (previous_datalogging_task_cmd == DATALOGGING_STOP)
            {
                previous_datalogging_task_cmd = DATALOGGING_START;
                ESP_LOGI(DATA_LOGGING_TAG, "Datalogging started.");
            }

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
            break;
        case DATALOGGING_STOP:
            if (previous_datalogging_task_cmd == DATALOGGING_START)
            {
                previous_datalogging_task_cmd = DATALOGGING_STOP;
                ESP_LOGI(DATA_LOGGING_TAG, "Datalogging stopped.");
            }

            break;
        default:
            break;
        }
    }
}
// ****************************************************************

// main
imu_data_t data;
void app_main(void)
{
    // initialize GPIO
    gpio_init();
    // initialize buttons
    buttons_init();
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
    imu_data_queue = xQueueCreate(IMU_QUEUE_LEN, sizeof(imu_data_t));
    // initialize all tasks
    // IMU reading task
    xTaskCreatePinnedToCore(imu_task,
                            "imu calibrate and read task",
                            2048,
                            NULL,
                            1,
                            &task_handle_imu,
                            0);
    // reading tasks of other sensors

    // sensor data logging task
    xTaskCreatePinnedToCore(data_logging_task,
                            "data logging task",
                            2048,
                            NULL,
                            1,
                            &task_handle_data_logging,
                            0);

    // notification LED task
    xTaskCreatePinnedToCore(notif_led_task,
                            "notification LED task",
                            1024,
                            NULL,
                            5,
                            &task_handle_status_led,
                            0);

    // button task
    while (1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

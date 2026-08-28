

// #include "esp_flash.h"
// #include "esp_chip_info.h"
// #include "spi_flash_mmap.h"
// #include "esp_littlefs.h"

#include "include/uart.h"
#include "include/nvs.h"
#include "include/led_module.h"
#include "include/button_module.h"
#include "include/imu.h"
#include "include/sd_card.h"
#include "esp_timer.h"
#include "common.h"
#include "config.h"
#include "datalogging.h"

#define MAIN_TAG "MAIN"

// task handles
// ****************************************************************
TaskHandle_t task_handle_status_led;
TaskHandle_t task_handle_imu_data;
TaskHandle_t task_handle_main_data_logging;
TaskHandle_t task_handle_SD_card_datalogger;

/* queue for IMU data coming from IMU task to datalogging task */
QueueHandle_t queue_imu;
/* queues for raw IMU data from datalogging task to the SD card, UART, and BLE */
QueueHandle_t queue_raw_sdcard;
/* queue for orientation from datalogging task to the SD card, UART, and BLE */
QueueHandle_t queue_orientation_sdcard, queue_orientation_UART, queue_orientation_BLE;
/* streambuffers */
StreamBufferHandle_t streambuffer_imu, streambuffer_sd;

// bool datalogging = false;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

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

    /* initialize the i2c bus */
    i2c_master_init(&bus_handle, &dev_handle, LSM6DS3_SENSOR_ADDR);
    ESP_LOGI(MAIN_TAG, "I2C initialized successfully");

    // initialize NVS
    nvs_init();
    // initialize UART
    uart_configure();
    // initialize MPU6050
    // mpu6050_init(&dev);
    // mpu6050_calibrate(&dev, NULL, NULL);
    // ESP_LOGI(MAIN_TAG, "IMU calibrated");
    imu_set_offset_read_cb(NVS_read_imu_calibration_offsets);
    imu_set_offset_write_cb(NVS_write_imu_calibration_offsets);
    imu_init(dev_handle, IMU_ODR_HZ, IMU_XL_FS, IMU_G_FS, false);
    // initialize littleFS

    /*
    initialize SD card
    The SD card is an integral part to datalogging.
    If the card doesn't initialize, attempt to initialize n times in a loop.
    */
    sd_card_configure_wrapper();
    ESP_LOGI(MAIN_TAG, "Starting all tasks!");

    /* create all queues */
    queue_imu = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_data_t));
    queue_raw_sdcard = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));
    queue_orientation_sdcard = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));
    queue_orientation_UART = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));
    queue_orientation_BLE = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));

    /* create all streambuffers */
    streambuffer_imu = xStreamBufferCreate(sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS);
    streambuffer_sd = xStreamBufferCreate(sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS);

    /* create all parameters */
    params_imu_task_t task_imu_params;
    task_main_datalogging_params_t task_main_datalogging_params;
    task_SD_card_datalogger_params_t task_SD_card_datalogger_params;

    /* task_imu_params */
    task_imu_params.task_handle_status_led = &task_handle_status_led;
    task_imu_params.queue_imu = queue_imu;
    task_imu_params.streambuffer_imu = streambuffer_imu;
    task_imu_params.dev_handle = dev_handle;

    /* task_main_datalogging_params */
    task_main_datalogging_params.queue_imu = queue_imu;
    task_main_datalogging_params.streambuffer_imu = streambuffer_imu;
    task_main_datalogging_params.queue_raw_sdcard = queue_raw_sdcard;
    task_main_datalogging_params.queue_orientation_sdcard = queue_orientation_sdcard;
    task_main_datalogging_params.queue_orientation_UART = queue_orientation_UART;
    task_main_datalogging_params.queue_orientation_BLE = queue_orientation_BLE;
    task_main_datalogging_params.streambuffer_sd = streambuffer_sd;

    /* task_SD_card_datalogger_params */
    task_SD_card_datalogger_params.queue_sdcard = queue_raw_sdcard;
    task_SD_card_datalogger_params.streambuffer_sd = streambuffer_sd;

    /* initialize all tasks */
    // IMU reading task
    xTaskCreatePinnedToCore(task_imu,
                            "imu calibrate and read task",
                            8192,
                            &task_imu_params,
                            2,
                            &task_handle_imu_data,
                            0);

    // sensor data logging task
    xTaskCreatePinnedToCore(task_main_datalogging,
                            "main data logging task",
                            8192,
                            &task_main_datalogging_params,
                            3,
                            &task_handle_main_data_logging,
                            0);

    // notification LED task
    xTaskCreatePinnedToCore(task_notification_LED,
                            "notification LED task",
                            1024,
                            NULL,
                            10,
                            &task_handle_status_led,
                            0);

    // micro SD card datalogging task
    xTaskCreatePinnedToCore(task_SD_card_datalogger,
                            "SD card data logging task",
                            7000,
                            &task_SD_card_datalogger_params,
                            5,
                            &task_handle_SD_card_datalogger,
                            0);

    while (1)
    {
        vTaskDelay((2000 / portTICK_PERIOD_MS));
    }
}

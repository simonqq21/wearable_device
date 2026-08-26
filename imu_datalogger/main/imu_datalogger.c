

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

#define MAIN_TAG "MAIN"

typedef struct
{
    QueueHandle_t queue_imu_data;
    // QueueHandle_t queue_sd_card_datalogger;
    StreamBufferHandle_t stream_buffer_imu;
    // QueueHandle_t queue_UART_datastreamer;
    // QueueHandle_t queue_BLE_datastreamer;
    StreamBufferHandle_t stream_buffer_sd;

} task_main_datalogging_params_t;

// task handles
// ****************************************************************
TaskHandle_t task_handle_status_led;
TaskHandle_t task_handle_imu_data;
TaskHandle_t task_handle_main_data_logging;
TaskHandle_t task_handle_SD_card_datalogger;

// queue for status LED states
// queue from IMU publisher to main datalogger subscriber
QueueHandle_t queue_imu_data;
// queue from main datalogger publisher to SD card datalogger subscriber
QueueHandle_t queue_sd_card_datalogger;
QueueHandle_t queue_UART_datastreamer;
QueueHandle_t queue_BLE_datastreamer;
StreamBufferHandle_t stream_buffer_imu, stream_buffer_sd;

// bool datalogging = false;

esp_err_t sd_card_configure_wrapper(void);

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

/**
 * @brief FreeRTOS task to get sensor data and distribute them to the SD card
 * datalogging task, UART data streaming task, and BLE data streaming task.
 *
 *
 */
#define DATA_LOGGING_TAG "DATA LOGGING"

static void task_main_datalogging(void *params)
{
    cmd_task_datalogging_t temp, datalogging_task_cmd, previous_datalogging_task_cmd;
    // uint8_t imu_data_received;
    // imu_data_t imu_data;
    task_main_datalogging_params_t *datalogging_params = (task_main_datalogging_params_t *)params;
    // QueueHandle_t queue_imu_data = datalogging_params->queue_imu_data;
    StreamBufferHandle_t stream_buffer_imu = datalogging_params->stream_buffer_imu;
    StreamBufferHandle_t stream_buffer_sd = datalogging_params->stream_buffer_sd;
    // QueueHandle_t queue_sd_card_datalogger = datalogging_params->queue_sd_card_datalogger;
    // QueueHandle_t queue_UART_datastreamer = datalogging_params->queue_UART_datastreamer;
    // QueueHandle_t queue_BLE_datastreamer = datalogging_params->queue_BLE_datastreamer;

    int num_bytes_read = 0;
    imu_data_t imu_data_buf[NUM_FIFO_TIMESTAMPS];

    previous_datalogging_task_cmd = CMD_DATALOGGING_STOP;
    datalogging_task_cmd = CMD_DATALOGGING_STOP;
    while (1)
    {
        // wait for task notification from button
        if (xTaskNotifyWait(0,
                            0,
                            (uint32_t *)&temp,
                            5 / portTICK_PERIOD_MS) != pdTRUE)
        {
            datalogging_task_cmd = temp;
        }

        switch (datalogging_task_cmd)
        {
        // go datalogging
        case CMD_DATALOGGING_GO:
            if (previous_datalogging_task_cmd == CMD_DATALOGGING_STOP)
            {
                previous_datalogging_task_cmd = CMD_DATALOGGING_GO;
                ESP_LOGI(DATA_LOGGING_TAG, "Datalogging started.");
            }

            // receive IMU data from queue
            // imu_data_received = xQueueReceive(queue_imu_data, (void *)&imu_data, 100 / portTICK_PERIOD_MS);
            num_bytes_read = xStreamBufferReceive(stream_buffer_imu,
                                                  &imu_data_buf,
                                                  sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS,
                                                  100 / portTICK_PERIOD_MS);

            // receive data from other sensors' queues

            if (num_bytes_read > 0)
            {
                // // send data to SD card datalogger queue
                // if (queue_sd_card_datalogger != NULL)
                //     xQueueSend(queue_sd_card_datalogger, &imu_data, 100 / );
                if (stream_buffer_sd != NULL)
                    if (xStreamBufferSend(stream_buffer_sd,
                                          &imu_data_buf,
                                          num_bytes_read,
                                          10 / portTICK_PERIOD_MS) == 0)
                    {
                        ESP_LOGE(IMU_TAG, "ERROR: Could not put item on SD stream buffer.");
                    }
                // // send data to UART data streaming queue
                // if (queue_UART_datastreamer != NULL)
                //     xQueueSend(queue_UART_datastreamer, &imu_data, 10 / );
                // // send data to BLE data streaming queue
                // if (queue_BLE_datastreamer != NULL)
                //     xQueueSend(queue_BLE_datastreamer, &imu_data, 10 / );

                // display IMU data
                // ESP_LOGI(DATA_LOGGING_TAG, "timestamp=%d", imu_data.timestamp);
                // ESP_LOGI(DATA_LOGGING_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_data.ax, imu_data.ay, imu_data.az);
                // ESP_LOGI(DATA_LOGGING_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_data.gx, imu_data.gy, imu_data.gz);
                // ESP_LOGI(DATA_LOGGING_TAG, "Temperature:  %.1f\n", imu_data.temp);
            }
            break;

        // stop datalogging
        case CMD_DATALOGGING_STOP:
            if (previous_datalogging_task_cmd == CMD_DATALOGGING_GO)
            {
                previous_datalogging_task_cmd = CMD_DATALOGGING_STOP;
                ESP_LOGI(DATA_LOGGING_TAG, "Datalogging stopped.");
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
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

    // initialize all queues
    queue_imu_data = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));
    queue_sd_card_datalogger = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));

    stream_buffer_imu = xStreamBufferCreate(sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS);
    stream_buffer_sd = xStreamBufferCreate(sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS);

    // create all primitives
    task_imu_params_t task_imu_params;
    task_main_datalogging_params_t task_main_datalogging_params;
    task_SD_card_datalogger_params_t task_SD_card_datalogger_params;

    // task_imu_params
    task_imu_params.task_handle_status_led = &task_handle_status_led;
    // task_imu_params.queue_imu_data = queue_imu_data;
    task_imu_params.stream_buffer_imu = stream_buffer_imu;
    task_imu_params.dev_handle = dev_handle;

    // task_main_datalogging_params
    task_main_datalogging_params.queue_imu_data = queue_imu_data;
    // task_main_datalogging_params.queue_sd_card_datalogger = queue_sd_card_datalogger;
    task_main_datalogging_params.stream_buffer_imu = stream_buffer_imu;
    task_main_datalogging_params.stream_buffer_sd = stream_buffer_sd;

    // task_SD_card_datalogger_params
    // task_SD_card_datalogger_params.queue_sd_card_datalogger = queue_sd_card_datalogger;
    task_SD_card_datalogger_params.stream_buffer_sd = stream_buffer_sd;

    // initialize all tasks
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

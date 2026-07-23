

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
    TaskHandle_t *task_handle_status_led;
    QueueHandle_t queue_imu_data;

} task_imu_params_t;

typedef struct
{
    QueueHandle_t queue_imu_data;
    QueueHandle_t queue_sd_card_datalogger;
    // QueueHandle_t queue_UART_datastreamer;
    // QueueHandle_t queue_BLE_datastreamer;

} task_main_datalogging_params_t;

typedef struct
{
    QueueHandle_t queue_sd_card_datalogger;
} task_SD_card_datalogger_params_t;

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

// bool datalogging = false;
cmd_task_datalogging_t datalogging_task_cmd, previous_datalogging_task_cmd;
cmd_imu_task_t imu_task_cmd, ongoing_imu_task_cmd;
cmd_task_sd_card_datalogging_t cmd_task_sd_card_datalogging, prev_cmd_task_sd_card_datalogging;
TickType_t imu_task_delay_period;

esp_err_t sd_card_configure_wrapper(void);

void set_datalogging_led(uint8_t led_value)
{
    gpio_set_level(LED1_PIN, led_value);
}

/**
 * @brief notification LED task
 *
 * The notification LED has the following states: STATUS_LED_OFF, STATUS_LED_ON, STATUS_LED_BLINK, and STATUS_LED_FAST_BLINK.
 * The notification LED state is set via a task notification.
 */
void task_notification_LED(void *pvParameters)
{
    cmd_task_status_led_t status_led_state;
    uint8_t led_value = 0;
    TickType_t loop_delay = 20 / portTICK_PERIOD_MS;

    while (1)
    {
        // xQueueReceive(status_led_queue, &status_led_state, loop_delay);
        // status_led_state = ulTaskNotifyTakeIndexed(0, pdFALSE, loop_delay);
        // use task notification as queue
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&status_led_state,
                        loop_delay);

        switch (status_led_state)
        {
        // notification LED blinks when datalogging is ongoing
        case STATUS_LED_BLINK:
            led_value = !led_value;
            loop_delay = 500 / portTICK_PERIOD_MS;
            break;
        // notification LED blinks fast when calibration is ongoing
        case STATUS_LED_FAST_BLINK:
            led_value = !led_value;
            loop_delay = 100 / portTICK_PERIOD_MS;
            break;
        //
        case STATUS_LED_ON:
            led_value = 1;
            loop_delay = 20 / portTICK_PERIOD_MS;
            break;
        // notification LED is off when datalogging is stopped
        default:
            led_value = 0;
            loop_delay = 20 / portTICK_PERIOD_MS;
            break;
        }
        set_datalogging_led(led_value);
        // ESP_LOGI("LED_module", "%d\n", status_led_state);
    }
}

/**
 * @brief IMU reading task
 *
 */
void task_imu(void *params)
{
    imu_data_t imu_data;
    task_imu_params_t *imu_task_params = (task_imu_params_t *)params;
    QueueHandle_t queue_imu_data = imu_task_params->queue_imu_data;
    TaskHandle_t *task_handle_status_led = imu_task_params->task_handle_status_led;

    while (1)
    {
        // variable delay
        if (ongoing_imu_task_cmd == IMU_READ_LOOP)
        {
            imu_task_delay_period = IMU_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS;
        }
        else
        {
            imu_task_delay_period = portMAX_DELAY;
        }

        // wait for task notification from button
        if (xTaskNotifyWait(0,
                            0,
                            (uint32_t *)&imu_task_cmd,
                            imu_task_delay_period) == pdTRUE)
        {
            ongoing_imu_task_cmd = imu_task_cmd;
        }

        switch (ongoing_imu_task_cmd)
        {
        // calibrate IMU
        case IMU_CALIBRATE:
            // notification LED fast blink
            if (*task_handle_status_led != NULL)
                xTaskNotify(*task_handle_status_led, STATUS_LED_FAST_BLINK, eSetValueWithOverwrite);
            // calibrate IMU
            imu_calibrate_user();
            // notification LED off
            if (*task_handle_status_led != NULL)
                xTaskNotify(*task_handle_status_led, STATUS_LED_OFF, eSetValueWithOverwrite);
            // stop IMU after calibration
            ongoing_imu_task_cmd = IMU_STOP;
            break;
        // measure IMU and send measurements to queue
        case IMU_READ_LOOP:
            imu_measure(&imu_data);
            imu_data.timestamp = esp_timer_get_time() / 1000;
            ESP_LOGI(IMU_TAG, "timestamp=%d", imu_data.timestamp);
            ESP_LOGI(IMU_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_data.ax, imu_data.ay, imu_data.az);
            ESP_LOGI(IMU_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_data.gx, imu_data.gy, imu_data.gz);
            ESP_LOGI(IMU_TAG, "Temperature:  %.1f\n", imu_data.temp);
            // send data to queue
            if (queue_imu_data != NULL)
                if (xQueueSend(queue_imu_data, &imu_data, IMU_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS) != pdTRUE)
                {
                    ESP_LOGE(IMU_TAG, "ERROR: Could not put item on IMU queue.");
                }
            break;
        default: // IMU_STOP
            // check IMU task task notification queue
            break;
        }
        // vTaskDelay(IMU_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief FreeRTOS task to get sensor data and distribute them to the SD card
 * datalogging task, UART data streaming task, and BLE data streaming task.
 *
 *
 */
#define DATA_LOGGING_TAG "DATA LOGGING"
static void task_main_datalogging(void *params)
{
    uint8_t imu_data_received;
    imu_data_t imu_data;
    task_main_datalogging_params_t *datalogging_params = (task_main_datalogging_params_t *)params;
    QueueHandle_t queue_imu_data = datalogging_params->queue_imu_data;
    QueueHandle_t queue_sd_card_datalogger = datalogging_params->queue_sd_card_datalogger;
    // QueueHandle_t queue_UART_datastreamer = datalogging_params->queue_UART_datastreamer;
    // QueueHandle_t queue_BLE_datastreamer = datalogging_params->queue_BLE_datastreamer;

    while (1)
    {
        // wait for task notification from button
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&datalogging_task_cmd,
                        100 / portTICK_PERIOD_MS);

        switch (datalogging_task_cmd)
        {
        // go datalogging
        case DATALOGGING_GO:
            if (previous_datalogging_task_cmd == DATALOGGING_STOP)
            {
                previous_datalogging_task_cmd = DATALOGGING_GO;
                ESP_LOGI(DATA_LOGGING_TAG, "Datalogging started.");
            }

            // receive IMU data from queue
            imu_data_received = xQueueReceive(queue_imu_data, (void *)&imu_data, 100 / portTICK_PERIOD_MS);

            // receive data from other sensors' queues

            if (imu_data_received)
            {
                // send data to SD card datalogger queue
                if (queue_sd_card_datalogger != NULL)
                    xQueueSend(queue_sd_card_datalogger, &imu_data, 100 / portTICK_PERIOD_MS);
                // send data to UART data streaming queue
                if (queue_UART_datastreamer != NULL)
                    xQueueSend(queue_UART_datastreamer, &imu_data, 100 / portTICK_PERIOD_MS);
                // send data to BLE data streaming queue
                if (queue_BLE_datastreamer != NULL)
                    xQueueSend(queue_BLE_datastreamer, &imu_data, 100 / portTICK_PERIOD_MS);

                // display IMU data
                ESP_LOGI(DATA_LOGGING_TAG, "timestamp=%d", imu_data.timestamp);
                ESP_LOGI(DATA_LOGGING_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_data.ax, imu_data.ay, imu_data.az);
                ESP_LOGI(DATA_LOGGING_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_data.gx, imu_data.gy, imu_data.gz);
                ESP_LOGI(DATA_LOGGING_TAG, "Temperature:  %.1f\n", imu_data.temp);
            }
            break;

        // stop datalogging
        case DATALOGGING_STOP:
            if (previous_datalogging_task_cmd == DATALOGGING_GO)
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

FILE *close_file(FILE *f, unsigned long *file_size)
{
    *file_size = 0;
    if (REQUIRE_SD_CARD)
    {
        if (f != NULL)
        {
            fclose(f);
        }
    }
    f = NULL;
    return f;
}

FILE *open_file(FILE *f, char *filepath, unsigned long *file_size)
{
    *file_size = 0;
    if (REQUIRE_SD_CARD)
    {
        f = fopen(filepath, "wb");

        if (f == NULL)
        {
            ESP_LOGE(SD_CARD_TAG, "Failed to open file for writing");
        }
        else
        {
            ESP_LOGI(SD_CARD_TAG, "FILE IS NOT NULL");
        }
    }
    return f;
}

void write_imu_data_to_card(FILE *f, imu_data_t *rcv_imu_data, unsigned long *file_size)
{
    if (REQUIRE_SD_CARD)
    {
        if (f != NULL)
        {
            fwrite(rcv_imu_data, sizeof(imu_data_t), 1, f);
            *file_size = ftell(f);
        }
        else
            ESP_LOGE(SD_CARD_TAG, "file is null");
    }
}

/**
 * @brief SD card datalogging subscriber task
 *
 * Grab data sent from the IMU datalogging publisher task to log to the SD card.
 * Create and open new binary file with given index
 */
static void task_SD_card_datalogger(void *params)
{
    task_SD_card_datalogger_params_t *sd_datalogger_params = (task_SD_card_datalogger_params_t *)params;
    QueueHandle_t queue_sd_card_datalogger = sd_datalogger_params->queue_sd_card_datalogger;

    char filename[20];
    char filepath[30];
    FILE *f = NULL;
    // struct stat st;
    int max_idx;
    unsigned long file_size = 0;
    imu_data_t rcv_imu_data;

    // get index of latest datalog file
    // max_idx = get_latest_datalog_idx(MOUNT_POINT);
    // if (max_idx < 0)
    // {
    //     ESP_LOGI(SD_CARD_TAG, "no previous IMU datalogs.");
    // }
    // else
    // {
    //     ESP_LOGI(SD_CARD_TAG, "Latest IMU datalog has index %d.", max_idx);
    // }

    while (1)
    {
        // check for the signal to stop datalogging, which will save the file.
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&cmd_task_sd_card_datalogging, 40 / portTICK_PERIOD_MS);

        /*
         run once when stop signal received

         if stop signal received, save and close the file.
        */
        if (cmd_task_sd_card_datalogging == SD_CARD_STOP && prev_cmd_task_sd_card_datalogging == SD_CARD_START)
        {
            f = close_file(f, &file_size);
            ESP_LOGI(SD_CARD_TAG, "File written.");
            ESP_LOGI(SD_CARD_TAG, "file size: %d", file_size);
        }

        /*
        run once when start signal received

        if start signal received, open a new file..
        */
        else if (cmd_task_sd_card_datalogging == SD_CARD_START && prev_cmd_task_sd_card_datalogging == SD_CARD_STOP)
        {

            // if no file is open, create a new file.
            if (f == NULL)
            { // if SD card not initialized, attempt to mount it.
                if (!sd_card_is_initialized())
                {
                    sd_card_configure_wrapper();
                }
                file_size = 0;
                // get the latest datalog file index and add one to it for the new datalog file
                max_idx = get_latest_datalog_idx(MOUNT_POINT);
                max_idx++;
                // create filename IMU_FILENAME_FORMAT
                sprintf(filename, "imu_%06d.bin", max_idx);
                ESP_LOGI(SD_CARD_TAG, "filename: %s", filename);
                // create filepath
                sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                ESP_LOGI(SD_CARD_TAG, "filepath: %s", filepath);
                // create and write new binary file
                ESP_LOGI(SD_CARD_TAG, "Writing to new file");
                f = open_file(f, filepath, &file_size);
                if (f == NULL)
                {
                    ESP_LOGE(SD_CARD_TAG, "FILE IS NULL");
                }
                else
                {
                    ESP_LOGI(SD_CARD_TAG, "FILE IS NOT NULL");
                }
            }
        }

        /*
        run continuously while datalogging is ongoing


        */
        else if (cmd_task_sd_card_datalogging == SD_CARD_START && prev_cmd_task_sd_card_datalogging == SD_CARD_START)
        {
            // while the file is less than the maximum datalog file size
            if (file_size < MAX_DATALOG_FILE_SIZE)
            {
                // if no file is open, create a new file.
                if (f == NULL)
                { // if SD card not initialized, attempt to mount it.
                    if (!sd_card_is_initialized())
                    {
                        sd_card_configure_wrapper();
                    }
                    file_size = 0;
                    // get the latest datalog file index and add one to it for the new datalog file
                    max_idx = get_latest_datalog_idx(MOUNT_POINT);
                    max_idx++;
                    // create filename IMU_FILENAME_FORMAT
                    sprintf(filename, "imu_%06d.bin", max_idx);
                    ESP_LOGI(SD_CARD_TAG, "filename: %s", filename);
                    // create filepath
                    sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                    ESP_LOGI(SD_CARD_TAG, "filepath: %s", filepath);
                    // create and write new binary file
                    ESP_LOGI(SD_CARD_TAG, "Writing to new file");
                    f = open_file(f, filepath, &file_size);
                    if (f == NULL)
                    {
                        ESP_LOGE(SD_CARD_TAG, "FILE IS NULL");
                    }
                    else
                    {
                        ESP_LOGI(SD_CARD_TAG, "FILE IS NOT NULL");
                    }
                }

                // read from the queue.
                if (xQueueReceive(queue_sd_card_datalogger, (void *)&rcv_imu_data, 10 / portTICK_PERIOD_MS) == pdTRUE) // portMAX_DELAY
                {
                    ESP_LOGI(SD_CARD_TAG, "received data: ");
                    ESP_LOGI(SD_CARD_TAG, "Accel %.4f, %.4f, %.4f", rcv_imu_data.ax, rcv_imu_data.ay, rcv_imu_data.az);
                    ESP_LOGI(SD_CARD_TAG, "Gyro %.4f, %.4f, %.4f", rcv_imu_data.gx, rcv_imu_data.gy, rcv_imu_data.gz);
                    ESP_LOGI(SD_CARD_TAG, "Temp %.2f", rcv_imu_data.temp);

                    write_imu_data_to_card(f, &rcv_imu_data, &file_size);
                }
            }
            else
            { // once the file reaches the maximum datalog file size
                f = close_file(f, &file_size);
                ESP_LOGI(SD_CARD_TAG, "File written.");
                ESP_LOGI(SD_CARD_TAG, "file size: %d", file_size);
            }
        }

        /*
        run continuously while datalogging is stopped

        do nothing
        */
        else if (cmd_task_sd_card_datalogging == SD_CARD_STOP && prev_cmd_task_sd_card_datalogging == SD_CARD_STOP)
        {
        }

        prev_cmd_task_sd_card_datalogging = cmd_task_sd_card_datalogging;
    }
}

esp_err_t sd_card_configure_wrapper(void)
{
    esp_err_t ret = ESP_OK;
    if (REQUIRE_SD_CARD && !sd_card_is_initialized())
    {
        // for (int i = 0; i < 5; i++)
        // {
        ret = sd_configure();
        if (ret != ESP_OK)
        {
            ESP_LOGE(MAIN_TAG, "SD card initialization failed");
        }
        else
        {
            ESP_LOGI(MAIN_TAG, "SD card initialized");
            return ret;
            // break;
        }
        // }
    }
    return ret;
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
    imu_set_offset_read_cb(NVS_read_imu_calibration_offsets);
    imu_set_offset_write_cb(NVS_write_imu_calibration_offsets);
    imu_init_custom(false);
    // initialize littleFS

    /*
    initialize SD card
    The SD card is an integral part to datalogging.
    If the card doesn't initialize, attempt to initialize n times in a loop.
    */
    sd_card_configure_wrapper();
    ESP_LOGI(MAIN_TAG, "Starting all tasks!");

    // initialize all queues
    queue_imu_data = xQueueCreate(IMU_QUEUE_LEN, sizeof(imu_data_t));
    queue_sd_card_datalogger = xQueueCreate(IMU_QUEUE_LEN, sizeof(imu_data_t));

    // create all primitives
    task_imu_params_t task_imu_params;
    task_main_datalogging_params_t task_main_datalogging_params;
    task_SD_card_datalogger_params_t task_SD_card_datalogger_params;
    // task_imu_params
    task_imu_params.task_handle_status_led = &task_handle_status_led;
    task_imu_params.queue_imu_data = queue_imu_data;

    // task_main_datalogging_params
    task_main_datalogging_params.queue_imu_data = queue_imu_data;
    task_main_datalogging_params.queue_sd_card_datalogger = queue_sd_card_datalogger;
    // task_SD_card_datalogger_params
    task_SD_card_datalogger_params.queue_sd_card_datalogger = queue_sd_card_datalogger;

    // initialize all tasks
    // IMU reading task
    xTaskCreatePinnedToCore(task_imu,
                            "imu calibrate and read task",
                            2048,
                            &task_imu_params,
                            1,
                            &task_handle_imu_data,
                            0);

    // sensor data logging task
    xTaskCreatePinnedToCore(task_main_datalogging,
                            "main data logging task",
                            2048,
                            &task_main_datalogging_params,
                            1,
                            &task_handle_main_data_logging,
                            0);

    // notification LED task
    xTaskCreatePinnedToCore(task_notification_LED,
                            "notification LED task",
                            1024,
                            NULL,
                            5,
                            &task_handle_status_led,
                            0);

    // micro SD card datalogging task
    xTaskCreatePinnedToCore(task_SD_card_datalogger,
                            "SD card data logging task",
                            4096,
                            &task_SD_card_datalogger_params,
                            5,
                            &task_handle_SD_card_datalogger,
                            0);

    // button task
    while (1)
    {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

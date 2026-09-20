
#include "include/uart.h"
#include "include/nvs.h"
#include "include/led_module.h"
#include "include/button_module.h"
#include "include/imu.h"
#include "include/sd_card.h"
#include "include/uart.h"
#include "include/ble.h"
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
TaskHandle_t task_handle_uart;
TaskHandle_t task_handle_ble;

/* queue for IMU data coming from IMU task to datalogging task */
QueueHandle_t queue_imu;
/* queues for raw IMU data from datalogging task to the SD card, UART, and BLE */
QueueHandle_t queue_raw_sdcard;
/* queue for orientation from datalogging task to the SD card, UART, and BLE */
QueueHandle_t queue_orientation_sdcard, queue_orientation_UART, queue_orientation_BLE;
/* queue for heart rate values */
QueueHandle_t queue_heart_rate_UART, queue_heart_rate_BLE;
/* queue for haptics command play builtin sequence */
QueueHandle_t queue_haptics_command_play_builtin_BLE;
/* queue for haptics command play custom sequence */
QueueHandle_t queue_haptics_command_play_custom_BLE;

/* shared variables */
haptics_status_t haptics_status;
float battery_voltage;

bool datalogging = false;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

// ****************************************************************

/* create all parameters */
params_task_imu_t params_task_imu;
params_task_main_datalogging_t params_task_main_datalogging;
params_task_SD_card_datalogger_t params_task_SD_card_datalogger;
params_task_uart_t params_task_uart;
params_task_ble_t params_task_ble;

// static void timer_callback(void *arg);

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

    /* initialize BT */
    ble_configure();

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

    /* initialize haptics */
    // haptics_init(&haptics_hw,
    //              EN_PIN,
    //              haptics_channels);
    // haptics_disable_all(&haptics_hw);

    // // Define timer configuration
    // const esp_timer_create_args_t timer_args = {
    //     .callback = &timer_callback,
    //     .name = "asdf timer",
    // };

    // esp_timer_handle_t timer_handle;
    // ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));

    // // Start periodic timer with a 1 ms period (1000 microseconds)
    // ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 1000));

    // // ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 500000));

    // haptics_load_pulse_seq(&haptics_hw, pulse_null, 1);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    /* create all queues */
    queue_imu = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_data_t));
    queue_raw_sdcard = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(imu_data_t));
    queue_orientation_sdcard = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(orientation_data_t));
    queue_orientation_UART = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(orientation_data_t));
    queue_orientation_BLE = xQueueCreate(NUM_FIFO_TIMESTAMPS, sizeof(orientation_data_t));
    ESP_LOGI(MAIN_TAG, "orientation queue pointer = %p", queue_orientation_BLE);
    ESP_LOGI(MAIN_TAG, "orientation queue pointer = %p", &queue_orientation_BLE);

    ESP_LOGI(MAIN_TAG, "params_task_uart = %p", &params_task_uart);
    ESP_LOGI(MAIN_TAG, "params_task_ble = %p", &params_task_ble);

    /* params_task_imu */
    params_task_imu.task_handle_status_led = &task_handle_status_led;
    params_task_imu.queue_imu = queue_imu;
    // params_task_imu.streambuffer_imu = streambuffer_imu;
    params_task_imu.dev_handle = dev_handle;

    /* params_task_main_datalogging */
    params_task_main_datalogging.queue_imu = queue_imu;
    // params_task_main_datalogging.streambuffer_imu = streambuffer_imu;
    params_task_main_datalogging.queue_raw_sdcard = queue_raw_sdcard;
    params_task_main_datalogging.queue_orientation_sdcard = queue_orientation_sdcard;
    params_task_main_datalogging.queue_orientation_UART = queue_orientation_UART;
    params_task_main_datalogging.queue_orientation_BLE = queue_orientation_BLE;
    // params_task_main_datalogging.streambuffer_sd = streambuffer_sd;

    /* params_task_SD_card_datalogger */
    params_task_SD_card_datalogger.queue_raw_sdcard = queue_raw_sdcard;
    params_task_SD_card_datalogger.queue_orientation_sdcard = queue_orientation_sdcard;
    // params_task_SD_card_datalogger.streambuffer_sd = streambuffer_sd;

    /* params_task_uart */
    params_task_uart.queue_orientation_UART = queue_orientation_UART;
    params_task_uart.queue_heart_rate_UART = queue_heart_rate_UART;

    /* params_task_ble */
    params_task_ble.queue_orientation_BLE = queue_orientation_BLE;
    // params_task_ble.queue_heart_rate_BLE = queue_heart_rate_BLE;
    // params_task_ble.queue_haptics_command_play_builtin_BLE =
    //     queue_haptics_command_play_builtin_BLE;
    // params_task_ble.queue_haptics_command_play_custom_BLE =
    //     queue_haptics_command_play_custom_BLE;
    // // params_task_ble.haptics_command_config = ;
    // params_task_ble.haptics_status = &haptics_status;
    // params_task_ble.battery_voltage = &battery_voltage;

    /* initialize all tasks */
    // IMU reading task
    xTaskCreatePinnedToCore(task_imu,
                            "imu calibrate and read task",
                            2048,
                            &params_task_imu,
                            2,
                            &task_handle_imu_data,
                            0);

    // sensor data logging task
    xTaskCreatePinnedToCore(task_main_datalogging,
                            "main data logging task",
                            8192,
                            &params_task_main_datalogging,
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
                            10000, //
                            &params_task_SD_card_datalogger,
                            5,
                            &task_handle_SD_card_datalogger,
                            0);
    /* UART streaming task */
    xTaskCreatePinnedToCore(task_uart_streaming,
                            "UART data streaming task",
                            3072,
                            &params_task_uart,
                            3,
                            &task_handle_uart,
                            0);

    /* BLE streaming task */
    xTaskCreatePinnedToCore(task_ble_streaming,
                            "BLE data streaming task",
                            10000,
                            &params_task_ble,
                            3,
                            &task_handle_ble,
                            0);
    while (1)
    {
        vTaskDelay((2000 / portTICK_PERIOD_MS));
    }
}

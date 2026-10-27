
#include "uart.h"
#include "nvs.h"
#include "gpio_module.h"
#include "imu.h"
#include "heart_rate.h"
#include "sd_card.h"
#include "uart.h"
#include "ble.h"
#include "haptics_lib.h"

#include "TaskBLE.h"
#include "TaskHaptics.h"
#include "TaskSDCard.h"
#include "TaskSensors.h"
#include "TaskSensorFusion.h"
#include "TaskUART.h"
#include "TaskGPIO.h"

#include "common.h"
#include "config.h"
#include "datalogging.h"

#define MAIN_TAG "MAIN"

// task handles
TaskHandle_t task_handle_GPIO;
TaskHandle_t task_handle_sensors;
TaskHandle_t task_handle_sensor_fusion;
TaskHandle_t task_handle_SD_card_datalogger;
TaskHandle_t task_handle_uart;
TaskHandle_t task_handle_ble;
TaskHandle_t task_handle_haptics;

/* queue for IMU data from IMU */
QueueHandle_t queue_imu_sensor;
/* queue for heart rate data from heart rate sensor */
QueueHandle_t queue_heart_rate_sensor;
/* Queue for IMU data to SD card task */
QueueHandle_t queue_imu_sdcard;
/* Queue for heart rate data to SD card task */
QueueHandle_t queue_heart_rate_sdcard;
/* Queue for IMU data to UART task */
QueueHandle_t queue_imu_UART;
/* Queue for heart rate data to UART task */
QueueHandle_t queue_heart_rate_UART;
/* Queue for IMU data to BLE task */
QueueHandle_t queue_imu_BLE;
/* Queue for heart rate data to BLE task */
QueueHandle_t queue_heart_rate_BLE;

/* queues for raw IMU data from datalogging task to the SD card, UART, and BLE */
QueueHandle_t queue_raw_sdcard;
/* queue for orientation from datalogging task to the SD card, UART, and BLE */
QueueHandle_t queue_orientation_sdcard, queue_orientation_UART, queue_orientation_BLE;
/* queue for heart rate values */
QueueHandle_t queue_heart_rate_UART, queue_heart_rate_BLE;
/* queue for haptics command play builtin sequence */
QueueHandle_t queue_haptics_BLE;
// /* queue for haptics command play custom sequence */
// QueueHandle_t queue_haptics_command_play_custom_BLE;

/* shared variables */
haptics_status_t haptics_status;
float battery_voltage;

bool datalogging = false;

/**
 * hardware handles
 */
/* button handles */
button_handle_t btn1;
/* i2c bus and device handles */
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t imu_dev_handle;
i2c_MAX30102_handle_t heart_rate_dev_handle;

/* haptics hw struct */
haptic_actuator_array_t haptics_hw;
haptic_channel_pins_t haptics_channels[4] = {
    {CHANNEL_0_PIN_1, CHANNEL_0_PIN_2},
    {CHANNEL_1_PIN_1, CHANNEL_1_PIN_2},
    {CHANNEL_2_PIN_1, CHANNEL_2_PIN_2},
    {CHANNEL_3_PIN_1, CHANNEL_3_PIN_2}};

// ****************************************************************

/* create all task parameter structs */
params_task_sensors_t params_task_sensors;
params_task_sensor_fusion_t params_task_sensor_fusion;
params_task_SD_card_datalogger_t params_task_SD_card_datalogger;
params_task_uart_t params_task_uart;
params_task_ble_t params_task_ble;
params_task_haptics_t params_task_haptics;

// Callback function executed when the timer fires
static void haptics_timer_callback(void *arg)
{
    // i = (i + 1) % 4;
    // haptics_play_pulse(&haptics_hw, &pulse_seq_1[i]);
    haptics_ISR_callback(&haptics_hw);
}

// main
imu_raw_data_t data;
void app_main(void)
{
    // initialize GPIO (simple inputs and outputs)
    gpio_init();

    // initialize NVS
    nvs_init();

    /* initialize the i2c bus */
    i2c_master_init(&bus_handle);
    // i2c_master_init(&bus_handle, &heart)
    ESP_LOGI(MAIN_TAG, "I2C initialized successfully");

    /* initialize IMU */
    i2c_master_dev_handle_t imu_dev_handle;
    i2c_master_add_device(&bus_handle,
                          &imu_dev_handle,
                          LSM6DS3_SENSOR_ADDR);
    imu_set_offset_read_cb(NVS_read_imu_calibration_offsets);
    imu_set_offset_write_cb(NVS_write_imu_calibration_offsets);
    imu_init(imu_dev_handle, IMU_ODR_HZ, IMU_XL_FS, IMU_G_FS, false);

    /* initialize heart rate sensor */
    i2c_MAX30102_config_t MAX30102_config = {
        .MAX30102_device = {
            .device_address = MAX30102_DEVICE_ADDR,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        }};
    i2c_MAX30102_init(bus_handle,
                      &MAX30102_config,
                      &heart_rate_dev_handle);
    uint8_t part_id;

    esp_err_t heart_rate_status = i2c_MAX30102_read_reg(heart_rate_dev_handle,
                                                        REG_PART_ID,
                                                        &part_id,
                                                        1);

    // initialize UART
    uart_configure();

    /* initialize BT */
    ble_configure();

    /*
initialize SD card
The SD card is an integral part to datalogging.
If the card doesn't initialize, attempt to initialize n times in a loop.
*/
    sd_card_configure_wrapper();
    ESP_LOGI(MAIN_TAG, "MAX30102 part ID = %d %x %X\n", heart_rate_status, part_id, part_id);
    ESP_LOGI(MAIN_TAG, "Starting all tasks!");

    /* initialize haptics */
    haptics_init(&haptics_hw,
                 EN_PIN,
                 haptics_channels);
    haptics_disable_all(&haptics_hw);

    // Define periodic timer configuration
    const esp_timer_create_args_t haptics_timer_args = {
        .callback = &haptics_timer_callback,
        .name = "haptics player timer",
    };
    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&haptics_timer_args, &timer_handle));
    // Start periodic timer with a 1 ms period (1000 microseconds)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 1000));

    /* create all queues */
    queue_imu_sensor = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_raw_data_t));
    queue_imu_sdcard = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_unified_data_t));
    queue_imu_UART = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_unified_data_t));
    queue_imu_BLE = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(imu_unified_data_t));

    queue_heart_rate_sensor = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(hr_ppg_msg_t));
    queue_heart_rate_sdcard = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(int));
    queue_heart_rate_UART = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(int));
    queue_heart_rate_BLE = xQueueCreate(NUM_FIFO_TIMESTAMPS * 2, sizeof(int));

    queue_haptics_BLE = xQueueCreate(LEN_QUEUE_HAPTICS, sizeof(uint8_t));

    // ESP_LOGI(MAIN_TAG, "orientation queue pointer = %p", queue_orientation_BLE);
    // ESP_LOGI(MAIN_TAG, "orientation queue pointer = %p", &queue_orientation_BLE);

    // ESP_LOGI(MAIN_TAG, "params_task_uart = %p", &params_task_uart);
    // ESP_LOGI(MAIN_TAG, "params_task_ble = %p", &params_task_ble);

    /* params GPIO */

    /* set sensors task parameters */
    params_task_sensors = (params_task_sensors_t){
        .queue_imu_sensor = queue_imu_sensor,
        .queue_heart_rate_sensor = queue_heart_rate_sensor,
        .imu_dev_handle = imu_dev_handle,
        .heart_rate_dev_handle = heart_rate_dev_handle->i2c_dev};

    /* set sensor fusion task parameters */
    params_task_sensor_fusion.queue_imu_sensor = queue_imu_sensor;
    params_task_sensor_fusion.queue_heart_rate_sensor = queue_heart_rate_sensor;
    params_task_sensor_fusion.queue_imu_sdcard = queue_imu_sdcard;
    params_task_sensor_fusion.queue_imu_UART = queue_imu_UART;
    params_task_sensor_fusion.queue_imu_BLE = queue_imu_BLE;
    params_task_sensor_fusion.queue_heart_rate_sdcard = queue_heart_rate_sdcard;
    params_task_sensor_fusion.queue_heart_rate_UART = queue_heart_rate_UART;
    params_task_sensor_fusion.queue_heart_rate_BLE = queue_heart_rate_BLE;

    /* set SD card task parameters */
    params_task_SD_card_datalogger.queue_imu_sdcard = queue_imu_sdcard;
    params_task_SD_card_datalogger.queue_heart_rate_sdcard = queue_heart_rate_sdcard;

    /* Set UART task parameters */
    params_task_uart.queue_imu_UART = queue_imu_UART;
    params_task_uart.queue_heart_rate_UART = queue_heart_rate_UART;

    /* Set BLE task parameters */
    params_task_ble.queue_imu_BLE = queue_imu_BLE;
    params_task_ble.queue_heart_rate_BLE = queue_heart_rate_BLE;
    params_task_ble.queue_haptics_BLE = queue_haptics_BLE;
    // params_task_ble.queue_haptics_command_play_custom_BLE =
    // queue_haptics_command_play_custom_BLE;
    // // params_task_ble.haptics_command_config = ;
    // params_task_ble.haptics_status = &haptics_status;
    // params_task_ble.battery_voltage = &battery_voltage;

    /* params_task_haptics */
    params_task_haptics.queue_haptics_BLE = queue_haptics_BLE;
    params_task_haptics.haptics_hw = &haptics_hw;

    /* initialize all tasks */
    // GPIO task
    xTaskCreatePinnedToCore(task_GPIO,
                            "GPIO task",
                            1024,
                            NULL,
                            10,
                            &task_handle_GPIO,
                            0);

    // i2c sensors (IMU and HR) reading task
    xTaskCreatePinnedToCore(task_sensors,
                            "i2c sensors task",
                            2048,
                            &params_task_sensors,
                            2,
                            &task_handle_sensors,
                            0);

    // sensor data logging task
    xTaskCreatePinnedToCore(task_sensor_fusion,
                            "sensor fusion task",
                            8192,
                            &params_task_sensor_fusion,
                            3,
                            &task_handle_sensor_fusion,
                            0);

    // micro SD card datalogging task
    xTaskCreatePinnedToCore(task_SD_card_datalogger,
                            "SD card datalogging task",
                            10000, //
                            &params_task_SD_card_datalogger,
                            5,
                            &task_handle_SD_card_datalogger,
                            0);

    /* UART task */
    xTaskCreatePinnedToCore(task_uart_streaming,
                            "UART data streaming task",
                            3072,
                            &params_task_uart,
                            3,
                            &task_handle_uart,
                            0);

    /* BLE task */
    xTaskCreatePinnedToCore(task_ble_streaming,
                            "BLE data streaming task",
                            10000,
                            &params_task_ble,
                            3,
                            &task_handle_ble,
                            0);
    /* haptics task */
    xTaskCreatePinnedToCore(task_haptics,
                            "haptic actuators task",
                            2048,
                            &params_task_haptics,
                            1,
                            &task_handle_haptics,
                            0);

    while (1)
    {
        vTaskDelay((2000 / portTICK_PERIOD_MS));
    }
}

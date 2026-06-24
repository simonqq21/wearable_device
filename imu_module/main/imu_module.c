#include <stdio.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"

#include <esp_err.h>
#include <esp_log.h>

#include <mpu6050.h>

#include "esp_flash.h"
#include "esp_chip_info.h"
#include "spi_flash_mmap.h"
#include "esp_littlefs.h"

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "button_gpio.h"

#define IMU_TAG "IMU"
#define NVS_TAG "NVS"

// i2c
// ****************************************************************
#define SDA_PIN 21
#define SCL_PIN 22
// ****************************************************************

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

// NVS
// ****************************************************************
#define IMU_OFFSETS_NVS_KEY "imu_offsets"
#define NVS_NAMESPACE "NVS"
typedef struct
{
    int16_t oAx, oAy, oAz;
    int16_t oGx, oGy, oGz;
} imu_calibration_offsets_t;

void nvs_init(void)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// write IMU calibration offsets to NVS
esp_err_t NVS_write_imu_calibration_offsets(imu_calibration_offsets_t *offsets)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open NVS handle
    ESP_LOGI(NVS_TAG, "\nOpening Non-Volatile Storage (NVS) handle...");
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NVS_TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    // Write blob
    ESP_LOGI(NVS_TAG, "write IMU calibration offsets to NVS...");
    err = nvs_set_blob(my_handle, IMU_OFFSETS_NVS_KEY, &offsets, sizeof(imu_calibration_offsets_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(NVS_TAG, "Failed to write IMU calibration offsets to NVS!");
        nvs_close(my_handle);
        return err;
    }

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(NVS_TAG, "Failed to commit data");
    }

    nvs_close(my_handle);
    return err;
}

// read IMU calibration offsets from NVS
esp_err_t NVS_read_imu_calibration_offsets(imu_calibration_offsets_t *offsets)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = 0;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK)
        return err;

    // 1. Read test data blob
    ESP_LOGI(NVS_TAG, "IMU calibration offsets from blob:");
    err = nvs_get_blob(my_handle, IMU_OFFSETS_NVS_KEY, offsets, &required_size);
    if (err == ESP_OK)
    {
        ESP_LOGI(NVS_TAG, "accel_offsets_x = %.2f", offsets->oAx);
        ESP_LOGI(NVS_TAG, "accel_offsets_y = %.2f", offsets->oAy);
        ESP_LOGI(NVS_TAG, "accel_offsets_z = %.2f", offsets->oAz);
        ESP_LOGI(NVS_TAG, "gyro_offsets_x = %.2f", offsets->oGx);
        ESP_LOGI(NVS_TAG, "gyro_offsets_y = %.2f", offsets->oGy);
        ESP_LOGI(NVS_TAG, "gyro_offsets_z = %.2f", offsets->oGz);
    }
    else if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(NVS_TAG, "Test data not found!");
    }
    nvs_close(my_handle);
    return ESP_OK;
}

// ****************************************************************

// littleFS
// ****************************************************************

// ****************************************************************

// MPU6050
// ****************************************************************
#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

mpu6050_dev_t dev = {0};

// self-test MPU6050

// initialize and configure MPU6050
void mpu6050_init_custom(void)
{
    uint8_t mpu6050_id = 0;
    imu_calibration_offsets_t offsets;
    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, SDA_PIN, SCL_PIN));
    // search for MPU6050 on i2c bus
    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(IMU_TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(IMU_TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // initialize and wake up MPU6050
    ESP_ERROR_CHECK(mpu6050_init(&dev));
    // get MPU6050 WHOAMI
    ESP_ERROR_CHECK(mpu6050_get_device_id(&dev, &mpu6050_id));
    ESP_LOGI(IMU_TAG, "MPU6050 ID: %d", mpu6050_id);
    // set IMU DLPF
    ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_2));
    // set IMU DHPF
    ESP_ERROR_CHECK(mpu6050_set_dhpf_mode(&dev, MPU6050_DHPF_0_63));
    // set IMU sample rate
    ESP_ERROR_CHECK(mpu6050_set_rate(&dev, 0));
    // set IMU accel and gyro ranges
    ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_2));
    ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_250));
    // set IMU clock source
    ESP_ERROR_CHECK(mpu6050_set_clock_source(&dev, MPU6050_CLOCK_PLL_Y));
    // enable MPU6050 FIFO
    mpu6050_set_fifo_enabled(&dev, true);
    // ESP_LOGI(IMU_TAG, "Accel range: %d", dev.ranges.accel);
    // ESP_LOGI(IMU_TAG, "Gyro range:  %d", dev.ranges.gyro);
    uint8_t calibrate_imu = false;
    if (calibrate_imu)
    {
        float accel_bias[3], gyro_bias[3];
        // calibrate IMU
        mpu6050_calibrate(&dev, accel_bias, gyro_bias);
        // read accel and gyro calibration offsets from IMU
        mpu6050_get_accel_offset(&dev, MPU6050_X_AXIS, &offsets.oAx);
        mpu6050_get_accel_offset(&dev, MPU6050_Y_AXIS, &offsets.oAy);
        mpu6050_get_accel_offset(&dev, MPU6050_Z_AXIS, &offsets.oAz);
        mpu6050_get_gyro_offset(&dev, MPU6050_X_AXIS, &offsets.oGx);
        mpu6050_get_gyro_offset(&dev, MPU6050_X_AXIS, &offsets.oGy);
        mpu6050_get_gyro_offset(&dev, MPU6050_X_AXIS, &offsets.oGz);
        // write accel and gyro calibration offsets to NVS
        NVS_write_imu_calibration_offsets(&offsets);
    }
    else
    {
        // read accel and gyro calibration offsets from NVS
        NVS_read_imu_calibration_offsets(&offsets);
        // write accel and gyro calibration offsets to IMU
        mpu6050_set_accel_offset(&dev, MPU6050_X_AXIS, offsets.oAx);
        mpu6050_set_accel_offset(&dev, MPU6050_Y_AXIS, offsets.oAy);
        mpu6050_set_accel_offset(&dev, MPU6050_Z_AXIS, offsets.oAz);
        mpu6050_set_gyro_offset(&dev, MPU6050_X_AXIS, offsets.oGx);
        mpu6050_set_gyro_offset(&dev, MPU6050_Y_AXIS, offsets.oGy);
        mpu6050_set_gyro_offset(&dev, MPU6050_Z_AXIS, offsets.oGz);
    }
}

typedef struct
{
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;
    float temp;
} mpu6050_data;

// log MPU6050 data
void mpu6050_log(mpu6050_data *data)
{
    ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &data->temp));
    ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &data->accel, &data->rotation));
}

// ****************************************************************

// UART
// ****************************************************************
// configure UART
#define UART_PORT_NUM 0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 128
static void uart_configure(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    /**
     * If you want to use USB, set TXD to 1 and RXD to 3.
     */
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, 1, 3, 0, 0));
}
// ****************************************************************

//
void app_main(void)
{
    // initialize GPIO

    // initialize i2c
    ESP_ERROR_CHECK(i2cdev_init());

    // initialize UART

    // initialize MPU6050

    // initialize NVS

    // initialize littleFS

    // MPU6050 self-test and calibration

    // data logging task

    while (1)
    {
    }
}

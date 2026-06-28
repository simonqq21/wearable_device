#include "../include/imu.h"

// MPU6050
// ****************************************************************
// #ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
// #else
// #define ADDR MPU6050_I2C_ADDRESS_HIGH
// #endif

mpu6050_dev_t dev = {0};
imu_calibration_offsets_t offsets;

extern QueueHandle_t imu_data_queue;
// self-test MPU6050

void mpu6050_calibrate_user(void)
{
    offsets.oAx = 0;
    offsets.oAy = 0;
    offsets.oAz = 0;
    offsets.oGx = 0;
    offsets.oGy = 0;
    offsets.oGz = 0;
    imu_data_t temp;

    // configure MPU6050
    int num_samples = 5000;
    // set accelerometer full scale range
    mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_2); // ±2 G
    // set gyroscope full scale range
    mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_250); // ±250 deg/s
    // set clock source
    mpu6050_set_clock_source(&dev, MPU6050_CLOCK_PLL_Y);
    // set sample rate to 1 Khz
    mpu6050_set_rate(&dev, 0);
    // enable MPU6050 FIFO
    mpu6050_set_fifo_enabled(&dev, true);
    // set DLPF
    mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_1);
    // delay for user to place the IMU on a flat surface
    ESP_LOGI(IMU_TAG, "Calibration starting, place the device on a flat surface.");
    vTaskDelay(pdMS_TO_TICKS(3000));
    // take 500 samples of all six axes at 100 Hz
    for (int i = 0; i < num_samples; i++)
    {
        mpu6050_measure_raw(&temp);
        offsets.oAx += temp.acc.x;
        offsets.oAy += temp.acc.y;
        offsets.oAz += temp.acc.z - 1; // account for gravity
        offsets.oGx += temp.rot.x;
        offsets.oGy += temp.rot.y;
        offsets.oGz += temp.rot.z;
        // vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);
    // average all six axes
    offsets.oAx /= num_samples;
    offsets.oAy /= num_samples;
    offsets.oAz /= num_samples;
    offsets.oGx /= num_samples;
    offsets.oGy /= num_samples;
    offsets.oGz /= num_samples;
    // save six offset floats to NVS
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);
    NVS_write_imu_calibration_offsets(&offsets);
    ESP_LOGI(IMU_TAG, "MPU6050 calibrated.");
    // NVS_read_imu_calibration_offsets(&offsets);
}

void mpu6050_load_calibration_offsets(void)
{
    // read accel and gyro calibration offsets from NVS
    NVS_read_imu_calibration_offsets(&offsets);
    ESP_LOGI(IMU_TAG, "MPU6050 read calibration from NVS.");
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.2f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.2f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.2f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.2f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.2f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.2f", offsets.oGz);
}

// initialize and configure MPU6050
void mpu6050_init_custom(uint8_t calibrate_imu)
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

    // calibrate IMU
    if (calibrate_imu)
    {
        mpu6050_calibrate_user();
    }
    else
    {
        mpu6050_load_calibration_offsets();
    }

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
    ESP_LOGI(IMU_TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(IMU_TAG, "Gyro range:  %d", dev.ranges.gyro);
}

esp_err_t mpu6050_measure_raw(imu_data_t *data)
{
    // measure MPU6050
    ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &data->temp));
    ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &data->acc, &data->rot));
    return ESP_OK;
}

// read measurements from MPU6050
esp_err_t mpu6050_measure(imu_data_t *data)
{
    // measure MPU6050
    ESP_ERROR_CHECK(mpu6050_measure_raw(data));
    // apply calibration offsets
    data->acc.x -= offsets.oAx;
    data->acc.y -= offsets.oAy;
    data->acc.z -= offsets.oAz;
    data->rot.x -= offsets.oGx;
    data->rot.y -= offsets.oGy;
    data->rot.z -= offsets.oGz;
    return ESP_OK;
}

// queue for IMU data from the IMU measuring task to the data logging task
const int imu_queue_len = 30;
const uint16_t imu_sample_rate_hz = 10;
const uint16_t imu_sample_period_ms = 1000 / imu_sample_rate_hz;

void imu_read_task(void *pvParameters)
{
    imu_data_t data;
    while (1)
    {
        mpu6050_measure(&data);
        if (xQueueSend(imu_data_queue, &data, 10) != pdTRUE)
        {
            ESP_LOGE(IMU_TAG, "ERROR: Could not put item on IMU queue.");
        }
        vTaskDelay(imu_sample_period_ms / portTICK_PERIOD_MS);
    }
}
// ****************************************************************

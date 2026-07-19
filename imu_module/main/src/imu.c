#include "../include/imu.h"

// MPU6050
// ****************************************************************

// const uint16_t IMU_SAMPLE_PERIOD_MS = 1000 / IMU_SAMPLE_RATE_HZ;

mpu6050_dev_t dev = {0};
imu_calibration_offsets_t offsets;

extern QueueHandle_t queue_imu_data;

extern TaskHandle_t task_handle_status_led;

// self-test MPU6050
// queue for IMU data from the IMU measuring task to the data logging task

// NVS offset read write function callback
nvs_func_t offset_read_cb, offset_write_cb;

void imu_set_offset_read_cb(nvs_func_t cb)
{
    offset_read_cb = cb;
}

void imu_set_offset_write_cb(nvs_func_t cb)
{
    offset_write_cb = cb;
}

void imu_calibrate_user(void)
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
    mpu6050_set_clock_source(&dev, MPU6050_CLOCK_PLL_X);
    // set sample rate to 1 Khz
    mpu6050_set_rate(&dev, 0);
    // enable MPU6050 FIFO
    mpu6050_set_fifo_enabled(&dev, true);
    // set DLPF
    mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_3);
    // wake IMU up
    mpu6050_set_sleep_enabled(&dev, false);
    // delay for user to place the IMU on a flat surface
    ESP_LOGI(IMU_TAG, "Calibration starting, place the device on a flat surface.");
    vTaskDelay(pdMS_TO_TICKS(3000));
    // take 500 samples of all six axes at 100 Hz
    for (int i = 0; i < num_samples; i++)
    {
        imu_measure_raw(&temp);
        offsets.oAx += temp.ax;
        offsets.oAy += temp.ay;
        offsets.oAz += temp.az - 1; // account for gravity
        offsets.oGx += temp.gx;
        offsets.oGy += temp.gy;
        offsets.oGz += temp.gz;
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

    // NVS_write_imu_calibration_offsets(&offsets);
    offset_write_cb(&offsets);
    ESP_LOGI(IMU_TAG, "MPU6050 calibrated.");
    // NVS_read_imu_calibration_offsets(&offsets);
    offset_read_cb(&offsets);

    vTaskDelay(pdMS_TO_TICKS(1000));
}

void imu_load_calibration_offsets(void)
{
    // read accel and gyro calibration offsets from NVS
    // NVS_read_imu_calibration_offsets(&offsets);
    offset_read_cb(&offsets);
    ESP_LOGI(IMU_TAG, "MPU6050 read calibration from NVS.");
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.2f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.2f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.2f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.2f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.2f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.2f", offsets.oGz);
}

// initialize and configure MPU6050
void imu_init_custom(uint8_t calibrate_imu)
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
        imu_calibrate_user();
    }
    else
    {
        imu_load_calibration_offsets();
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

/*
read raw measurements from MPU6050
*/
esp_err_t imu_measure_raw(imu_data_t *data)
{
    // measure MPU6050
    mpu6050_acceleration_t temp_acc;
    mpu6050_rotation_t temp_rot;
    ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &data->temp));
    ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &temp_acc, &temp_rot));
    data->ax = temp_acc.x;
    data->ay = temp_acc.y;
    data->az = temp_acc.z;
    data->gx = temp_rot.x;
    data->gy = temp_rot.y;
    data->gz = temp_rot.z;
    return ESP_OK;
}

/*
read calibration-offset measurements from MPU6050
*/
esp_err_t imu_measure(imu_data_t *data)
{
    // measure MPU6050
    ESP_ERROR_CHECK(imu_measure_raw(data));
    // apply calibration offsets
    data->ax -= offsets.oAx;
    data->ay -= offsets.oAy;
    data->az -= offsets.oAz;
    data->gx -= offsets.oGx;
    data->gy -= offsets.oGy;
    data->gz -= offsets.oGz;
    return ESP_OK;
}

// void acc_to_euler(imu_data_t *data, euler_angles_t *euler_angles)
// {
//     // convert accelerometer data to euler)
//     /*
//     [ax, ay, az] = self.get_acc()
//         phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
//         theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))
//         return [phi, theta]
//         */
//     euler_angles->roll_phi = atan2(data->ay, sqrt(data->ax * data->ax + data->az * data->az));
//     euler_angles->pitch_theta = atan2(-data->ax, sqrt(data->ay * data->ay + data->az * data->az));
//     euler_angles->yaw_psi = 0;
// }

// void mpu6050_preprocess(imu_data_t *x)
// {
//     static imu_data_t y_prev;
//     // lowpass filter for noise reduction
//     // recursive averaging filter
//     imu_data_t y;
//     float averaging_alpha = 0.5;
//     y.acc.x = averaging_alpha * x->acc.x + (1 - averaging_alpha) * y_prev.acc.x;
//     y.acc.y = averaging_alpha * x->acc.y + (1 - averaging_alpha) * y_prev.acc.y;
//     y.acc.z = averaging_alpha * x->acc.z + (1 - averaging_alpha) * y_prev.acc.z;
//     y.rot.x = averaging_alpha * x->rot.x + (1 - averaging_alpha) * y_prev.rot.x;
//     y.rot.y = averaging_alpha * x->rot.y + (1 - averaging_alpha) * y_prev.rot.y;
//     y.rot.z = averaging_alpha * x->rot.z + (1 - averaging_alpha) * y_prev.rot.z;
//     y_prev = y;
//     // convert accelerometer and gyroscope readings to roll pitch and yaw
//     euler_angles_t euler_angles;
//     acc_to_euler(&y, &euler_angles);

//     // sensor fusion (Complementary filter)

//     // sensor fusion (Kalman Filter)

//     // double integration and dead reckoning?
// }

// ****************************************************************

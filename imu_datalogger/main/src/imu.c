#include "imu.h"

/* IMU calibration constants storage */
imu_calibration_offsets_t offsets;

/* NVS offset read write function callback */
nvs_func_t offset_read_cb,
    offset_write_cb;

void imu_set_offset_read_cb(nvs_func_t cb)
{
    offset_read_cb = cb;
}

void imu_set_offset_write_cb(nvs_func_t cb)
{
    offset_write_cb = cb;
}

void imu_calibrate(i2c_master_dev_handle_t imu_dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer)
{
    offsets.oAx = 0;
    offsets.oAy = 0;
    offsets.oAz = 0;
    offsets.oGx = 0;
    offsets.oGy = 0;
    offsets.oGz = 0;
    int num_samples_to_average = 500;
    int num_samples_to_read = 0;
    uint16_t num_samples_taken = 0;
    uint16_t num_samples_taken_from_fifo = 0;

    ESP_LOGI(IMU_TAG, "Calibration starting, place the device on a flat surface.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* start reading data from FIFO */
    // lsm6ds3_fifo_reset_start(imu_dev_handle);

    while (num_samples_taken < num_samples_to_average)
    {
        lsm6ds3_read_raw_data(imu_dev_handle, &lsm6ds3_fifo_buffer[0]);
        offsets.oAx += lsm6ds3_fifo_buffer[0].accel[0];
        offsets.oAy += lsm6ds3_fifo_buffer[0].accel[1];
        offsets.oAz += lsm6ds3_fifo_buffer[0].accel[2] - 1; // account for gravity
        offsets.oGx += lsm6ds3_fifo_buffer[0].gyro[0];
        offsets.oGy += lsm6ds3_fifo_buffer[0].gyro[1];
        offsets.oGz += lsm6ds3_fifo_buffer[0].gyro[2];
        vTaskDelay(IMU_LOGGING_TIMEDELTA_MS / portTICK_PERIOD_MS);
        num_samples_taken++;
    }

    ESP_LOGI(IMU_TAG, "num_samples_taken_from_fifo = %d\n", num_samples_taken);
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);

    /* stop and reset FIFO */
    lsm6ds3_fifo_reset(imu_dev_handle);

    /* average all six axes */
    offsets.oAx /= num_samples_taken;
    offsets.oAy /= num_samples_taken;
    offsets.oAz /= num_samples_taken;
    offsets.oGx /= num_samples_taken;
    offsets.oGy /= num_samples_taken;
    offsets.oGz /= num_samples_taken;

    // save six offset floats to NVS
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);

    offset_write_cb(&offsets);
    ESP_LOGI(IMU_TAG, "IMU calibrated.");
    offset_read_cb(&offsets);

    vTaskDelay(pdMS_TO_TICKS(1000));
}

void imu_load_calibration_offsets(void)
{
    // read accel and gyro calibration offsets from NVS
    // NVS_read_imu_calibration_offsets(&offsets);
    offset_read_cb(&offsets);
    ESP_LOGI(IMU_TAG, "IMU read calibration from NVS.");
    ESP_LOGI(IMU_TAG, "accel_offsets_x = %.2f", offsets.oAx);
    ESP_LOGI(IMU_TAG, "accel_offsets_y = %.2f", offsets.oAy);
    ESP_LOGI(IMU_TAG, "accel_offsets_z = %.2f", offsets.oAz);
    ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.2f", offsets.oGx);
    ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.2f", offsets.oGy);
    ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.2f", offsets.oGz);
}

void imu_init(i2c_master_dev_handle_t imu_dev_handle, uint16_t sample_rate, uint16_t xl_fs, uint16_t g_fs, uint8_t calibrate_imu)
{
    /* initialize lsm6ds3 */
    lsm6ds3_init_all(imu_dev_handle, IMU_ODR_HZ, IMU_XL_FS, IMU_G_FS);
}

uint16_t imu_read_FIFO_calibrated(i2c_master_dev_handle_t imu_dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, imu_raw_data_t *imu_data_buffer, uint16_t num_samples)
{
    /* read from LSM6DS3 FIFO */
    uint16_t num_timesteps_read = lsm6ds3_fifo_read(imu_dev_handle, lsm6ds3_fifo_buffer, num_samples);

    if (num_timesteps_read > 0)
    {
        /* apply calibration offsets to every sample read from the FIFO */
        for (int i = 0; i < num_samples; i++)
        {
            /* convert lsm6ds3_data_t to imu_raw_data_t */
            __imu_convert_vals(&lsm6ds3_fifo_buffer[i], &imu_data_buffer[i]);
            /* apply offset calibration */
            imu_apply_calibration(&imu_data_buffer[i]);
        }
    }
    return num_timesteps_read;
}

int imu_measure_raw(i2c_master_dev_handle_t imu_dev_handle, imu_raw_data_t *data)
{

    // measure LSM6DS3
    lsm6ds3_data_t lsm6ds3_data;
    int val = lsm6ds3_read_raw_data(imu_dev_handle, &lsm6ds3_data);
    __imu_convert_vals(&lsm6ds3_data, data);
    return val;
}

void __imu_convert_vals(lsm6ds3_data_t *lsm6ds3_data, imu_raw_data_t *imu_data)
{
    imu_data->temp = lsm6ds3_data->temp;
    imu_data->ax = lsm6ds3_data->accel[0];
    imu_data->ay = lsm6ds3_data->accel[1];
    imu_data->az = lsm6ds3_data->accel[2];
    imu_data->gx = lsm6ds3_data->gyro[0];
    imu_data->gy = lsm6ds3_data->gyro[1];
    imu_data->gz = lsm6ds3_data->gyro[2];
}

void imu_apply_calibration(imu_raw_data_t *data)
{
    data->ax -= offsets.oAx;
    data->ay -= offsets.oAy;
    data->az -= offsets.oAz;
    data->gx -= offsets.oGx;
    data->gy -= offsets.oGy;
    data->gz -= offsets.oGz;
}

int imu_measure_calibrated(i2c_master_dev_handle_t imu_dev_handle, imu_raw_data_t *data)
{
    // measure IMU
    int val = imu_measure_raw(imu_dev_handle, data);
    // apply calibration offsets
    imu_apply_calibration(data);
    return val;
}

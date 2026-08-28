#include "../include/imu.h"

// ****************************************************************

// const uint16_t IMU_SAMPLE_PERIOD_MS = 1000 / IMU_ODR_HZ;

/* IMU calibration constants storage */
imu_calibration_offsets_t offsets;

/* NVS offset read write function callback */
nvs_func_t offset_read_cb,
    offset_write_cb;

/**
 * @brief IMU reading task
 *
 * @param params type params_imu_task_t
 */
void task_imu(void *params)
{
    /* FIFO buffer storage for lsm6ds3 */
    lsm6ds3_data_t lsm6ds3_fifo_buffer[NUM_FIFO_TIMESTAMPS];
    /* IMU data buffer */
    imu_data_t imu_data_buffer[NUM_FIFO_TIMESTAMPS];

    /* state machine vars */
    cmd_imu_task_t imu_task_cmd = CMD_IMU_STOP;
    cmd_imu_task_t prev_imu_task_cmd = CMD_IMU_STOP;

    /* IMU task delay period */
    TickType_t imu_task_delay_period;
    // imu_data_t imu_data;

    /* parse freeRTOS primitives from params */
    params_imu_task_t *params_imu_task = (params_imu_task_t *)params;

    /* status LED task handle */
    TaskHandle_t *task_handle_status_led = params_imu_task->task_handle_status_led;
    /* IMU data queue */
    QueueHandle_t queue_imu = params_imu_task->queue_imu;
    /* IMU data stream buffer */
    StreamBufferHandle_t streambuffer_imu = params_imu_task->streambuffer_imu;
    /* IMU i2c device handle */
    i2c_master_dev_handle_t dev_handle = params_imu_task->dev_handle;

    /* current timestamp */
    uint64_t cur_timestamp;
    /* number of samples read from the FIFO */
    uint16_t num_samples_read;

    while (1)
    {
        /* variable delay */
        if (imu_task_cmd == CMD_IMU_READ_LOOP)
        {
            /* delay for half the time to fill the FIFO */
            imu_task_delay_period = IMU_LOGGING_TIMEDELTA_MS * NUM_FIFO_TIMESTAMPS / 2;
        }
        else
        {
            imu_task_delay_period = portMAX_DELAY;
        }

        /* wait for task notification from button */
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&imu_task_cmd,
                        (imu_task_delay_period / portTICK_PERIOD_MS));
        // if (xTaskNotifyWait(0,
        //                     0,
        //                     (uint32_t *)&imu_task_cmd,
        //                     (imu_task_delay_period / portTICK_PERIOD_MS)) == pdTRUE)
        // ESP_LOGI(IMU_TAG, "cmd = %d", imu_task_cmd);

        /* runs when IMU calibration is initiated */
        if (imu_task_cmd == CMD_IMU_CALIBRATE)
        {
            /* notification LED fast blink */
            if (*task_handle_status_led != NULL)
                xTaskNotify(*task_handle_status_led, STATUS_LED_FAST_BLINK, eSetValueWithOverwrite);
            /* calibrate IMU */
            imu_calibrate(dev_handle, lsm6ds3_fifo_buffer);
            /* notification LED off */
            if (*task_handle_status_led != NULL)
                xTaskNotify(*task_handle_status_led, STATUS_LED_OFF, eSetValueWithOverwrite);
            /* stop IMU after calibration */
            imu_task_cmd = CMD_IMU_STOP;
        }
        /* runs when datalogging is started */
        else if (imu_task_cmd == CMD_IMU_READ_LOOP && prev_imu_task_cmd == CMD_IMU_STOP)
        {
            /* set initial timestamp */
            cur_timestamp = esp_timer_get_time() / 1000;
            /* reset FIFO */
            lsm6ds3_fifo_reset_start(dev_handle);

            ESP_LOGI(IMU_TAG, "IMU started");
        }
        /* runs continously when logging data */
        else if (imu_task_cmd == CMD_IMU_READ_LOOP && prev_imu_task_cmd == CMD_IMU_READ_LOOP)
        {
            // ESP_LOGI(IMU_TAG, "IMU read");
            // imu_measure(&imu_data);
            /* read IMU FIFO buffer */
            num_samples_read = imu_read_FIFO_calibrated(dev_handle, lsm6ds3_fifo_buffer, imu_data_buffer, NUM_FIFO_TIMESTAMPS);

            /* if samples were read from the IMU FIFO buffer, send it to the queue / streambuffer. */
            if (num_samples_read > 0)
            {
                for (int i = 0; i < num_samples_read; i++)
                {
                    /* add timestamp to each IMU data in the FIFO */
                    imu_data_buffer[i].timestamp = cur_timestamp;
                    cur_timestamp += IMU_LOGGING_TIMEDELTA_MS;

                    /* send IMU data to queue */
                    if (queue_imu != NULL)
                    {
                        if (xQueueSend(queue_imu,
                                       &imu_data_buffer[i],
                                       (imu_task_delay_period / portTICK_PERIOD_MS)) != pdTRUE)
                        {
                            ESP_LOGE(IMU_TAG, "ERROR: Could not put item on IMU queue.");
                        }
                    }
                }

                // imu_data.timestamp = esp_timer_get_time() / 1000;
                // ESP_LOGI(IMU_TAG, "timestamp=%d", imu_data.timestamp);
                // ESP_LOGI(IMU_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_data.ax, imu_data.ay, imu_data.az);
                // ESP_LOGI(IMU_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_data.gx, imu_data.gy, imu_data.gz);
                // ESP_LOGI(IMU_TAG, "Temperature:  %.1f\n", imu_data.temp);

                // /* send IMU data to streambuffer */
                // if (streambuffer_imu != NULL)
                // {
                //     if (xStreamBufferSend(streambuffer_imu,
                //                           imu_data_buffer,
                //                           sizeof(imu_data_t) * num_samples_read,
                //                           10 / portTICK_PERIOD_MS) == 0)
                //     {
                //         ESP_LOGE(IMU_TAG, "ERROR: Could not put item on IMU stream buffer.");
                //     }
                // }
            }
        }
        /* runs when datalogging is stopped */
        else if (imu_task_cmd == CMD_IMU_STOP && prev_imu_task_cmd == CMD_IMU_READ_LOOP)
        {
            ESP_LOGI(IMU_TAG, "IMU stopped");
        }
        /* runs continuously when not logging data */
        else
        {
        }

        prev_imu_task_cmd = imu_task_cmd;

        // vTaskDelay(IMU_SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief set the callback function to read the IMU calibration constants
 * from the NVS.
 */
void imu_set_offset_read_cb(nvs_func_t cb)
{
    offset_read_cb = cb;
}

/**
 * @brief set the callback function to write the IMU calibration constants
 * to the NVS.
 */
void imu_set_offset_write_cb(nvs_func_t cb)
{
    offset_write_cb = cb;
}

/**
 * @brief calibrate the IMU
 *
 * @param dev_handle i2c handle of the IMU
 */
void imu_calibrate(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer)
{
    /* FIFO buffer storage for lsm6ds3 */
    // ESP_LOGI(IMU_TAG, "sizeof(float) = %d", sizeof(float));
    // ESP_LOGI(IMU_TAG, "sizeof(double) = %d", sizeof(double));

    offsets.oAx = 0;
    offsets.oAy = 0;
    offsets.oAz = 0;
    offsets.oGx = 0;
    offsets.oGy = 0;
    offsets.oGz = 0;
    int num_samples_to_average = NUM_FIFO_TIMESTAMPS;
    int num_samples_to_read = 0;
    uint16_t num_samples_taken = 0;
    uint16_t num_samples_taken_from_fifo = 0;

    ESP_LOGI(IMU_TAG, "Calibration starting, place the device on a flat surface.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* start reading data from FIFO */
    lsm6ds3_fifo_reset_start(dev_handle);

    while (num_samples_taken < num_samples_to_average)
    {
        /* split the number of samples to read into chunks that can fit in the FIFO */
        if ((num_samples_to_average - num_samples_taken) > NUM_FIFO_TIMESTAMPS)
        {
            num_samples_to_read = NUM_FIFO_TIMESTAMPS;
        }
        else
        {
            num_samples_to_read = (num_samples_to_average - num_samples_taken);
        }

        /* read samples from IMU FIFO */
        num_samples_taken_from_fifo = lsm6ds3_fifo_read(dev_handle, lsm6ds3_fifo_buffer, num_samples_to_read);

        /* if FIFO data was read */
        if (num_samples_taken_from_fifo > 0)
        {
            num_samples_taken += num_samples_taken_from_fifo;

            /* sum up the values */
            for (int i = 0; i < num_samples_taken_from_fifo; i++)
            {
                offsets.oAx += lsm6ds3_fifo_buffer[i].accel[0];
                offsets.oAy += lsm6ds3_fifo_buffer[i].accel[1];
                offsets.oAz += lsm6ds3_fifo_buffer[i].accel[2] - 1; // account for gravity
                offsets.oGx += lsm6ds3_fifo_buffer[i].gyro[0];
                offsets.oGy += lsm6ds3_fifo_buffer[i].gyro[1];
                offsets.oGz += lsm6ds3_fifo_buffer[i].gyro[2];
            }

            ESP_LOGI(IMU_TAG, "num_samples_taken_from_fifo = %d\n", num_samples_taken_from_fifo);
            ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
            ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
            ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
            ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
            ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
            ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);
        }
    }

    /* stop and reset FIFO */
    lsm6ds3_fifo_reset(dev_handle);

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

    // NVS_write_imu_calibration_offsets(&offsets);
    offset_write_cb(&offsets);
    ESP_LOGI(IMU_TAG, "IMU calibrated.");
    // NVS_read_imu_calibration_offsets(&offsets);
    offset_read_cb(&offsets);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // // configure IMU
    // int num_samples = 5000;
    // // set accelerometer full scale range
    // // mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_2); // ±2 G

    // // set gyroscope full scale range
    // mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_250); // ±250 deg/s
    // // set clock source
    // mpu6050_set_clock_source(&dev, MPU6050_CLOCK_PLL_X);
    // // set sample rate to 1 Khz
    // mpu6050_set_rate(&dev, 0);
    // // enable MPU6050 FIFO
    // mpu6050_set_fifo_enabled(&dev, true);
    // // set DLPF
    // mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_3);
    // // wake IMU up
    // mpu6050_set_sleep_enabled(&dev, false);
    // // delay for user to place the IMU on a flat surface
    // ESP_LOGI(IMU_TAG, "Calibration starting, place the device on a flat surface.");
    // vTaskDelay(pdMS_TO_TICKS(3000));
    // // take 500 samples of all six axes at 100 Hz
    // for (int i = 0; i < num_samples; i++)
    // {
    //     imu_measure_raw(&temp);
    //     offsets.oAx += temp.ax;
    //     offsets.oAy += temp.ay;
    //     offsets.oAz += temp.az - 1; // account for gravity
    //     offsets.oGx += temp.gx;
    //     offsets.oGy += temp.gy;
    //     offsets.oGz += temp.gz;
    //     // vTaskDelay(pdMS_TO_TICKS(10));
    // }
    // ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
    // ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
    // ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
    // ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
    // ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
    // ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);
    // // average all six axes
    // offsets.oAx /= num_samples;
    // offsets.oAy /= num_samples;
    // offsets.oAz /= num_samples;
    // offsets.oGx /= num_samples;
    // offsets.oGy /= num_samples;
    // offsets.oGz /= num_samples;
    // // save six offset floats to NVS
    // ESP_LOGI(IMU_TAG, "accel_offsets_x = %.4f", offsets.oAx);
    // ESP_LOGI(IMU_TAG, "accel_offsets_y = %.4f", offsets.oAy);
    // ESP_LOGI(IMU_TAG, "accel_offsets_z = %.4f", offsets.oAz);
    // ESP_LOGI(IMU_TAG, "gyro_offsets_x = %.4f", offsets.oGx);
    // ESP_LOGI(IMU_TAG, "gyro_offsets_y = %.4f", offsets.oGy);
    // ESP_LOGI(IMU_TAG, "gyro_offsets_z = %.4f", offsets.oGz);

    // // NVS_write_imu_calibration_offsets(&offsets);
    // offset_write_cb(&offsets);
    // ESP_LOGI(IMU_TAG, "MPU6050 calibrated.");
    // // NVS_read_imu_calibration_offsets(&offsets);
    // offset_read_cb(&offsets);

    // vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief load IMU calibration offsets from the NVS to memory
 */
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

/**
 * @brief initialize and configure IMU
 *
 * @param dev_handle i2c handle of the IMU
 * @param calibrate_imu calibrate the IMU if set
 */
void imu_init(i2c_master_dev_handle_t dev_handle, uint16_t sample_rate, uint16_t xl_fs, uint16_t g_fs, uint8_t calibrate_imu)
{
    /* initialize lsm6ds3 */
    lsm6ds3_init_all(dev_handle, IMU_ODR_HZ, IMU_XL_FS, IMU_G_FS);

    /* enable FIFO */
    lsm6ds3_fifo_init(dev_handle, IMU_ODR_HZ);

    // /* calibrate IMU and save calibration offsets to NVS */
    // if (calibrate_imu)
    // {
    //     imu_calibrate(dev_handle, );
    // }
    // /* load IMU calibration offsets from NVS */
    // else
    // {
    imu_load_calibration_offsets();
    // }

    // old code
    // uint8_t mpu6050_id = 0;
    // ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, SDA_PIN, SCL_PIN));
    // // search for MPU6050 on i2c bus
    // while (1)
    // {
    //     esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
    //     if (res == ESP_OK)
    //     {
    //         ESP_LOGI(IMU_TAG, "Found MPU60x0 device");
    //         break;
    //     }
    //     ESP_LOGE(IMU_TAG, "MPU60x0 not found");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    // // initialize and wake up MPU6050
    // ESP_ERROR_CHECK(mpu6050_init(&dev));

    // // calibrate IMU
    // if (calibrate_imu)
    // {
    //     imu_calibrate();
    // }
    // else
    // {
    //     imu_load_calibration_offsets();
    // }

    // // get MPU6050 WHOAMI
    // ESP_ERROR_CHECK(mpu6050_get_device_id(&dev, &mpu6050_id));
    // ESP_LOGI(IMU_TAG, "MPU6050 ID: %d", mpu6050_id);
    // // set IMU DLPF
    // ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_2));
    // // set IMU DHPF
    // ESP_ERROR_CHECK(mpu6050_set_dhpf_mode(&dev, MPU6050_DHPF_0_63));
    // // set IMU sample rate
    // ESP_ERROR_CHECK(mpu6050_set_rate(&dev, 0));
    // // set IMU accel and gyro ranges
    // ESP_ERROR_CHECK(mpu6050_set_full_scale_accel_range(&dev, MPU6050_ACCEL_RANGE_2));
    // ESP_ERROR_CHECK(mpu6050_set_full_scale_gyro_range(&dev, MPU6050_GYRO_RANGE_250));
    // // set IMU clock source
    // ESP_ERROR_CHECK(mpu6050_set_clock_source(&dev, MPU6050_CLOCK_PLL_Y));
    // // enable MPU6050 FIFO
    // mpu6050_set_fifo_enabled(&dev, true);
    // ESP_LOGI(IMU_TAG, "Accel range: %d", dev.ranges.accel);
    // ESP_LOGI(IMU_TAG, "Gyro range:  %d", dev.ranges.gyro);
}

/**
 * @brief read a number of samples from the FIFO and apply offsets to all measurements.
 *
 * @param dev_handle i2c handle of the IMU
 * @param imu_data_buffer pointer to IMU data buffer
 * @param num_samples number of samples to read from FIFO
 */
uint16_t imu_read_FIFO_calibrated(i2c_master_dev_handle_t dev_handle, lsm6ds3_data_t *lsm6ds3_fifo_buffer, imu_data_t *imu_data_buffer, uint16_t num_samples)
{
    /* read from LSM6DS3 FIFO */
    uint16_t num_timesteps_read = lsm6ds3_fifo_read(dev_handle, lsm6ds3_fifo_buffer, num_samples);

    if (num_timesteps_read > 0)
    {
        /* apply calibration offsets to every sample read from the FIFO */
        for (int i = 0; i < num_samples; i++)
        {
            /* convert lsm6ds3_data_t to imu_data_t */
            __imu_convert_vals(&lsm6ds3_fifo_buffer[i], &imu_data_buffer[i]);
            /* apply offset calibration */
            imu_apply_calibration(&imu_data_buffer[i]);
        }
    }
    return num_timesteps_read;
}

/**
 * @brief read raw measurements from IMU
 *
 * @param dev_handle i2c handle of the IMU
 * @param data pointer to destination struct
 */
esp_err_t imu_measure_raw(i2c_master_dev_handle_t dev_handle, imu_data_t *data)
{
    // measure LSM6DS3
    lsm6ds3_data_t lsm6ds3_data;
    lsm6ds3_read_raw_data(dev_handle, &lsm6ds3_data);
    __imu_convert_vals(&lsm6ds3_data, data);
    return ESP_OK;
}

/**
 * @brief convert LSM6DS3 data to IMU agnostic orientation data
 *
 * @param lsm6ds3_data pointer to lsm6ds3 data struct
 * @param imu__data pointer to IMU data struct
 */
void __imu_convert_vals(lsm6ds3_data_t *lsm6ds3_data, imu_data_t *imu_data)
{
    imu_data->temp = lsm6ds3_data->temp;
    imu_data->ax = lsm6ds3_data->accel[0];
    imu_data->ay = lsm6ds3_data->accel[1];
    imu_data->az = lsm6ds3_data->accel[2];
    imu_data->gx = lsm6ds3_data->gyro[0];
    imu_data->gy = lsm6ds3_data->gyro[1];
    imu_data->gz = lsm6ds3_data->gyro[2];
}

/**
 * @brief apply saved calibration offsets to raw IMU data
 *
 * @param imu__data pointer to IMU data struct
 */
void imu_apply_calibration(imu_data_t *data)
{
    data->ax -= offsets.oAx;
    data->ay -= offsets.oAy;
    data->az -= offsets.oAz;
    data->gx -= offsets.oGx;
    data->gy -= offsets.oGy;
    data->gz -= offsets.oGz;
}

/**
 * @brief read calibration-offset measurements from IMU
 *
 * @param dev_handle i2c handle of the IMU
 * @param imu__data pointer to IMU data struct
 */
esp_err_t imu_measure_calibrated(i2c_master_dev_handle_t dev_handle, imu_data_t *data)
{
    // measure IMU
    ESP_ERROR_CHECK(imu_measure_raw(dev_handle, data));
    // apply calibration offsets
    imu_apply_calibration(data);
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

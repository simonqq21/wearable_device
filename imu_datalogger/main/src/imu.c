#include "../include/imu.h"

/* IMU calibration constants storage */
imu_calibration_offsets_t offsets;

/* NVS offset read write function callback */
nvs_func_t offset_read_cb,
    offset_write_cb;

/**
 * @brief IMU reading task
 *
 * @param params type params_task_imu_t
 */
void task_imu(void *params)
{
    /* FIFO buffer storage for lsm6ds3 */
    lsm6ds3_data_t lsm6ds3_fifo_buffer[1];
    /* IMU data buffer */
    imu_data_t imu_data_buffer[1];

    /* state machine vars */
    cmd_imu_task_t imu_task_cmd = CMD_IMU_STOP;
    cmd_imu_task_t prev_imu_task_cmd = CMD_IMU_STOP;

    /* IMU task delay period */
    TickType_t imu_task_delay_period;
    // imu_data_t imu_data;

    /* parse freeRTOS primitives from params */
    params_task_imu_t *params_imu_task = (params_task_imu_t *)params;

    /* status LED task handle */
    TaskHandle_t *task_handle_status_led = params_imu_task->task_handle_status_led;
    /* IMU data queue */
    QueueHandle_t queue_imu = params_imu_task->queue_imu;
    /* IMU data stream buffer */
    // StreamBufferHandle_t streambuffer_imu = params_imu_task->streambuffer_imu;
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
            /* delay per IMU measurement */
            imu_task_delay_period = IMU_LOGGING_TIMEDELTA_MS;
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
            cur_timestamp = esp_timer_get_time();
            ESP_LOGI(IMU_TAG, "cur_timestamp start = %lld\n", cur_timestamp);
            /* reset FIFO */
            // lsm6ds3_fifo_reset_start(dev_handle);

            ESP_LOGI(IMU_TAG, "IMU started");
        }
        /* runs continously when logging data */
        else if (imu_task_cmd == CMD_IMU_READ_LOOP && prev_imu_task_cmd == CMD_IMU_READ_LOOP)
        {
            /* read one sample from IMU */
            if (imu_measure_calibrated(dev_handle, &imu_data_buffer[0]) == 0)
            {
                imu_data_buffer[0].timestamp = esp_timer_get_time();
                // ESP_LOGI(IMU_TAG, "calib IMU data %lld %.4f %.4f %.4f %.4f %.4f %.4f ",
                //          imu_data_buffer[0].timestamp,
                //          imu_data_buffer[0].ax,
                //          imu_data_buffer[0].ay,
                //          imu_data_buffer[0].az,
                //          imu_data_buffer[0].gx,
                //          imu_data_buffer[0].gy,
                //          imu_data_buffer[0].gz);

                /* send IMU data to queue */
                if (queue_imu != NULL)
                {
                    if (xQueueSend(queue_imu,
                                   &imu_data_buffer[0],
                                   (imu_task_delay_period / portTICK_PERIOD_MS)) != pdTRUE)
                    {
                        ESP_LOGE(IMU_TAG, "ERROR: Could not put item on IMU queue.");
                    }
                }
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
    offsets.oAx = 0;
    offsets.oAy = 0;
    offsets.oAz = 0;
    offsets.oGx = 0;
    offsets.oGy = 0;
    offsets.oGz = 0;
    int num_samples_to_average = 1500;
    int num_samples_to_read = 0;
    uint16_t num_samples_taken = 0;
    uint16_t num_samples_taken_from_fifo = 0;

    ESP_LOGI(IMU_TAG, "Calibration starting, place the device on a flat surface.");
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* start reading data from FIFO */
    // lsm6ds3_fifo_reset_start(dev_handle);

    while (num_samples_taken < num_samples_to_average)
    {
        lsm6ds3_read_raw_data(dev_handle, &lsm6ds3_fifo_buffer[0]);
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

    offset_write_cb(&offsets);
    ESP_LOGI(IMU_TAG, "IMU calibrated.");
    offset_read_cb(&offsets);

    vTaskDelay(pdMS_TO_TICKS(1000));
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
int imu_measure_raw(i2c_master_dev_handle_t dev_handle, imu_data_t *data)
{

    // measure LSM6DS3
    lsm6ds3_data_t lsm6ds3_data;
    int val = lsm6ds3_read_raw_data(dev_handle, &lsm6ds3_data);
    __imu_convert_vals(&lsm6ds3_data, data);
    return val;
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
int imu_measure_calibrated(i2c_master_dev_handle_t dev_handle, imu_data_t *data)
{
    // measure IMU
    int val = imu_measure_raw(dev_handle, data);
    // apply calibration offsets
    imu_apply_calibration(data);
    return val;
}

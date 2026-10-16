/**
 * This task polls both the PPG heart rate sensor and IMU sensor, and
 * send the raw data to the DSP task.
 *
 */

#include "TaskSensors.h"

extern TaskHandle_t task_handle_GPIO;

void task_sensors(void *params)
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
    params_task_sensors_t *params_imu_task = (params_task_sensors_t *)params;

    /* IMU data queue */
    QueueHandle_t queue_imu = params_imu_task->queue_imu;
    /* IMU data stream buffer */
    // StreamBufferHandle_t streambuffer_imu = params_imu_task->streambuffer_imu;
    /* IMU i2c device handle */
    i2c_master_dev_handle_t imu_dev_handle = params_imu_task->imu_dev_handle;

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
            if (task_handle_GPIO != NULL)
                xTaskNotify(task_handle_GPIO, STATUS_LED_FAST_BLINK, eSetValueWithOverwrite);
            /* calibrate IMU */
            imu_calibrate(imu_dev_handle, lsm6ds3_fifo_buffer);
            /* notification LED off */
            if (task_handle_GPIO != NULL)
                xTaskNotify(task_handle_GPIO, STATUS_LED_OFF, eSetValueWithOverwrite);
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
            // lsm6ds3_fifo_reset_start(imu_dev_handle);

            ESP_LOGI(IMU_TAG, "IMU started");
        }
        /* runs continously when logging data */
        else if (imu_task_cmd == CMD_IMU_READ_LOOP && prev_imu_task_cmd == CMD_IMU_READ_LOOP)
        {
            /* read one sample from IMU */
            if (imu_measure_calibrated(imu_dev_handle, &imu_data_buffer[0]) == 0)
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
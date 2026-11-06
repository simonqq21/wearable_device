/**
 * This task polls both the PPG heart rate sensor and IMU sensor, and
 * send the raw data to the DSP task.
 *
 */

#include "TaskSensors.h"

// GPIO task handle
extern TaskHandle_t task_handle_GPIO;

void stop_all_sensors(sensors_cmd_struct_t *sensors_cmds)
{
    atomic_store(&sensors_cmds->imu_cmd, CMD_IMU_STOP);
    atomic_store(&sensors_cmds->heart_rate_cmd, CMD_HEART_RATE_STOP);
}

void task_sensors(void *params)
{
    /* FIFO buffer storage for lsm6ds3 */
    lsm6ds3_data_t lsm6ds3_fifo_buffer[1];
    /* IMU data buffer */
    imu_raw_data_t imu_data_buffer[1];
    /* heart rate sensor data buffer */
    heart_rate_raw_data_t msg;

    /* state machine vars */
    cmd_imu_t imu_cmd = CMD_IMU_STOP;
    cmd_imu_t prev_imu_cmd = CMD_IMU_STOP;
    cmd_heart_rate_t heart_rate_cmd = CMD_HEART_RATE_STOP;
    cmd_heart_rate_t prev_heart_rate_cmd = CMD_HEART_RATE_STOP;

    /* IMU task delay period */
    TickType_t sensors_task_delay_period;
    // imu_raw_data_t imu_data;

    /* parse freeRTOS primitives from params */
    params_task_sensors_t *params_sensors_task = (params_task_sensors_t *)params;

    /* IMU data queue */
    QueueHandle_t queue_imu_sensor = params_sensors_task->queue_imu_sensor;
    /* heart rate data queue */
    QueueHandle_t queue_heart_rate_sensor = params_sensors_task->queue_heart_rate_sensor;
    /* i2c devices handles */
    i2c_master_dev_handle_t imu_dev_handle = params_sensors_task->imu_dev_handle;
    i2c_MAX30102_handle_t heart_rate_dev_handle = params_sensors_task->heart_rate_dev_handle;
    /* sensors cmd struct */
    sensors_cmd_struct_t *sensors_cmds = params_sensors_task->sensors_cmds;
    /* GPIO cmd struct */
    gpio_cmd_struct_t *gpio_cmds = (gpio_cmd_struct_t *)params_sensors_task->gpio_cmds;

    /* current timestamp */
    uint64_t cur_timestamp;
    /* number of samples read from the FIFO */
    uint16_t num_samples_read;

    /* Local to this task: MAX30102 driver hysteresis; published only via the queue. */
    int detect_flag = 1;
    int posted_gap = 0;

    /* raw heart rate data */
    heart_rate_raw_data_t heart_rate_raw_data;

    while (1)
    {
        /* load atomic IMU and heart rate commands */
        imu_cmd = (cmd_imu_t)atomic_load(&sensors_cmds->imu_cmd);
        heart_rate_cmd = (cmd_heart_rate_t)atomic_load(&sensors_cmds->heart_rate_cmd);

        /* variable delay */
        if (imu_cmd == CMD_IMU_START || heart_rate_cmd == CMD_HEART_RATE_START)
        {
            /* delay per IMU measurement */
            sensors_task_delay_period = IMU_LOGGING_TIMEDELTA_MS;
        }
        else
        {
            sensors_task_delay_period = 1000;
        }

        /* IMU state machine */
        /* runs when IMU calibration is initiated */
        if (imu_cmd == CMD_IMU_CALIBRATE)
        {
            /* stop all sensors */
            stop_all_sensors(sensors_cmds);
            /* notification LED fast blink */
            atomic_store(&gpio_cmds->status_led_cmd, CMD_STATUS_LED_FAST_BLINK);
            /* calibrate IMU */
            imu_calibrate(imu_dev_handle, lsm6ds3_fifo_buffer);
            /* notification LED off */
            atomic_store(&gpio_cmds->status_led_cmd, CMD_STATUS_LED_OFF);
            /* stop all sensors */
            stop_all_sensors(sensors_cmds);
        }

        /* runs when IMU is started */
        else if (imu_cmd == CMD_IMU_START && prev_imu_cmd == CMD_IMU_STOP)
        {
            /* set initial timestamp */
            cur_timestamp = esp_timer_get_time();
            ESP_LOGI(SENSORS_TAG, "IMU started at %lld\n", cur_timestamp);
        }

        /* runs continously when IMU is running */
        else if (imu_cmd == CMD_IMU_START && prev_imu_cmd == CMD_IMU_START)
        {
            /* read one sample from IMU */
            if (imu_measure_calibrated(imu_dev_handle, &imu_data_buffer[0]) == 0)
            {
                imu_data_buffer[0].timestamp = esp_timer_get_time();
                // ESP_LOGI(SENSORS_TAG, "calib IMU data %lld %.4f %.4f %.4f %.4f %.4f %.4f ",
                //          imu_data_buffer[0].timestamp,
                //          imu_data_buffer[0].ax,
                //          imu_data_buffer[0].ay,
                //          imu_data_buffer[0].az,
                //          imu_data_buffer[0].gx,
                //          imu_data_buffer[0].gy,
                //          imu_data_buffer[0].gz);

                /* send IMU data to queue */
                if (queue_imu_sensor != NULL)
                {
                    if (xQueueSend(queue_imu_sensor,
                                   &imu_data_buffer[0],
                                   (sensors_task_delay_period / portTICK_PERIOD_MS)) != pdTRUE)
                    {
                        ESP_LOGE(SENSORS_TAG, "ERROR: Could not put item on IMU queue.");
                    }
                }
            }
        }
        /* runs when IMU is stopped */
        else if (imu_cmd == CMD_IMU_STOP && prev_imu_cmd == CMD_IMU_START)
        {
            ESP_LOGI(SENSORS_TAG, "IMU stopped");
        }
        /* runs continuously while IMU is stopped */
        else if (imu_cmd == CMD_IMU_STOP && prev_imu_cmd == CMD_IMU_STOP)
        {
        }

        /* heart rate sensor state machine */
        /* runs when heart rate sensor is started */
        if (heart_rate_cmd == CMD_HEART_RATE_START && prev_heart_rate_cmd == CMD_HEART_RATE_STOP)
        {
            /* start heart rate sensor */
            cur_timestamp = esp_timer_get_time();
            ESP_LOGI(SENSORS_TAG, "heart rate started at %lld\n", cur_timestamp);
        }

        /* runs continously when heart rate sensor is running */
        else if (heart_rate_cmd == CMD_HEART_RATE_START && prev_heart_rate_cmd == CMD_HEART_RATE_START)
        {
            int got = 0;
            /* read one sample from the heart rate sensor */
            // esp_err_t err = i2c_MAX30102_read_red(heart_rate_dev_handle,
            //                                       0,
            //                                       &detect_flag,
            //                                       msg.samples,
            //                                       PPG_SAMPLE_RATE,
            //                                       &got);
            /* read one sample from the heart rate sensor */
            if (fifo_read_sample(heart_rate_dev_handle,
                                 &heart_rate_raw_data.red_val,
                                 &heart_rate_raw_data.ir_val) != ESP_OK)
            {
                ESP_LOGI(HEART_RATE_TAG, "failed to read the HR sensor");
                fifo_reset(heart_rate_dev_handle);
            }
            else
            {
                /* grab timestamp */
                heart_rate_raw_data.timestamp = esp_timer_get_time();
                /* send that sample to the heart rate queue */
                // ESP_LOGI(HEART_RATE_TAG, "red = %d, ir = %d",
                // heart_rate_raw_data.red_val,
                // heart_rate_raw_data.ir_val);
                if (xQueueSend(queue_heart_rate_sensor,
                               &heart_rate_raw_data,
                               sensors_task_delay_period / portTICK_PERIOD_MS) != pdTRUE)
                {
                    ESP_LOGE(SENSORS_TAG, "ERROR: Could not put item on heart rate queue.");
                }
            }
        }

        /* runs when heart rate sensor is stopped */
        else if (heart_rate_cmd == CMD_HEART_RATE_STOP && prev_heart_rate_cmd == CMD_HEART_RATE_START)
        {
            ESP_LOGI(SENSORS_TAG, "heart rate stopped");
        }

        /* runs continuously while heart rate sensor is stopped */
        else if (heart_rate_cmd == CMD_HEART_RATE_STOP && prev_heart_rate_cmd == CMD_HEART_RATE_STOP)
        {
        }

        prev_imu_cmd = imu_cmd;
        prev_heart_rate_cmd = heart_rate_cmd;

        vTaskDelay(sensors_task_delay_period / portTICK_PERIOD_MS);
    }
}
#include "TaskSensorFusion.h"

/**
 * @brief FreeRTOS task to get sensor data, perform sensor fusion, and distribute them to the SD card
 * datalogging task, UART data streaming task, and BLE data streaming task.
 *
 * The raw data is sent to the SD card, while the orientation data is sent to the UART, BLE, and SD card.
 */
void task_sensor_fusion(void *params)
{
    // state machine commands
    cmd_task_datalogging_t datalogging_task_cmd, prev_datalogging_task_cmd;
    // unified IMU data to send to downstream tasks
    imu_unified_data_t imu_unified_data;
    int64_t prev_timestamp = 0;

    /* datalogging task params */
    params_task_sensor_fusion_t *task_sensor_fusion_params = (params_task_sensor_fusion_t *)params;

    /* streambuffers */
    // StreamBufferHandle_t streambuffer_imu = task_sensor_fusion_params->streambuffer_imu;
    // StreamBufferHandle_t streambuffer_sd = task_sensor_fusion_params->streambuffer_sd;

    /* queues */
    QueueHandle_t queue_imu_sensor = task_sensor_fusion_params->queue_imu_sensor;
    QueueHandle_t queue_heart_rate_sensor = task_sensor_fusion_params->queue_heart_rate_sensor;
    QueueHandle_t queue_imu_sdcard = task_sensor_fusion_params->queue_imu_sdcard;
    QueueHandle_t queue_imu_UART = task_sensor_fusion_params->queue_imu_UART;
    QueueHandle_t queue_imu_BLE = task_sensor_fusion_params->queue_imu_BLE;
    QueueHandle_t queue_heart_rate_sdcard = task_sensor_fusion_params->queue_heart_rate_sdcard;
    QueueHandle_t queue_heart_rate_UART = task_sensor_fusion_params->queue_heart_rate_UART;
    QueueHandle_t queue_heart_rate_BLE = task_sensor_fusion_params->queue_heart_rate_BLE;

    int num_bytes_read = 0;
    uint16_t num_samples_read = 0;

    // imu_raw_data_t imu_data_buf[NUM_FIFO_TIMESTAMPS];

    /* FSM vars */
    prev_datalogging_task_cmd = CMD_DATALOGGING_STOP;
    datalogging_task_cmd = CMD_DATALOGGING_STOP;

    /* Fusion AHRS variables */
    const float sampleRate = IMU_ODR_HZ; // Hz

    // Calibration parameters (replace with actual calibration data)
    const FusionMatrix gyroscopeMisalignment = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
    const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};

    const FusionMatrix accelerometerMisalignment = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
    const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
    const FusionVector accelerometerOffset = {{0.0f, 0.0f, 0.0f}};

    // const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    // const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    /* initialize Fusion AHRS */
    // Instantiate AHRS algorithm
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .sampleRate = sampleRate,
        .convention = FusionConventionEnu,
        .gain = 0.5f,
        .gyroscopeRange = IMU_G_FS, /* replace with actual gyroscope range */
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .rejectionTimeout = 5.0f,
    };

    FusionAhrsSetSettings(&ahrs, &settings);

    // Instantiate bias algorithm
    FusionBias bias;
    FusionBiasInitialise(&bias);

    FusionBiasSettings biasSettings = fusionBiasDefaultSettings;
    biasSettings.sampleRate = sampleRate;

    FusionBiasSetSettings(&bias, &biasSettings);

    int decimation_counter = 0;
#define DECIMATION_FACTOR 8

    while (1)
    {
        /* wait for task notification from button */
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&datalogging_task_cmd,
                        5 / portTICK_PERIOD_MS);

        /* if datalogging was just started */
        if (datalogging_task_cmd == CMD_DATALOGGING_GO && prev_datalogging_task_cmd == CMD_DATALOGGING_STOP)
        {
            ESP_LOGI(DATALOGGING_TAG, "Datalogging started.");
            /* reset all queues */
            xQueueReset(queue_imu_sensor);
            xQueueReset(queue_imu_sdcard);
            xQueueReset(queue_imu_UART);
            xQueueReset(queue_imu_BLE);
            xQueueReset(queue_heart_rate_sensor);
            xQueueReset(queue_heart_rate_sdcard);
            xQueueReset(queue_heart_rate_UART);
            xQueueReset(queue_heart_rate_BLE);

            xQueueReceive(queue_imu_sensor, (void *)&imu_unified_data.imu_raw_data, portMAX_DELAY);
            /* initialize complementary filter */
            imu_unified_data.imu_orientation_data = complementary_filter(&imu_unified_data.imu_raw_data,
                                                                         imu_unified_data.imu_raw_data.timestamp - prev_timestamp,
                                                                         0.0);

            prev_timestamp = 0;
        }
        /* if datalogging is ongoing */
        if (datalogging_task_cmd == CMD_DATALOGGING_GO && prev_datalogging_task_cmd == CMD_DATALOGGING_GO)
        {
            // ESP_LOGI(DATALOGGING_TAG, "asdf %d", num_samples_read);
            /* receive raw IMU data from queue */
            if (xQueueReceive(queue_imu_sensor, (void *)&imu_unified_data.imu_raw_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
            {
                /* if this is not the first sample */
                if (prev_timestamp > 0)
                {
                    /* IMU complementary filter */
                    imu_unified_data.imu_orientation_data = complementary_filter(&imu_unified_data.imu_raw_data, imu_unified_data.imu_raw_data.timestamp - prev_timestamp, ALPHA);

                    ESP_LOGI(DATALOGGING_TAG, "complementary filter %lld %.2f %.2f %.2f",
                             imu_unified_data.imu_orientation_data.euler_angle.timestamp,
                             imu_unified_data.imu_orientation_data.euler_angle.x,
                             imu_unified_data.imu_orientation_data.euler_angle.y,
                             imu_unified_data.imu_orientation_data.euler_angle.z);

                    // /* Fusion AHRS sensor fusion */
                    // // Read sensors (replace with actual sensor data)
                    // const uint64_t timestamp = imu_raw_data.timestamp;
                    // FusionVector gyroscope = {{imu_raw_data.gx, imu_raw_data.gy, imu_raw_data.gz}};
                    // FusionVector accelerometer = {{imu_raw_data.ax, imu_raw_data.ay, imu_raw_data.az}};
                    // // FusionVector magnetometer = {1.0f, 0.0f, 0.0f};

                    // // Apply calibration
                    // gyroscope = FusionModelInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);

                    // accelerometer = FusionModelInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

                    // // magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);

                    // // Update bias algorithm
                    // gyroscope = FusionBiasUpdate(&bias, gyroscope);

                    // // Calculate delta time to compensate for gyroscope sample clock errors
                    // static uint64_t previousTimestamp = 0;
                    // const float deltaTime = (float)(timestamp - previousTimestamp);
                    // previousTimestamp = timestamp;

                    // FusionAhrsSetSamplePeriod(&ahrs, deltaTime);

                    // // Update AHRS algorithm
                    // // FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);
                    // FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

                    // // Print AHRS outputs
                    // const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

                    // const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

                    // // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
                    // //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
                    // //        earth.axis.x, earth.axis.y, earth.axis.z);

                    /* dummy orientation values */
                    /* if orientation format is in euler angles, stream euler angles. */
                    // if (ORIENTATION_FORMAT == ORIENTATION_EULER)
                    // {
                    //     /* dummy euler angle values */
                    //     imu_unified_data.imu_orientation_data.euler_angle.timestamp = imu_raw_data.timestamp;
                    //     imu_unified_data.imu_orientation_data.euler_angle.x = 30;
                    //     imu_unified_data.imu_orientation_data.euler_angle.y = 40;
                    //     imu_unified_data.imu_orientation_data.euler_angle.z = 50;
                    // }
                    // /* else if orientation format is in quaternions, stream quaternions. */
                    // else if (ORIENTATION_FORMAT == ORIENTATION_QUATERNION)
                    // {
                    //     /* dummy quaternion values */
                    //     imu_unified_data.imu_orientation_data.quaternion.timestamp = imu_raw_data.timestamp;
                    //     imu_unified_data.imu_orientation_data.quaternion.w = 1;
                    //     imu_unified_data.imu_orientation_data.quaternion.x = 2;
                    //     imu_unified_data.imu_orientation_data.quaternion.y = 3;
                    //     imu_unified_data.imu_orientation_data.quaternion.z = 5;
                    // }

                    /* send unified IMU data to SD card */
                    if (queue_imu_sdcard != NULL)
                    {
                        if (xQueueSend(queue_imu_sdcard, &imu_unified_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                        {
                            // ESP_LOGI(DATALOGGING_TAG, "send unified IMU data to SD card");
                        }
                        else
                        {
                            ESP_LOGE(DATALOGGING_TAG, "send unified IMU data to SD card fail");
                            xQueueReset(queue_imu_sdcard);
                        }
                    }

                    /* send unified IMU data to UART */
                    if (queue_imu_UART != NULL)
                    {
                        if (xQueueSend(queue_imu_UART, &imu_unified_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                        {
                            // ESP_LOGI(DATALOGGING_TAG, "sent unified IMU data to UART");
                        }
                        else
                        {
                            ESP_LOGE(DATALOGGING_TAG, "send unified IMU data to UART fail");
                            xQueueReset(queue_imu_UART);
                        }
                    }

                    /* send unified IMU data to BLE */
                    if (queue_imu_BLE != NULL)
                    {
                        if (decimation_counter == 0)
                        {
                            if (xQueueSend(queue_imu_BLE, &imu_unified_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                            {
                                // ESP_LOGI(DATALOGGING_TAG, "sent unified IMU data to BLE");
                            }
                            else
                            {
                                ESP_LOGE(DATALOGGING_TAG, "send unified IMU data to BLE fail");
                                xQueueReset(queue_imu_BLE);
                            }
                        }
                        decimation_counter = (decimation_counter + 1) % DECIMATION_FACTOR;
                    }

                    num_samples_read = (num_samples_read + 1) % NUM_FIFO_TIMESTAMPS;
                }
                prev_timestamp = imu_unified_data.imu_raw_data.timestamp;
            }

            /* test FIFO overflow */
            // vTaskDelay(1000 / portTICK_PERIOD_MS);

            // num_bytes_read = xStreamBufferReceive(streambuffer_imu,
            //                                       &imu_data_buf,
            //                                       sizeof(imu_raw_data_t) * NUM_FIFO_TIMESTAMPS,
            //                                       100 / portTICK_PERIOD_MS);

            // if (num_bytes_read > 0)
            // {

            //     if (streambuffer_sd != NULL)
            //         if (xStreamBufferSend(streambuffer_sd,
            //                               &imu_data_buf,
            //                               num_bytes_read,
            //                               10 / portTICK_PERIOD_MS) == 0)
            //         {
            //             ESP_LOGE(DATALOGGING_TAG, "ERROR: Could not put item on SD stream buffer.");
            //         }
            //     // // send data to UART data streaming queue
            //     // if (queue_UART != NULL)
            //     //     xQueueSend(queue_UART, &imu_raw_data, 10 / );
            //     // // send data to BLE data streaming queue
            //     // if (queue_BLE != NULL)
            //     //     xQueueSend(queue_BLE, &imu_raw_data, 10 / );

            //     // display IMU data
            //     // ESP_LOGI(DATALOGGING_TAG, "timestamp=%d", imu_raw_data.timestamp);
            //     // ESP_LOGI(DATALOGGING_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_raw_data.ax, imu_raw_data.ay, imu_raw_data.az);
            //     // ESP_LOGI(DATALOGGING_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_raw_data.gx, imu_raw_data.gy, imu_raw_data.gz);
            //     // ESP_LOGI(DATALOGGING_TAG, "Temperature:  %.1f\n", imu_raw_data.temp);
            // }
        }

        /* if datalogging was just stopped */
        if (datalogging_task_cmd == CMD_DATALOGGING_STOP && prev_datalogging_task_cmd == CMD_DATALOGGING_GO)
        {
            ESP_LOGI(DATALOGGING_TAG, "Datalogging stopped.");
        }
        /* if datalogging is not ongoing*/
        if (datalogging_task_cmd == CMD_DATALOGGING_STOP && prev_datalogging_task_cmd == CMD_DATALOGGING_STOP)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        prev_datalogging_task_cmd = datalogging_task_cmd;
    }
}

#include "datalogging.h"
#include "Fusion.h"

/**
 * @brief FreeRTOS task to get sensor data, perform sensor fusion, and distribute them to the SD card
 * datalogging task, UART data streaming task, and BLE data streaming task.
 *
 * The raw data is sent to the SD card, while the orientation data is sent to the UART, BLE, and SD card.
 */
#define DATALOGGING_TAG "DATA LOGGING"

void task_main_datalogging(void *params)
{
    cmd_task_datalogging_t datalogging_task_cmd, prev_datalogging_task_cmd;
    // uint8_t imu_data_received;
    imu_data_t imu_data;

    /* datalogging task params */
    task_main_datalogging_params_t *datalogging_params = (task_main_datalogging_params_t *)params;

    /* streambuffers */
    StreamBufferHandle_t streambuffer_imu = datalogging_params->streambuffer_imu;
    StreamBufferHandle_t streambuffer_sd = datalogging_params->streambuffer_sd;

    /* queues */
    QueueHandle_t queue_imu = datalogging_params->queue_imu;
    QueueHandle_t queue_raw_sdcard = datalogging_params->queue_raw_sdcard;
    QueueHandle_t queue_orientation_sdcard = datalogging_params->queue_orientation_sdcard;
    QueueHandle_t queue_orientation_UART = datalogging_params->queue_orientation_UART;
    QueueHandle_t queue_orientation_BLE = datalogging_params->queue_orientation_BLE;

    int num_bytes_read = 0;
    uint16_t num_samples_read = 0;

    // imu_data_t imu_data_buf[NUM_FIFO_TIMESTAMPS];

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

    while (1)
    {
        /* wait for task notification from button */
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&datalogging_task_cmd,
                        5 / portTICK_PERIOD_MS);

        // if (xTaskNotifyWait(0,
        //                     0,
        //                     (uint32_t *)&temp,
        //                     5 / portTICK_PERIOD_MS) != pdTRUE)
        // {
        //     datalogging_task_cmd = temp;
        // }
        /* if datalogging was just started */
        if (datalogging_task_cmd == CMD_DATALOGGING_GO && prev_datalogging_task_cmd == CMD_DATALOGGING_STOP)
        {
            ESP_LOGI(DATALOGGING_TAG, "Datalogging started.");
        }
        /* if datalogging is ongoing */
        if (datalogging_task_cmd == CMD_DATALOGGING_GO && prev_datalogging_task_cmd == CMD_DATALOGGING_GO)
        {
            /* receive IMU data from queue */
            if (xQueueReceive(queue_imu, (void *)&imu_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
            {

                /* send data to SD card datalogger queue */
                if (queue_raw_sdcard != NULL)
                {
                    if (xQueueSend(queue_raw_sdcard, &imu_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
                    {
                        if (num_samples_read == NUM_FIFO_TIMESTAMPS - 1)
                        {
                            ESP_LOGI(DATALOGGING_TAG, "%d raw samples sent to SD card", num_samples_read);
                        }
                    }
                    else
                    {
                        ESP_LOGI(DATALOGGING_TAG, "error sending data to SD card queue");
                    }
                }

                /* Fusion AHRS sensor fusion */
                // Read sensors (replace with actual sensor data)
                const uint64_t timestamp = imu_data.timestamp;
                FusionVector gyroscope = {{imu_data.gx, imu_data.gy, imu_data.gz}};
                FusionVector accelerometer = {{imu_data.ax, imu_data.ay, imu_data.az}};
                // FusionVector magnetometer = {1.0f, 0.0f, 0.0f};

                // Apply calibration
                gyroscope = FusionModelInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);

                accelerometer = FusionModelInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

                // magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);

                // Update bias algorithm
                gyroscope = FusionBiasUpdate(&bias, gyroscope);

                // Calculate delta time to compensate for gyroscope sample clock errors
                static uint64_t previousTimestamp = 0;
                const float deltaTime = (float)(timestamp - previousTimestamp);
                previousTimestamp = timestamp;

                FusionAhrsSetSamplePeriod(&ahrs, deltaTime);

                // Update AHRS algorithm
                // FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);
                FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

                // Print AHRS outputs
                const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

                const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

                // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
                //        euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
                //        earth.axis.x, earth.axis.y, earth.axis.z);

                /* send orientation data to SD card */
                /* send orientation data to UART */
                /* send orientation data to BLE */

                num_samples_read = (num_samples_read + 1) % NUM_FIFO_TIMESTAMPS;
            }

            // num_bytes_read = xStreamBufferReceive(streambuffer_imu,
            //                                       &imu_data_buf,
            //                                       sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS,
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
            //     //     xQueueSend(queue_UART, &imu_data, 10 / );
            //     // // send data to BLE data streaming queue
            //     // if (queue_BLE != NULL)
            //     //     xQueueSend(queue_BLE, &imu_data, 10 / );

            //     // display IMU data
            //     // ESP_LOGI(DATALOGGING_TAG, "timestamp=%d", imu_data.timestamp);
            //     // ESP_LOGI(DATALOGGING_TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", imu_data.ax, imu_data.ay, imu_data.az);
            //     // ESP_LOGI(DATALOGGING_TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", imu_data.gx, imu_data.gy, imu_data.gz);
            //     // ESP_LOGI(DATALOGGING_TAG, "Temperature:  %.1f\n", imu_data.temp);
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

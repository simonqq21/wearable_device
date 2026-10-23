#include "TaskSDCard.h"

/**
 * @brief SD card datalogging subscriber task
 *
 * Grab data sent from the IMU datalogging publisher task to log to the SD card.
 * Create and open new binary file with given index
 */
void task_SD_card_datalogger(void *params)
{
    // buffer for forming filenames and filepaths
    char filename[32];
    char filepath[40];

    /* SD car task FSM vars */
    cmd_task_sd_card_datalogging_t cmd_task_sd_card_datalogging = CMD_SD_CARD_STOP;
    cmd_task_sd_card_datalogging_t prev_cmd_task_sd_card_datalogging = CMD_SD_CARD_STOP;

    /* parse freeRTOS primitives from params */
    params_task_SD_card_datalogger_t *sd_datalogger_params = (params_task_SD_card_datalogger_t *)params;
    QueueHandle_t queue_imu_sdcard = sd_datalogger_params->queue_imu_sdcard;
    QueueHandle_t queue_heart_rate_sdcard = sd_datalogger_params->queue_heart_rate_sdcard;

    /* raw IMU data file handle */
    FILE *f_raw = NULL;
    unsigned long raw_file_size = 0;

    /* orientation data file handle */
    FILE *f_orientation = NULL;
    unsigned long orientation_file_size = 0;

    // struct stat st;
    /* highest datalog index */
    int max_idx = 0;

    /* buffers for raw IMU data */
    imu_unified_data_t imu_unified_data;
    imu_raw_data_t imu_raw_data_buf[SD_BUF_SIZE];
    imu_orientation_data_t imu_orientation_data_buf[SD_BUF_SIZE];

    /* current number of samples in current file */
    uint16_t num_imu_file_samples = 0;
    /* number of raw samples read */
    uint16_t imu_buffer_idx = 0;

    while (1)
    {
        // check for the signal to stop datalogging, which will save the file.
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&cmd_task_sd_card_datalogging, (2 / portTICK_PERIOD_MS));

        /*
         run once when stop signal received

         if stop signal received, save and close the files.
        */
        if (cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_START)
        {
            ESP_LOGI(SD_CARD_TAG, "IMU raw data file size: %d", raw_file_size);
            ESP_LOGI(SD_CARD_TAG, "IMU orientation file size: %d", orientation_file_size);
            // close IMU raw data file
            f_raw = close_file(f_raw, &raw_file_size);
            // close orientation data file
            f_orientation = close_file(f_orientation, &raw_file_size);
            ESP_LOGI(SD_CARD_TAG, "Files written.");
        }

        /*
        run once when start signal received

        if start signal received, open new files.
        */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_START && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP)
        {
            /* reset  */
            // num_bytes_read = 0;
            imu_buffer_idx = 0;
            num_imu_file_samples = 0;

            /* if SD card not initialized, attempt to mount it. */
            if (!sd_card_is_initialized())
            {
                sd_card_configure_wrapper();
            }

            /* get the latest datalog file index and add one to it for the new datalog file */
            if (f_raw == NULL || f_orientation == NULL)
            {
                max_idx = get_highest_datalog_idx(MOUNT_POINT);
                max_idx++;
            }

            // if no file is open, create a new file.
            if (f_raw == NULL)
            {
                raw_file_size = 0;
                /* create filename IMU_RAW_DATA_FILENAME_FORMAT */
                sprintf(filename, IMU_RAW_DATA_FILENAME_FORMAT, max_idx);
                ESP_LOGI(SD_CARD_TAG, "filename: %s", filename);
                /* create filepath */
                sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                ESP_LOGI(SD_CARD_TAG, "filepath: %s", filepath);
                /* create and write new binary file */
                ESP_LOGI(SD_CARD_TAG, "Writing to new raw file");
                f_raw = open_file(f_raw, filepath, &raw_file_size);
            }

            if (f_orientation == NULL)
            {
                orientation_file_size = 0;
                /* create filename IMU_ORIENTATION_FILENAME_FORMAT */
                sprintf(filename, IMU_ORIENTATION_FILENAME_FORMAT, max_idx);
                ESP_LOGI(SD_CARD_TAG, "filename: %s", filename);
                /* create filepath */
                sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                ESP_LOGI(SD_CARD_TAG, "filepath: %s", filepath);
                /* create and write new binary file */
                ESP_LOGI(SD_CARD_TAG, "Writing to new orientation file");
                f_orientation = open_file(f_orientation, filepath, &orientation_file_size);
            }
        }

        /* run continuously while datalogging is ongoing */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_START && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_START)
        {

            /* if SD card not initialized, attempt to mount it. */
            if (!sd_card_is_initialized())
            {
                sd_card_configure_wrapper();
            }

            /* get the latest datalog file index and add one to it for the new datalog file */
            if (f_raw == NULL || f_orientation == NULL)
            {
                max_idx = get_highest_datalog_idx(MOUNT_POINT);
                max_idx++;
            }

            /* while the number of recorded IMU samples is less than the maximum file length */
            if (num_imu_file_samples < IMU_MAX_SAMPLES_PER_FILE)
            {
                /* if there is no IMU raw data file open, create a new raw data file. */
                if (f_raw == NULL)
                {
                    raw_file_size = 0;
                    /* create filename IMU_RAW_DATA_FILENAME_FORMAT */
                    sprintf(filename, IMU_RAW_DATA_FILENAME_FORMAT, max_idx);
                    ESP_LOGI(SD_CARD_TAG, "IMU raw data filename: %s", filename);
                    /* create filepath */
                    sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                    ESP_LOGI(SD_CARD_TAG, "IMU raw data filepath: %s", filepath);
                    /*  create and write new binary file */
                    ESP_LOGI(SD_CARD_TAG, "Writing to new raw data file");
                    f_raw = open_file(f_raw, filepath, &raw_file_size);
                }

                /* if there is no IMU orientation data file open, create a new orientation data file. */
                if (f_orientation == NULL)
                {
                    orientation_file_size = 0;
                    /* create filename IMU_RAW_DATA_FILENAME_FORMAT */
                    sprintf(filename, IMU_ORIENTATION_FILENAME_FORMAT, max_idx);
                    ESP_LOGI(SD_CARD_TAG, "orientation data filename: %s", filename);
                    /* create filepath */
                    sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                    ESP_LOGI(SD_CARD_TAG, "orientation data filepath: %s", filepath);
                    /*  create and write new binary file */
                    ESP_LOGI(SD_CARD_TAG, "Writing to new orientation data file");
                    f_orientation = open_file(f_orientation, filepath, &orientation_file_size);
                }

                /* receive one IMU unified data from the IMU queue */
                if (xQueueReceive(queue_imu_sdcard, &imu_unified_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    //     ESP_LOGI(SD_CARD_TAG, "received data: ");
                    //     ESP_LOGI(SD_CARD_TAG, "Accel %.4f, %.4f, %.4f", imu_unified_data.ax, imu_unified_data.ay, imu_unified_data.az);
                    //     ESP_LOGI(SD_CARD_TAG, "Gyro %.4f, %.4f, %.4f", imu_unified_data.gx, imu_unified_data.gy, imu_unified_data.gz);
                    //     ESP_LOGI(SD_CARD_TAG, "Temp %.2f", imu_unified_data.temp);

                    /* copy data to buffer */
                    memcpy(&imu_raw_data_buf[imu_buffer_idx],
                           &imu_unified_data.imu_raw_data,
                           sizeof(imu_raw_data_t));
                    memcpy(&imu_orientation_data_buf[imu_buffer_idx],
                           &imu_unified_data.imu_orientation_data,
                           sizeof(imu_orientation_data_t));

                    /* write buffer data to card */
                    if (imu_buffer_idx == SD_BUF_SIZE - 1)
                    {
                        write_imu_raw_data_to_card(f_raw,
                                                   imu_raw_data_buf,
                                                   imu_buffer_idx,
                                                   &raw_file_size);
                        write_imu_orientation_data_to_card(f_orientation,
                                                           imu_orientation_data_buf,
                                                           imu_buffer_idx,
                                                           &orientation_file_size);
                    }

                    /* increment indices */
                    imu_buffer_idx = (imu_buffer_idx + 1) % SD_BUF_SIZE;
                    num_imu_file_samples++;
                }
            }
            /* once the number of IMU file samples exceeds the maximum IMU file sample count */
            else
            {
                ESP_LOGI(SD_CARD_TAG, "IMU raw data file size: %d", raw_file_size);
                f_raw = close_file(f_raw, &raw_file_size);
                ESP_LOGI(SD_CARD_TAG, "IMU orientation file size: %d", orientation_file_size);
                f_orientation = close_file(f_orientation, &orientation_file_size);

                /* reset the number of samples for the current file to 0 */
                num_imu_file_samples = 0;
                ESP_LOGI(SD_CARD_TAG, "Files written.");
            }
        }

        /* run continuously while datalogging is stopped */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP)
        {
            /* do nothing */
            vTaskDelay((100 / portTICK_PERIOD_MS));
        }

        prev_cmd_task_sd_card_datalogging = cmd_task_sd_card_datalogging;
    }
}
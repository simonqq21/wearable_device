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
    /* queue to receive raw IMU data from the main datalogger task */
    QueueHandle_t queue_raw_sdcard = sd_datalogger_params->queue_raw_sdcard;
    /* queue to receive orientation data from the main datalogger task */
    QueueHandle_t queue_orientation_sdcard = sd_datalogger_params->queue_orientation_sdcard;
    // StreamBufferHandle_t streambuffer_sd = sd_datalogger_params->streambuffer_sd;

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
    imu_data_t rcv_imu_data;
    imu_data_t imu_raw_data_buf[SD_BUF_SIZE];
    /* buffers for orientation data */
    orientation_data_t rcv_orientation_data;
    orientation_data_t orientation_data_buf[SD_BUF_SIZE];

    // int num_bytes_read = 0;
    /* number of raw samples read */
    uint16_t num_raw_data_samples_read = 0;
    /* number of orientation samples read */
    uint16_t num_orientation_data_samples_read = 0;

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
            num_raw_data_samples_read = 0;
            num_orientation_data_samples_read = 0;

            // if SD card not initialized, attempt to mount it.
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

            // ESP_LOGI(SD_CARD_TAG, "t3");

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

            // ESP_LOGI(SD_CARD_TAG, "t4");

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

            /* while the file is less than the maximum raw data file size */
            if (raw_file_size < MAX_IMU_RAW_DATA_FILE_SIZE)
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

                /* read raw IMU data from the queue */
                if (xQueueReceive(queue_raw_sdcard, &rcv_imu_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    //     ESP_LOGI(SD_CARD_TAG, "received data: ");
                    //     ESP_LOGI(SD_CARD_TAG, "Accel %.4f, %.4f, %.4f", rcv_imu_data.ax, rcv_imu_data.ay, rcv_imu_data.az);
                    //     ESP_LOGI(SD_CARD_TAG, "Gyro %.4f, %.4f, %.4f", rcv_imu_data.gx, rcv_imu_data.gy, rcv_imu_data.gz);
                    //     ESP_LOGI(SD_CARD_TAG, "Temp %.2f", rcv_imu_data.temp);

                    /* copy data to buffer */
                    memcpy(&imu_raw_data_buf[num_raw_data_samples_read],
                           &rcv_imu_data,
                           sizeof(imu_data_t));

                    /* write buffer data to card */
                    if (num_raw_data_samples_read == SD_BUF_SIZE - 1)
                    {
                        write_imu_raw_data_to_card(f_raw,
                                                   imu_raw_data_buf,
                                                   num_raw_data_samples_read,
                                                   &raw_file_size);
                    }

                    num_raw_data_samples_read = (num_raw_data_samples_read + 1) % SD_BUF_SIZE;
                }
            }
            /* once the raw data file reaches the maximum datalog file size */
            else
            {
                ESP_LOGI(SD_CARD_TAG, "IMU raw data file size: %d", raw_file_size);
                f_raw = close_file(f_raw, &raw_file_size);
                ESP_LOGI(SD_CARD_TAG, "File written.");
            }

            /* while the file is less than the maximum orientation data file size */
            if (orientation_file_size < MAX_IMU_ORIENTATION_DATA_FILE_SIZE)
            {
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

                /* read orientation data from the queue */
                if (xQueueReceive(queue_orientation_sdcard, &rcv_orientation_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    /* copy data to buffer */
                    memcpy(&orientation_data_buf[num_orientation_data_samples_read],
                           &rcv_orientation_data,
                           sizeof(orientation_data_t));

                    /* write buffer data to card */
                    if (num_orientation_data_samples_read == SD_BUF_SIZE - 1)
                    {
                        write_orientation_data_to_card(f_orientation,
                                                       orientation_data_buf,
                                                       num_orientation_data_samples_read,
                                                       &orientation_file_size);
                    }

                    num_orientation_data_samples_read = (num_orientation_data_samples_read + 1) % SD_BUF_SIZE;
                }
            }
            /* once the orientation file reaches the maximum datalog file size */
            else
            {
                ESP_LOGI(SD_CARD_TAG, "IMU orientation file size: %d", orientation_file_size);
                f_orientation = close_file(f_orientation, &orientation_file_size);
                ESP_LOGI(SD_CARD_TAG, "File written.");
            }
        }

        /*
        run continuously while datalogging is stopped
        */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP)
        {
            /* do nothing */
            vTaskDelay((100 / portTICK_PERIOD_MS));
        }

        prev_cmd_task_sd_card_datalogging = cmd_task_sd_card_datalogging;
    }
}
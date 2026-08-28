#include "sd_card.h"

const int IMU_EXPECTED_LEN = strlen("imu_000000.bin");
uint8_t sd_card_initialized = 0;
extern QueueHandle_t queue_imu;

/**
 * @brief SD card datalogging subscriber task
 *
 * Grab data sent from the IMU datalogging publisher task to log to the SD card.
 * Create and open new binary file with given index
 *
 * @param params params_task_SD_card_datalogger_t
 */
void task_SD_card_datalogger(void *params)
{
    cmd_task_sd_card_datalogging_t cmd_task_sd_card_datalogging = CMD_SD_CARD_STOP;
    cmd_task_sd_card_datalogging_t prev_cmd_task_sd_card_datalogging = CMD_SD_CARD_STOP;

    /* parse freeRTOS primitives from params */
    params_task_SD_card_datalogger_t *sd_datalogger_params = (params_task_SD_card_datalogger_t *)params;
    QueueHandle_t queue_sdcard = sd_datalogger_params->queue_sdcard;
    StreamBufferHandle_t streambuffer_sd = sd_datalogger_params->streambuffer_sd;

    char filename[20];
    char filepath[30];
    FILE *f = NULL;
    // struct stat st;
    int max_idx;
    unsigned long file_size = 0;

    imu_data_t rcv_imu_data;
    imu_data_t imu_data_buf[NUM_FIFO_TIMESTAMPS];
    int num_bytes_read = 0;
    uint16_t num_samples_read = 0;

    while (1)
    {
        // check for the signal to stop datalogging, which will save the file.
        xTaskNotifyWait(0,
                        0,
                        (uint32_t *)&cmd_task_sd_card_datalogging, (2 / portTICK_PERIOD_MS));

        /*
         run once when stop signal received

         if stop signal received, save and close the file.
        */
        if (cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_START)
        {
            f = close_file(f, &file_size);
            ESP_LOGI(SD_CARD_TAG, "File written.");
            ESP_LOGI(SD_CARD_TAG, "file size: %d", file_size);
        }

        /*
        run once when start signal received

        if start signal received, open a new file..
        */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_START && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_STOP)
        {
            /* reset  */
            num_bytes_read = 0;
            num_samples_read = 0;

            // if no file is open, create a new file.
            if (f == NULL)
            { // if SD card not initialized, attempt to mount it.
                if (!sd_card_is_initialized())
                {
                    sd_card_configure_wrapper();
                }
                file_size = 0;
                /* get the latest datalog file index and add one to it for the new datalog file */
                max_idx = get_latest_datalog_idx(MOUNT_POINT);
                max_idx++;
                /* create filename IMU_FILENAME_FORMAT */
                sprintf(filename, "imu_%06d.bin", max_idx);
                ESP_LOGI(SD_CARD_TAG, "filename: %s", filename);
                /* create filepath */
                sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                ESP_LOGI(SD_CARD_TAG, "filepath: %s", filepath);
                /* create and write new binary file */
                ESP_LOGI(SD_CARD_TAG, "Writing to new file");
                f = open_file(f, filepath, &file_size);
            }
        }

        /* run continuously while datalogging is ongoing */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_START && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_START)
        {
            /* while the file is less than the maximum datalog file size */
            if (file_size < MAX_DATALOG_FILE_SIZE)
            {
                /* if no file is open, create a new file. */
                if (f == NULL)
                {
                    /* if SD card not initialized, attempt to mount it. */
                    if (!sd_card_is_initialized())
                    {
                        sd_card_configure_wrapper();
                    }
                    file_size = 0;
                    /* get the latest datalog file index and add one to it for the new datalog file */
                    max_idx = get_latest_datalog_idx(MOUNT_POINT);
                    max_idx++;
                    /* create filename IMU_FILENAME_FORMAT */
                    sprintf(filename, "imu_%06d.bin", max_idx);
                    ESP_LOGI(SD_CARD_TAG, "filename: %s", filename);
                    /* create filepath */
                    sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
                    ESP_LOGI(SD_CARD_TAG, "filepath: %s", filepath);
                    /*  create and write new binary file */
                    ESP_LOGI(SD_CARD_TAG, "Writing to new file");
                    f = open_file(f, filepath, &file_size);
                }

                /* read from the queue */
                if (xQueueReceive(queue_sdcard, &rcv_imu_data, 10 / portTICK_PERIOD_MS) == pdTRUE)
                {
                    //     ESP_LOGI(SD_CARD_TAG, "received data: ");
                    //     ESP_LOGI(SD_CARD_TAG, "Accel %.4f, %.4f, %.4f", rcv_imu_data.ax, rcv_imu_data.ay, rcv_imu_data.az);
                    //     ESP_LOGI(SD_CARD_TAG, "Gyro %.4f, %.4f, %.4f", rcv_imu_data.gx, rcv_imu_data.gy, rcv_imu_data.gz);
                    //     ESP_LOGI(SD_CARD_TAG, "Temp %.2f", rcv_imu_data.temp);

                    /* copy data to buffer */
                    memcpy(&imu_data_buf[num_samples_read], &rcv_imu_data, sizeof(imu_data_t));

                    /* write buffer data to card */
                    if (num_samples_read == NUM_FIFO_TIMESTAMPS - 1)
                    {
                        write_imu_data_to_card(f, imu_data_buf, num_samples_read, &file_size);
                    }

                    num_samples_read = (num_samples_read + 1) % NUM_FIFO_TIMESTAMPS;
                }

                // // read from stream buffer
                // num_bytes_read = xStreamBufferReceive(streambuffer_sd,
                //                                       imu_data_buf,
                //                                       sizeof(imu_data_t) * NUM_FIFO_TIMESTAMPS,
                //                                       (100 / portTICK_PERIOD_MS));
                // if (num_bytes_read > 0)
                // {
                //     ESP_LOGI(SD_CARD_TAG, "%d bytes to write", num_bytes_read);
                //     // for (int i = 0; i < num_bytes_read / sizeof(imu_data_t); i++)
                //     write_imu_data_to_card(f, &imu_data_buf, num_bytes_read / sizeof(imu_data_t), &file_size);
                // }
            }
            /* once the file reaches the maximum datalog file size */
            else
            {
                f = close_file(f, &file_size);
                ESP_LOGI(SD_CARD_TAG, "File written.");
                ESP_LOGI(SD_CARD_TAG, "file size: %d", file_size);
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

/**
 * @brief configure SD card
 *
 */
esp_err_t sd_configure(void)
{
    sd_card_initialized = 0;
    esp_err_t ret;
    // SD card mount configuration
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(SD_CARD_TAG, "Initializing SD card");
    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(SD_CARD_TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = 5000;

    // SD card SPI bus configuration
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SD_CARD_TAG, "Failed to initialize bus.");
        return ret;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = host.slot;

    ESP_LOGI(SD_CARD_TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(SD_CARD_TAG, "Failed to mount filesystem. "
                                  "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(SD_CARD_TAG, "Failed to initialize the card (%s). "
                                  "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
            // check_sd_card_pins(&config, pin_count);
        }
        return ret;
    }
    ESP_LOGI(SD_CARD_TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    sd_card_initialized = 1;
    return ESP_OK;
}

/**
 * @brief wrapper to configure SD card
 */
esp_err_t sd_card_configure_wrapper(void)
{
    esp_err_t ret = ESP_OK;
    if (REQUIRE_SD_CARD && !sd_card_is_initialized())
    {
        ret = sd_configure();
        if (ret != ESP_OK)
        {
            ESP_LOGE(SD_CARD_TAG, "SD card initialization failed");
        }
        else
        {
            ESP_LOGI(SD_CARD_TAG, "SD card initialized");
            return ret;
            // break;
        }
        // }
    }
    return ret;
}

/**
 * @brief returns if the SD card was initialized successfully.
 *
 * @return whether the SD card is initialized
 */
uint8_t sd_card_is_initialized(void)
{
    return sd_card_initialized;
}

/**
 * @brief get latest file index on SD card
 *
 * the format of the files is "imu_%06d.bin", idx
 * eg. imu_000000.bin
 *
 * @param *dir_path the path to the datalog file destination
 * @return the greatest idx existent in the dir.
 * if none exist, return -1.
 */
int get_latest_datalog_idx(const char *dir_path)
{
    int max_idx = -1;
    // Open the directory stream
    DIR *dir = opendir(dir_path);

    // Check if the directory opened successfully
    if (dir == NULL)
    {
        perror("Unable to open directory");
        return -2;
    }

    struct dirent *entry;
    printf("Listing files in directory: %s\n", dir_path);
    printf("-----------------------------------\n");

    // Read directory entries sequentially
    while ((entry = readdir(dir)) != NULL)
    {
        // Filter out the navigation shortcuts "." and ".." if desired
        if (entry->d_name[0] == '.' && (entry->d_name[1] == '\0' ||
                                        (entry->d_name[1] == '.' && entry->d_name[2] == '\0')))
        {
            continue;
        }

        // Print the file/folder name
        // printf("%s\n", entry->d_name);
        if (strlen(entry->d_name) == IMU_EXPECTED_LEN)
        {
            int file_number;
            char trailing_chars[2] = {0};
            sscanf(entry->d_name, IMU_SCAN_FORMAT, &file_number, trailing_chars);
            if (file_number > max_idx)
            {
                max_idx = file_number;
            }
            //
        }
    }

    // Close the directory stream
    closedir(dir);
    ESP_LOGI(SD_CARD_TAG, "max_idx: %d", max_idx);
    return max_idx;
}

// /*
// get latest file index on SD card
// */
// void list_directory_files(const char *dir_path)
// {
//     // Open the directory stream
//     DIR *dir = opendir(dir_path);

//     // Check if the directory opened successfully
//     if (dir == NULL)
//     {
//         perror("Unable to open directory");
//         return;
//     }

//     struct dirent *entry;
//     printf("Listing files in directory: %s\n", dir_path);
//     printf("-----------------------------------\n");

//     // Read directory entries sequentially
//     while ((entry = readdir(dir)) != NULL)
//     {
//         // Filter out the navigation shortcuts "." and ".." if desired
//         if (entry->d_name[0] == '.' && (entry->d_name[1] == '\0' ||
//                                         (entry->d_name[1] == '.' && entry->d_name[2] == '\0')))
//         {
//             continue;
//         }

//         // Print the file/folder name
//         printf("%s\n", entry->d_name);
//     }

//     // Close the directory stream
//     closedir(dir);
// }

/**
 * @brief read an IMU datalog file given an index
 *
 * @param idx index of the datalog file
 */
esp_err_t read_datalog_file(int idx)
{
    char filename[15];
    char filepath[30];
    FILE *f;
    struct stat st;
    unsigned int num_records = 0;
    unsigned int batch_size = 8;
    /* generate filename from idx */
    sprintf(filename, IMU_FILENAME_FORMAT, idx);
    /* generate filepath from filename */
    sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
    /* read and stat the newly created binary file */
    num_records = 0;
    imu_data_t imu_data_2[batch_size];
    /* open the binary file */
    f = fopen(filepath, "rb");
    if (f == NULL)
    {
        ESP_LOGE(SD_CARD_TAG, "Failed to open file for reading");
        return 1;
    }
    unsigned int elements_read = 0;
    /* keep reading IMU data structs from the file as long as there is still data to read */
    while ((elements_read = fread(imu_data_2, sizeof(imu_data_t), batch_size, f)) == batch_size)
    {
        num_records += elements_read;
    }
    if (elements_read > 0)
    {
        // Process the final partial chunk of data (elements_read bytes)
        num_records += elements_read;
    }
    /* print file size and the number of records read from file */
    stat(filepath, &st);
    ESP_LOGI(SD_CARD_TAG, "file size: %d", st.st_size);
    ESP_LOGI(SD_CARD_TAG, "num_records read from file: %d", num_records);
    fclose(f);
    return 0;
}

/**
 * @brief wrapper to close an opened file on an SD card
 *
 * @param f file pointer to close
 * @param file_size for zeroing the file size variable
 */
FILE *close_file(FILE *f, unsigned long *file_size)
{
    *file_size = 0;
    if (REQUIRE_SD_CARD)
    {
        if (f != NULL)
        {
            fclose(f);
        }
    }
    f = NULL;
    return f;
}

/**
 * @brief wrapper to open a file on an SD card
 *
 * @param f file pointer
 * @param file_size for zeroing the file size variable
 */
FILE *open_file(FILE *f, char *filepath, unsigned long *file_size)
{
    *file_size = 0;
    if (REQUIRE_SD_CARD)
    {
        f = fopen(filepath, "wb");

        if (f == NULL)
        {
            ESP_LOGE(SD_CARD_TAG, "Failed to open file for writing");
        }
        else
        {
            ESP_LOGI(SD_CARD_TAG, "FILE IS NOT NULL");
        }
    }
    return f;
}

/**
 * @brief wrapper for writing an IMU data struct to the SD card
 *
 * @param f file pointer
 * @param rcv_imu_data pointer to array of IMU data structs
 * @param num_elements number of elements to write to card
 * @param file_size file size to increment
 */
void write_imu_data_to_card(FILE *f, imu_data_t *rcv_imu_data, int num_elements, unsigned long *file_size)
{
    if (REQUIRE_SD_CARD)
    {
        if (f != NULL)
        {
            fwrite(rcv_imu_data, sizeof(imu_data_t), num_elements, f);
            *file_size = ftell(f);
        }
        else
            ESP_LOGE(SD_CARD_TAG, "file is null");
    }
}

/*
datalogging strategy:
when datalogging started, create a new file.
Each file is large enough to hold 10 minutes worth of 50 Hz IMU data
4 bytes / float * 6 axes = 24 bytes per struct.
50 times per second * 24 bytes = 1200 bytes / second
1200 bytes / second * 60 * 10 = 720000 bytes = 703.125 kilobytes
when the file size reaches 705 kB (approx. 10 mins IMU recording), close the file and open a new file.
filename: "imu_{n}.bin" where n is a zero-padded number padded to 5 digits.
eg. imu_000000.bin, imu_000001.bin, ...
*/
#include "sd_card.h"

const int IMU_EXPECTED_LEN = strlen("imu_000000.bin");
uint8_t sd_card_initialized = 0;
extern QueueHandle_t queue_imu;

// buffer for forming filenames and filepaths
char filename[32];
char filepath[40];

/**
 * @brief SD card datalogging subscriber task
 *
 * Grab data sent from the IMU datalogging publisher task to log to the SD card.
 * Create and open new binary file with given index
 */
void task_SD_card_datalogger(void *params)
{
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

            // ESP_LOGI(SD_CARD_TAG, "t1");

            // if SD card not initialized, attempt to mount it.
            if (!sd_card_is_initialized())
            {
                sd_card_configure_wrapper();
            }

            // ESP_LOGI(SD_CARD_TAG, "t2");

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
            // ESP_LOGI(SD_CARD_TAG, "t5");
        }

        /* run continuously while datalogging is ongoing */
        else if (cmd_task_sd_card_datalogging == CMD_SD_CARD_START && prev_cmd_task_sd_card_datalogging == CMD_SD_CARD_START)
        {
            // ESP_LOGI(SD_CARD_TAG, "t6");

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

            // ESP_LOGI(SD_CARD_TAG, "t7");

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

                // // read from stream buffer
                // num_bytes_read = xStreamBufferReceive(streambuffer_sd,
                //                                       imu_data_buf,
                //                                       sizeof(imu_data_t) * SD_BUF_SIZE,
                //                                       (100 / portTICK_PERIOD_MS));
                // if (num_bytes_read > 0)
                // {
                //     ESP_LOGI(SD_CARD_TAG, "%d bytes to write", num_bytes_read);
                //     // for (int i = 0; i < num_bytes_read / sizeof(imu_data_t); i++)
                //     write_imu_raw_data_to_card(f_raw, &imu_data_buf, num_bytes_read / sizeof(imu_data_t), &file_size);
                // }
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

/**
 * @brief initialize and configure SD card
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
    host.max_freq_khz = 4000;

    // SD card SPI bus configuration
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 5000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(SD_CARD_TAG, "Failed to initialize bus: %d", ret);
        // return ret;
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
        sd_card_initialized = 0;
    }
    else
    {
        ESP_LOGI(SD_CARD_TAG, "Filesystem mounted");
        // Card has been initialized, print its properties
        sdmmc_card_print_info(stdout, card);
        sd_card_initialized = 1;
    }

    return ret;
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
 * Find the highest index of the IMU raw datalog files and
 * the orientation files. Get the highest index among the two to
 * avoid overwriting any existing raw data/orientation files.
 *
 * @param *dir_path the path to the datalog file destination
 * @return the greatest idx existent in the dir.
 * if none exist, return -1.
 */
int get_highest_datalog_idx(const char *dir_path)
{
    int max_idx = -1;
    /* Open the directory stream */
    DIR *dir = opendir(dir_path);
    int file_number;
    char trailing_chars[2] = {0};

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
        printf("%s\n", entry->d_name);

        // if (strlen(entry->d_name) == IMU_EXPECTED_LEN)

        /* if the file matches the raw data filename prefix */
        if (strncmp(IMU_RAW_DATA_FILENAME_PREFIX, entry->d_name, strlen(IMU_RAW_DATA_FILENAME_PREFIX)) == 0)
        {

            sscanf(entry->d_name, IMU_RAW_DATA_SCAN_FORMAT, &file_number, trailing_chars);
            if (file_number > max_idx)
            {
                max_idx = file_number;
            }
        }

        /* if the filename matches the orientation data filename prefix */
        if (strncmp(IMU_ORIENTATION_FILENAME_PREFIX, entry->d_name, strlen(IMU_ORIENTATION_FILENAME_PREFIX)) == 0)
        {
            sscanf(entry->d_name, IMU_ORIENTATION_SCAN_FORMAT, &file_number, trailing_chars);
            if (file_number > max_idx)
            {
                max_idx = file_number;
            }
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
    FILE *f_raw;
    struct stat st;
    unsigned int num_records = 0;
    unsigned int batch_size = 8;
    /* generate filename from idx */
    sprintf(filename, IMU_RAW_DATA_FILENAME_FORMAT, idx);
    /* generate filepath from filename */
    sprintf(filepath, "%s/%s", MOUNT_POINT, filename);
    /* read and stat the newly created binary file */
    num_records = 0;
    imu_data_t imu_data_2[batch_size];
    /* open the binary file */
    f_raw = fopen(filepath, "rb");
    if (f_raw == NULL)
    {
        ESP_LOGE(SD_CARD_TAG, "Failed to open file for reading");
        return 1;
    }
    unsigned int elements_read = 0;
    /* keep reading IMU data structs from the file as long as there is still data to read */
    while ((elements_read = fread(imu_data_2,
                                  sizeof(imu_data_t),
                                  batch_size,
                                  f_raw)) == batch_size)
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
    fclose(f_raw);
    return 0;
}

/**
 * @brief wrapper to close an opened file on an SD card
 *
 * @param f_raw file pointer to close
 * @param file_size for zeroing the file size variable
 */
FILE *close_file(FILE *f_raw, unsigned long *file_size)
{
    *file_size = 0;
    if (REQUIRE_SD_CARD)
    {
        if (f_raw != NULL)
        {
            fclose(f_raw);
        }
    }
    f_raw = NULL;
    return f_raw;
}

/**
 * @brief wrapper to open a file on an SD card
 *
 * @param f_raw file pointer
 * @param file_size for zeroing the file size variable
 */
FILE *open_file(FILE *f_raw, char *filepath, unsigned long *file_size)
{
    *file_size = 0;
    if (REQUIRE_SD_CARD)
    {
        f_raw = fopen(filepath, "wb");

        if (f_raw == NULL)
        {
            ESP_LOGE(SD_CARD_TAG, "Failed to open file for writing");
        }
        else
        {
            ESP_LOGI(SD_CARD_TAG, "FILE IS NOT NULL");
        }
    }
    return f_raw;
}

/**
 * @brief wrapper for writing an IMU data struct to the SD card
 *
 * @param f_raw file pointer
 * @param rcv_imu_data pointer to array of IMU data structs
 * @param num_elements number of elements to write to card
 * @param file_size file size to increment
 */
void write_imu_raw_data_to_card(FILE *f_raw,
                                imu_data_t *rcv_imu_data,
                                int num_elements,
                                unsigned long *file_size)
{
    if (REQUIRE_SD_CARD)
    {
        if (f_raw != NULL)
        {
            fwrite(rcv_imu_data, sizeof(imu_data_t), num_elements, f_raw);
            *file_size = ftell(f_raw);
        }
        else
            ESP_LOGE(SD_CARD_TAG, "file is null");
    }
}

/**
 * @brief wrapper for writing an orientation data struct to the SD card
 *
 * @param f_raw file pointer
 * @param orientation_data pointer to array of orientation data structs
 * @param num_elements number of elements to write to card
 * @param file_size file size to increment
 */
void write_orientation_data_to_card(FILE *f_raw,
                                    orientation_data_t *orientation_data,
                                    int num_elements,
                                    unsigned long *file_size)
{
    if (REQUIRE_SD_CARD)
    {
        if (f_raw != NULL)
        {

            fwrite(orientation_data, sizeof(orientation_data_t), num_elements, f_raw);
            *file_size = ftell(f_raw);
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
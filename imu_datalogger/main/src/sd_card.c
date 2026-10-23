#include "sd_card.h"

uint8_t sd_card_initialized = 0;
extern QueueHandle_t queue_imu_sensor;

/**
 * @brief initialize and configure SD card
 *
 */
esp_err_t sd_configure(void)
{
    sd_card_initialized = 0;
    esp_err_t ret;
    /*
    SD card mount configuration
    Options for mounting the filesystem.
    If format_if_mount_failed is set to true, SD card will be partitioned and
    formatted in case when mounting fails.
    */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(SD_CARD_TAG, "Initializing SD card");
    /*
    Use settings defined above to initialize SD card and mount FAT filesystem.
    Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    Please check its source code and implement error recovery when developing
    production applications.
    */
    ESP_LOGI(SD_CARD_TAG, "Using SPI peripheral");

    /*
    By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    */
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

    /*
    This initializes the slot without card detect (CD) and write protect (WP) signals.
    Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    */
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

        // // Print the file/folder name
        // printf("%s\n", entry->d_name);

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

/**
 * @brief read an IMU datalog file given an index
 *
 * @param idx index of the datalog file
 */
esp_err_t read_datalog_file(int idx)
{
    // buffer for forming filenames and filepaths
    char filename[32];
    char filepath[40];
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
    imu_raw_data_t imu_data_2[batch_size];
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
                                  sizeof(imu_raw_data_t),
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

// /**
//  *
//  */
// void write_imu_unified_data_to_card(
//     FILE *f_raw,
//     imu_unified_data_t *imu_unified_data,
//     int num_elements,
//     unsigned long *imu_raw_file_size,
//     unsigned long *imu_orientation_file_size)
// {
//     write_imu_raw_data_to_card(f_raw,
//                                imu_unified_data->imu_raw_data,
//                                num_elements,
//                                imu_raw_file_size);
//     write_imu_orientation_data_to_card(f_raw,
//                                        imu_unified_data->imu_orientation_data,
//                                        num_elements,
//                                        imu_orientation_file_size);
// }

/**
 * @brief wrapper for writing an IMU data struct to the SD card
 *
 * @param f_raw file pointer
 * @param imu_raw_data pointer to array of IMU data structs
 * @param num_elements number of elements to write to card
 * @param file_size file size to increment
 */
void write_imu_raw_data_to_card(FILE *f_raw,
                                imu_raw_data_t *imu_raw_data,
                                int num_elements,
                                unsigned long *file_size)
{
    if (REQUIRE_SD_CARD)
    {
        if (f_raw != NULL)
        {
            fwrite(imu_raw_data, sizeof(imu_raw_data_t), num_elements, f_raw);
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
 * @param imu_orientation_data pointer to array of orientation data structs
 * @param num_elements number of elements to write to card
 * @param file_size file size to increment
 */
void write_imu_orientation_data_to_card(FILE *f_raw,
                                        imu_orientation_data_t *imu_orientation_data,
                                        int num_elements,
                                        unsigned long *file_size)
{
    if (REQUIRE_SD_CARD)
    {
        if (f_raw != NULL)
        {

            fwrite(imu_orientation_data, sizeof(imu_orientation_data_t), num_elements, f_raw);
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
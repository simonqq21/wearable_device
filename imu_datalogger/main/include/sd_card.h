#ifndef SD_H
#define SD_H

#include "common.h"

#define SD_CARD_TAG "SD_CARD"
/* SD card mount point */
#define MOUNT_POINT "/sdcard"

#define SD_BUF_SIZE 100

/**
 * SD card datalog file formats
 * %1s specifies a string conversion with a maximum field width of 1 character.
 * %1s is used for trailing newline here
 */
#define IMU_RAW_DATA_FILENAME_PREFIX "imu_raw_"
#define IMU_RAW_DATA_FILENAME_FORMAT IMU_RAW_DATA_FILENAME_PREFIX "%06d.bin"
#define IMU_RAW_DATA_SCAN_FORMAT IMU_RAW_DATA_FILENAME_PREFIX "%6d.bin%1s"
#define IMU_ORIENTATION_FILENAME_PREFIX "imu_orientation_"
#define IMU_ORIENTATION_FILENAME_FORMAT IMU_ORIENTATION_FILENAME_PREFIX "%06d.bin"
#define IMU_ORIENTATION_SCAN_FORMAT IMU_ORIENTATION_FILENAME_PREFIX "%06d.bin%1s"
/* maximum SD card datalog file size */
#define MAX_IMU_RAW_DATA_FILE_SIZE (sizeof(imu_data_t) * IMU_ODR_HZ * DATALOG_FILE_DURATION_S)
#define MAX_IMU_ORIENTATION_DATA_FILE_SIZE (sizeof(orientation_data_t) * IMU_ODR_HZ * DATALOG_FILE_DURATION_S)

/**
 * @brief parameters for the SD card datalogging task
 *
 * @param queue_raw_sdcard queue to receive raw IMU data from the main datalogger task
 * @param queue_orientation_sdcard queue to receive orientation data from the main datalogger task
 */
typedef struct
{
    QueueHandle_t queue_raw_sdcard;
    QueueHandle_t queue_orientation_sdcard;
    StreamBufferHandle_t streambuffer_sd;
} params_task_SD_card_datalogger_t;

void task_SD_card_datalogger(void *params);
esp_err_t sd_configure(void);
esp_err_t sd_card_configure_wrapper(void);
uint8_t sd_card_is_initialized(void);
int get_highest_datalog_idx(const char *dir_path);
esp_err_t read_datalog_file(int idx);
FILE *close_file(FILE *f, unsigned long *file_size);
FILE *open_file(FILE *f, char *filepath, unsigned long *file_size);
void write_imu_raw_data_to_card(FILE *f, imu_data_t *rcv_imu_data, int num_elements, unsigned long *file_size);
void write_orientation_data_to_card(FILE *f, orientation_data_t *orientation_data, int num_elements, unsigned long *file_size);
esp_err_t sd_card_configure_wrapper(void);

#endif
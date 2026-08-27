#ifndef SD_H
#define SD_H

#include "common.h"

/* SD card pins */
#define SD_MISO_PIN 4
#define SD_MOSI_PIN 15
#define SD_SCK_PIN 14
#define SD_CS_PIN 13

#define SD_CARD_TAG "SD_CARD"
/* SD card mount point */
#define MOUNT_POINT "/sdcard"

/* SD card datalog file formats */
#define IMU_FILENAME_FORMAT "imu_%06d.bin"
#define IMU_SCAN_FORMAT "imu_%6d.bin%1s"
/* maximum SD card datalog file size */
#define MAX_DATALOG_FILE_SIZE (sizeof(imu_data_t) * IMU_ODR_HZ * DATALOG_FILE_DURATION_S)

/**
 * @brief parameters for the SD card datalogging task
 *
 * @param streambuffer_sd stream buffer to receive data from the main datalogger task
 */
typedef struct
{
    QueueHandle_t queue_sdcard;
    StreamBufferHandle_t streambuffer_sd;
} task_SD_card_datalogger_params_t;

void task_SD_card_datalogger(void *params);
esp_err_t sd_configure(void);
esp_err_t sd_card_configure_wrapper(void);
uint8_t sd_card_is_initialized(void);
int get_latest_datalog_idx(const char *dir_path);
esp_err_t read_datalog_file(int idx);
FILE *close_file(FILE *f, unsigned long *file_size);
FILE *open_file(FILE *f, char *filepath, unsigned long *file_size);
void write_imu_data_to_card(FILE *f, imu_data_t *rcv_imu_data, int num_elements, unsigned long *file_size);
void write_imu_data_to_card(FILE *f, imu_data_t *rcv_imu_data, int num_elements, unsigned long *file_size);
esp_err_t sd_card_configure_wrapper(void);

#endif
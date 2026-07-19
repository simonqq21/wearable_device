#ifndef SD_H
#define SD_H

#include "common.h"

#define SD_MISO_PIN 4
#define SD_MOSI_PIN 15
#define SD_SCK_PIN 14
#define SD_CS_PIN 13

#define SD_CARD_TAG "SD_CARD"
#define MOUNT_POINT "/sdcard"

#define IMU_FILENAME_FORMAT "imu_%06d.bin"
#define IMU_SCAN_FORMAT "imu_%6d.bin%1s"
#define MAX_DATALOG_FILE_SIZE (sizeof(imu_data_t) * IMU_SAMPLE_RATE_HZ * DATALOG_FILE_DURATION_S)

/**
 * @brief configure SD card
 *
 *
 */
esp_err_t sd_configure(void);

/**
 * @brief returns if the SD card was initialized successfully.
 *
 */
uint8_t sd_card_is_initialized(void);

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
int get_latest_datalog_idx(const char *dir_path);

/**
 * @brief read an IMU datalog file given an index
 */
esp_err_t read_datalog_file(int idx);

/**
 * @brief SD card datalogging subscriber task
 *
 * Grab data sent from the IMU datalogging publisher task to log to the SD card.
 * Create and open new binary file with given index
 */
// static void task_SD_card_datalogger(void *params);

#endif
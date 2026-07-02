#ifndef NVS_H
#define NVS_H

#include "common.h"

#define NVS_TAG "NVS"

void nvs_init(void);
esp_err_t NVS_write_imu_calibration_offsets(imu_calibration_offsets_t *offsets);
esp_err_t NVS_read_imu_calibration_offsets(imu_calibration_offsets_t *offsets);

#endif
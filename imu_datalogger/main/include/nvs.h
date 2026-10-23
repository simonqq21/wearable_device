#ifndef NVS_H
#define NVS_H

#include "common.h"

#define NVS_TAG "NVS"

// ESP32 NVS config
#define IMU_OFFSETS_NVS_KEY "imu_offsets"
#define NVS_NAMESPACE "NVS"

/* IMU calibration offsets struct */
typedef struct
{
    float oAx, oAy, oAz;
    float oGx, oGy, oGz;
} imu_calibration_offsets_t;

void nvs_init(void);
esp_err_t NVS_write_imu_calibration_offsets(imu_calibration_offsets_t *offsets);
esp_err_t NVS_read_imu_calibration_offsets(imu_calibration_offsets_t *offsets);

#endif
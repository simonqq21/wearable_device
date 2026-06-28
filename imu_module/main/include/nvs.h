#ifndef NVS_H
#define NVS_H

#include <esp_err.h>
#include <esp_log.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#define IMU_OFFSETS_NVS_KEY "imu_offsets"
#define NVS_NAMESPACE "NVS"
#define NVS_TAG "NVS"

typedef struct
{
    float oAx;
    float oAy;
    float oAz;
    float oGx;
    float oGy;
    float oGz;
} imu_calibration_offsets_t;

void nvs_init(void);
esp_err_t NVS_write_imu_calibration_offsets(imu_calibration_offsets_t *offsets);
esp_err_t NVS_read_imu_calibration_offsets(imu_calibration_offsets_t *offsets);

#endif
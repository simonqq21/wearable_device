#ifndef NVS_H
#define NVS_H

#include "common.h"
#include "haptics_lib.h"

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

/**
 * @brief initialize NVS
 *
 */
void nvs_init(void);

/**
 * @brief write IMU calibration offsets to the NVS
 *
 * @param
 */
esp_err_t NVS_write_imu_calibration_offsets(imu_calibration_offsets_t *offsets);

/**
 * @brief read IMU calibration offsets from the NVS
 */
esp_err_t NVS_read_imu_calibration_offsets(imu_calibration_offsets_t *offsets);

/**
 * @brief write haptics array sequence to the NVS
 *
 * @param
 */
esp_err_t NVS_write_haptics_sequence(char *key,
                                     haptic_pulse_t *haptics_pulse_sequence,
                                     uint8_t len);

/**
 * @brief read haptics array from the NVS
 *
 * @param
 */
esp_err_t NVS_read_haptics_sequence(char *key,
                                    haptic_pulse_t *haptics_pulse_sequence,
                                    uint8_t len);

#endif
#ifndef DATALOGGING_H
#define DATALOGGING_H

#include "common.h"
#include "Fusion.h"
#include "imu.h"

#define DATALOGGING_TAG "DATALOGGING"

/**
 * @brief IMU complementary filter
 *
 * @param imu_data calibrated IMU data
 * @param dt time delta between IMU data samples
 * @param alpha complementary filter parameter
 */
imu_orientation_data_t complementary_filter(imu_raw_data_t *imu_data, int dt, float alpha);

#endif
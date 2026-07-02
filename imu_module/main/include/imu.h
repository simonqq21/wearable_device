#ifndef IMU_H
#define IMU_H

#include "../include/nvs.h"
#include "../include/uart.h"
#include "common.h"

#define IMU_TAG "IMU"

void mpu6050_init_custom(uint8_t calibrate_imu);
esp_err_t mpu6050_measure_raw(imu_data_t *data);
esp_err_t mpu6050_measure(imu_data_t *data);
void imu_task(void *pvParameters);
void mpu6050_calibrate_user(void);
#endif
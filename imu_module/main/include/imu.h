#ifndef IMU_H
#define IMU_H

#include <mpu6050.h>
#include "../include/nvs.h"
#include "../include/uart.h"
#include "common.h"

#include <esp_err.h>
#include <esp_log.h>
//
// i2c
// ****************************************************************
#define SDA_PIN 21
#define SCL_PIN 22
// ****************************************************************
#define IMU_TAG "IMU"

typedef struct
{
    mpu6050_acceleration_t acc;
    mpu6050_rotation_t rot;
    float temp;
} imu_data_t;

void mpu6050_init_custom(uint8_t calibrate_imu);
esp_err_t mpu6050_measure_raw(imu_data_t *data);
esp_err_t mpu6050_measure(imu_data_t *data);
void imu_read_task(void *pvParameters);
void mpu6050_calibrate_user(void);
#endif
#ifndef IMU_H
#define IMU_H

#include "../include/nvs.h"
#include "../include/uart.h"
#include "common.h"
#include "math.h"

#define IMU_TAG "IMU"

#define SDA_PIN 21
#define SCL_PIN 22

// ESP32 IMU MPU6050 config
// #ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
// #else
// #define ADDR MPU6050_I2C_ADDRESS_HIGH
// #endif

// NVS offset read write function callback typedef
typedef esp_err_t (*nvs_func_t)(imu_calibration_offsets_t *);

void imu_set_offset_read_cb(nvs_func_t cb);
void imu_set_offset_write_cb(nvs_func_t cb);
void imu_init_custom(uint8_t calibrate_imu);
esp_err_t imu_measure_raw(imu_data_t *data);
esp_err_t imu_measure(imu_data_t *data);
void mpu6050_preprocess(imu_data_t *data);
// void task_imu(void *pvParameters);
void imu_calibrate_user(void);
#endif
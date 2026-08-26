#ifndef CONFIG_H
#define CONFIG_H

#define REQUIRE_SD_CARD 1

// FreeRTOS primitives config

// #define IMU_QUEUE_LEN (IMU_ODR_HZ * 2)
// #define IMU_SAMPLE_PERIOD_MS (1000 / IMU_ODR_HZ)
#define DATALOG_FILE_DURATION_S (60)
#endif
#ifndef CONFIG_H
#define CONFIG_H

#define REQUIRE_SD_CARD 1

// FreeRTOS primitives config

// ESP32 SD card config
#define MOUNT_POINT "/sdcard"
/* number of timestamps that can fit in the FIFO at once */
#define NUM_FIFO_TIMESTAMPS 50
#define LEN_QUEUE_HAPTICS 25

/* logging frequency of the IMU */
#define IMU_ODR_HZ 208
/* IMU accelerometer full scale */
#define IMU_XL_FS 4
/* IMU gyroscope full scale*/
#define IMU_G_FS 1000
/* maximum duration per datalog file in seconds */
#define IMU_FILE_DURATION (60)
/* maximum number of IMU samples per file */
#define IMU_MAX_SAMPLES_PER_FILE (IMU_ODR_HZ * IMU_FILE_DURATION)

/* Set ORIENTATION_FORMAT to 1 to stream quaternions, or 0 to stream euler angles */
#define ORIENTATION_EULER 0
#define ORIENTATION_QUATERNION 1
#define ORIENTATION_FORMAT ORIENTATION_EULER

#define ALPHA 0.98
#define GYRO_DEADBAND 0.01f
#endif
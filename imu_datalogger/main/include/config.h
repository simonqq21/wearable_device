#ifndef CONFIG_H
#define CONFIG_H

#define REQUIRE_SD_CARD 1

// FreeRTOS primitives config

// ESP32 SD card config
#define MOUNT_POINT "/sdcard"
/* number of timestamps that can fit in the FIFO at once */
#define NUM_FIFO_TIMESTAMPS 100

/* logging frequency of the IMU */
#define IMU_ODR_HZ 104
/* IMU accelerometer full scale */
#define IMU_XL_FS 4
/* IMU gyroscope full scale*/
#define IMU_G_FS 500
/* maximum duration per datalog file in seconds */
#define DATALOG_FILE_DURATION_S (60)
/* Set ORIENTATION_FORMAT to 1 to stream quaternions, or 0 to stream euler angles */
#define ORIENTATION_EULER 0
#define ORIENTATION_QUATERNION 1
#define ORIENTATION_FORMAT ORIENTATION_EULER

#endif
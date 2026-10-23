#include "datalogging.h"

imu_orientation_data_t complementary_filter(imu_raw_data_t *imu_data, int dt, float alpha)
{
    imu_orientation_data_t complementary_filter_result;
    float acc_roll, acc_pitch;
    static float gyro_roll = 0.0, gyro_pitch = 0.0, gyro_yaw = 0.0;

    /* get angles from accelerometer
    roll happens around the x-axis, pitch happens around the y-axis, and yaw happens around the z-axis. */
    acc_roll = atan2f(imu_data->ay,
                      sqrt(imu_data->ax * imu_data->ax + imu_data->az * imu_data->az)) *
               180.0 / M_PI;
    acc_pitch = atan2f(imu_data->ax,
                       sqrt(imu_data->ay * imu_data->ay + imu_data->az * imu_data->az)) *
                180.0 / M_PI;

    if (alpha == 0.0)
    {
        gyro_roll = acc_roll;
        gyro_pitch = acc_pitch;
        gyro_yaw = 0;
    }
    /* integrate gyroscope angles */
    else
    {
        if (fabsf(imu_data->gx) < GYRO_DEADBAND)
        {
            imu_data->gx = 0;
        }
        else
        {
            gyro_roll += imu_data->gx * dt / 1000000.0;
        }
        if (fabsf(imu_data->gy) < GYRO_DEADBAND)
        {
            imu_data->gy = 0;
        }
        else
        {
            gyro_pitch += imu_data->gy * dt / 1000000.0;
        }
        if (fabsf(imu_data->gz) < GYRO_DEADBAND)
        {
            imu_data->gz = 0;
        }
        else
        {
            gyro_yaw += imu_data->gz * dt / 1000000.0;
        }
    }

    /* accel only */
    // complementary_filter_result.euler_angle.timestamp = imu_data->timestamp;
    // complementary_filter_result.euler_angle.x = acc_roll;
    // complementary_filter_result.euler_angle.y = acc_pitch;
    // complementary_filter_result.euler_angle.z = gyro_yaw;

    /* combine the accelerometer and gyroscope angles */
    gyro_roll = alpha * gyro_roll + (1.0 - alpha) * acc_roll;
    gyro_pitch = alpha * gyro_pitch + (1.0 - alpha) * acc_pitch;
    complementary_filter_result.euler_angle.timestamp = imu_data->timestamp;
    complementary_filter_result.euler_angle.x = gyro_roll;
    complementary_filter_result.euler_angle.y = gyro_pitch;
    complementary_filter_result.euler_angle.z = gyro_yaw;

    return complementary_filter_result;
}

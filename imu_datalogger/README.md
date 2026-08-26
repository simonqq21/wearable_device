# ESP32 IMU Module Full Example

## Hardware
- ESP32 development board
- MPU6050 6-axis IMU 
    - record inertial measurement signals with the accelerometer and gyroscope 
- SD card slot
    - record inertial measurement signals with the accelerometer and gyroscope     
- button
    - initiate calibration, start-stop logging
- LED
    - indicate status and actions

## ESP-IDF Components
- MPU6050
    - idf.py add-dependency "espressif/mpu6050^1.2.1"
- NVS 
    - store small values like 6-axis IMU calibration values
    - 
- littleFS
    - record inertial measurement signals with the accelerometer and gyroscope 
    - idf.py add-dependency joltwallet/littlefs==1.22.1
- SD card
    - record inertial measurement signals with the accelerometer and gyroscope 

- Button
    - handle debouncing and button event callbacks
    - idf.py add-dependency "espressif/button=*"
- GPIO
    - 

# Proposed Hardware Specs
- ESP32
- 2 buttons (customizable) 
- 3 LEDs (initialization OK, datalogging/calibration)
- IMU
- heart rate sensor 
- SD card slot
- small MOSFETs for driving haptics?

Pinouts:
- SD card 
    - SD MISO - GPIO2 with 10k pullup resistor
    - SD CS - GPIO13 with 10k pullup resistor
    - D SCK - GPIO14 
    - SD MOSI - GPIO15  with 10k pullup resistor
- I2C bus
    - **HRM and IMU**
    - SDA - GPIO21 
    - SCL - GPIO22
- LEDs
    - GPIO25, GPIO26, GPIO27
- Buttons
    - GPIO18, GPIO19
- Motors
    - 2, 5, 12, 22, 23, 32, 33, 34, 35 , 


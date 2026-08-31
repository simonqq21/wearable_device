#ifndef PINS_H
#define PINS_H

/* SD card pins */
#define SD_MISO_PIN 4
#define SD_MOSI_PIN 15
#define SD_SCK_PIN 14
#define SD_CS_PIN 13
/* button pins */
#define BTN1_PIN 18
#define BTN2_PIN 19
/* LED pins */
#define LED1_PIN 25 // recording LED
#define LED2_PIN 26 //
#define LED3_PIN 27
/* i2c pins */
#define I2C_MASTER_SCL_PIN 22 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_PIN 21 /*!< GPIO number used for I2C master data  */
/* UART pins */
#define UART_RX_PIN 3
#define UART_TX_PIN 1
#define UART_STREAM_TX_PIN 23 // for streaming orientation data via UART
#define UART_STREAM_RX_PIN 35 // for streaming orientation data via UART

#endif
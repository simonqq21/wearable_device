#ifndef UART_H
#define UART_H

#include "common.h"

// ESP32 UART config
#define UART_PORT_NUM 0
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 128

esp_err_t uart_configure(void);

#endif
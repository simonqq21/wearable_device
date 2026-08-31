#ifndef UART_H
#define UART_H

#include "common.h"
#include "driver/uart.h"

#define UART_TAG "UART"

// ESP32 UART config
#define UART_PORT_NUM 0
#define UART_STREAMING_PORT_NUM 1
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE 128

/**
 * @brief parameters for UART task
 *
 * @param queue_orientation_UART IMU orientation queue sent to UART
 */
typedef struct
{
    QueueHandle_t queue_orientation_UART;
} params_task_uart_t;

esp_err_t uart_configure(void);
void task_uart_streaming(void *params);

uint16_t cobs_encode(char *src, char *dst, uint16_t len);
uint16_t cobs_decode(char *src, char *dst, uint16_t len);

#endif
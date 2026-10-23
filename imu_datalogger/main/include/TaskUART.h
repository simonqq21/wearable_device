#ifndef TASK_UART_H
#define TASK_UART_H

#include "uart.h"
#include "imu.h"

/**
 * @brief parameters for UART task
 *
 * @param queue_imu_UART incoming queue of unified IMU values
 * @param queue_heart_rate_UART incoming queue of unified heart rate values
 */
typedef struct
{
    QueueHandle_t queue_imu_UART;
    QueueHandle_t queue_heart_rate_UART;
} params_task_uart_t;

void task_uart_streaming(void *params);

#endif
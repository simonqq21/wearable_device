#include "TaskUART.h"

/**
 * @brief UART task
 *
 * This task streams orientation data via UART.
 *
 * Each orientation data is a union with either an Euler angle or a quaternion.
 *
 */
void task_uart_streaming(void *params)
{
    /* orientation data */
    imu_unified_data_t imu_unified_data;
    uint16_t encoded_bytes_len;
    /* COBS-encoded bytes buffer */
    char encoded_bytes_buf[sizeof(imu_unified_data_t) + 5];
    /* UART task parameters */
    params_task_uart_t *params_task_uart = (params_task_uart_t *)params;

    QueueHandle_t queue_imu_UART = params_task_uart->queue_imu_UART;
    QueueHandle_t queue_heart_rate_UART = params_task_uart->queue_heart_rate_UART;

    while (1)
    {
        /* receive orientation values from the UART orientation queue */
        if (xQueueReceive(queue_imu_UART, &imu_unified_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            /* COBS-encode the orientation data */
            encoded_bytes_len = cobs_encode((char *)&imu_unified_data, encoded_bytes_buf, sizeof(imu_unified_data_t));
            // ESP_LOGI(UART_TAG, "enc_bytes_len = %d\n", encoded_bytes_len);
            /* stream the COBS-encoded orientation data out the UART port */
            uart_write_bytes(UART_STREAMING_PORT_NUM, encoded_bytes_buf, encoded_bytes_len);
        }
    }
}
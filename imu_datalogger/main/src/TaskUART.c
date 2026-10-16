#include "TaskUART.h"

/**
 * @brief UART task
 *
 * This task streams orientation data via UART.
 *
 * Each orientation data is a union with either an Euler angle or a quaternion.
 *
 * @param queue_orientation_UART queue of incoming orientation data from the
 *  main datalogging task
 *
 */
void task_uart_streaming(void *params)
{
    /* orientation data */
    orientation_data_t rcv_orientation_data;
    uint16_t encoded_bytes_len;
    /* COBS-encoded bytes buffer */
    char encoded_bytes_buf[sizeof(orientation_data_t) + 3];
    /* UART task parameters */
    params_task_uart_t *params_task_uart = (params_task_uart_t *)params;
    QueueHandle_t queue_orientation_UART = params_task_uart->queue_orientation_UART;

    while (1)
    {
        /* receive orientation values from the UART orientation queue */
        if (xQueueReceive(queue_orientation_UART, &rcv_orientation_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            /* COBS-encode the orientation data */
            encoded_bytes_len = cobs_encode((char *)&rcv_orientation_data, encoded_bytes_buf, sizeof(orientation_data_t));
            // ESP_LOGI(UART_TAG, "enc_bytes_len = %d\n", encoded_bytes_len);
            /* stream the COBS-encoded orientation data out the UART port */
            uart_write_bytes(UART_STREAMING_PORT_NUM, encoded_bytes_buf, encoded_bytes_len);
        }
    }
}
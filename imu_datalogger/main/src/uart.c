#include "../include/uart.h"

esp_err_t uart_configure(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    /**
     * If you want to use USB, set TXD to 1 and RXD to 3.
     */
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, 1, 3, 0, 0));
    return ESP_OK;
}

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
    params_task_uart_t *params_uart_task = (params_task_uart_t *)params;
    QueueHandle_t queue_orientation_UART = params_uart_task->queue_orientation_UART;

    while (1)
    {
        /* receive orientation values from the UART orientation queue */
        if (xQueueReceive(queue_orientation_UART, &rcv_orientation_data, 100 / portTICK_PERIOD_MS) == pdTRUE)
        {
            /* COBS-encode the orientation data */
            encoded_bytes_len = cobs_encode((char *)&rcv_orientation_data, encoded_bytes_buf, sizeof(orientation_data_t));
            /* stream the COBS-encoded orientation data out the UART port */
            uart_write_bytes(UART_PORT_NUM, encoded_bytes_buf, encoded_bytes_len);
        }
    }
}

/**
 * @brief COBS (consistent overhead byte stuffing) encoding function
 *
 * @param src pointer to source unencoded byte sequence
 * @param dst pointer to destination memory for the COBS stuffed byte sequence
 * @param len length of unencoded byte sequence
 * @return length of COBS encoded byte sequence
 */
uint16_t cobs_encode(char *src, char *dst, uint16_t len)
{
    size_t read_index, write_index, code_index;
    /* read index */
    read_index = 0;
    /* write index
    write_index starts at 1 because the first byte is the number of
     bytes to the first 0x0 */
    write_index = 1;
    /* code index in the dst buffer
    it starts at zero and is set to
    indices where 0x0 is read. */
    code_index = 0;
    /* code holds the length from the current
    write pointer to where the next 0x0 is in the
    source buffer.
    code starts at 1 because the very first byte in
    a COBS encoded string is the length to the next
    0x0.
    */
    uint8_t code = 1;

    while (read_index < len)
    {
        /* if current read byte is 0x0 */
        if (src[read_index] == 0)
        {
            /* set the code index to the number of bytes
            from the previous 0x0 to the current 0x0 */
            dst[code_index] = code;
            /* move code_index to the next write_index
            code_index gets assigned the value of write_index
            before the increment*/
            code_index = write_index++;
            /* reset number of bytes to the next 0x0 to 1 */
            code = 1;
        }
        /* if current read byte is not 0x0 */
        else
        {
            /* copy the byte to dst */
            dst[write_index++] = src[read_index];
            /* increment number of bytes to the next 0x0 */
            code++;
            /* if code exceeds 0xFF */
            if (code == 0xFF)
            {
                /* set the current code index in dst to 0xFF */
                dst[code_index] = code;
                /* move the code index after the write index */
                code_index = write_index++;
                /* reset number of bytes to the next 0x0 to 1. */
                code = 1;
            }
        }
        read_index++;
    }
    dst[code_index] = code;
    dst[write_index++] = 0x00;
    return write_index;
}

/**
 * @brief COBS (consistent overhead byte stuffing) decoding function
 *
 * @param src pointer to source encoded byte sequence
 * @param dst pointer to destination memory for the COBS decoded byte sequence
 * @param len length of encoded byte sequence
 *
 * @return length of COBS decoded byte sequence
 */
uint16_t cobs_decode(char *src, char *dst, uint16_t len)
{
    size_t read_index, write_index;
    /* read index */
    read_index = 0;
    /* write index */
    write_index = 0;
    /* code value */
    uint8_t code = 0;

    // read COBS encoded string
    while (read_index < len)
    {
        /* get code
        if code exceeded 0xFF, just read the next code byte. */
        code = src[read_index];

        /* out of bounds error */
        if (read_index + code > len && code != 1)
        {
            return 0;
        }

        read_index++;
        /* copy all the bytes until the next 0x0 */
        for (int i = 1; i < code; i++)
        {
            dst[write_index++] = src[read_index++];
        }

        /* write the 0x0 once the next 0x0 index has been reached */
        if (code < 0xFF && read_index < len)
        {
            dst[write_index++] = 0x00;
        }
    }
    write_index--;
    return write_index;
}
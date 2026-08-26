#include "../include/led_module.h"

// LED
// ****************************************************************

// queue for status LED states
extern QueueHandle_t status_led_queue;

void gpio_init(void)
{
    // zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
}
// gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
// gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);

void set_datalogging_led(uint8_t led_value)
{
    gpio_set_level(LED1_PIN, led_value);
}

/**
 * @brief notification LED task
 *
 * The notification LED has the following states: STATUS_LED_OFF, STATUS_LED_ON, STATUS_LED_BLINK, and STATUS_LED_FAST_BLINK.
 * The notification LED state is set via a task notification.
 *
 * @param pvParameters
 *
 */
void task_notification_LED(void *pvParameters)
{
    uint32_t cmd;
    cmd_task_status_led_t status_led_state = STATUS_LED_OFF;
    uint8_t led_value = 0;
    unsigned int loop_delay = 10;
    while (1)
    {
        // xQueueReceive(status_led_queue, &status_led_state, loop_delay);
        // status_led_state = ulTaskNotifyTakeIndexed(0, pdFALSE, loop_delay);
        // use task notification as queue
        if (xTaskNotifyWait(0,
                            0,
                            &cmd,
                            loop_delay / portTICK_PERIOD_MS) == pdTRUE)
        {
            status_led_state = cmd;
        }

        switch (status_led_state)
        {
        // notification LED blinks when datalogging is ongoing
        case STATUS_LED_BLINK:
            led_value = !led_value;
            loop_delay = 500;
            break;
        // notification LED blinks fast when calibration is ongoing
        case STATUS_LED_FAST_BLINK:
            led_value = !led_value;
            loop_delay = 100;
            break;
        //
        case STATUS_LED_ON:
            led_value = 1;
            loop_delay = 20;
            break;
        // notification LED is off when datalogging is stopped
        default:
            led_value = 0;
            loop_delay = 20;
            break;
        }
        set_datalogging_led(led_value);
        // ESP_LOGI("LED_module", "%d\n", status_led_state);
    }
}
// ****************************************************************

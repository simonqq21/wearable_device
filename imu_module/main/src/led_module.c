#include "../include/led_module.h"

// LED
// ****************************************************************

// queue for status LED states
extern QueueHandle_t status_led_queue;

void gpio_init(void)
{
    status_led_queue = xQueueCreate(5, sizeof(status_led_state_t));
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

// notification LED task
/*
notification LED blinks when datalogging is ongoing,
off when datalogging is stopped
*/

void notif_led_task(void *pvParameters)
{
    status_led_state_t status_led_state;
    uint8_t led_value = 0;
    TickType_t loop_delay = 20 / portTICK_PERIOD_MS;
    while (1)
    {
        xQueueReceive(status_led_queue, &status_led_state, loop_delay);

        switch (status_led_state)
        {
        case STATUS_LED_BLINK:
            led_value = !led_value;
            loop_delay = 500 / portTICK_PERIOD_MS;
            break;
        case STATUS_LED_FADE:
            break;
        default:
            led_value = 0;
            loop_delay = 20 / portTICK_PERIOD_MS;
            break;
        }

        gpio_set_level(LED1_PIN, led_value);
    }
}

// ****************************************************************

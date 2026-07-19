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

// ****************************************************************

#include "TaskGPIO.h"

void task_GPIO(void *params)
{
    cmd_task_status_led_t status_led_state = CMD_STATUS_LED_OFF;
    uint8_t led_value = 0;
    unsigned int loop_delay = 20;

    params_task_gpio_t *params_task_gpio = (params_task_gpio_t *)params;

    gpio_cmd_struct_t *gpio_cmds = (gpio_cmd_struct_t *)params_task_gpio->gpio_cmds;
    /* set button callbacks */
    buttons_set_callbacks();

    while (1)
    {
        status_led_state = (cmd_task_status_led_t)atomic_load(&gpio_cmds->status_led_cmd);
        switch (status_led_state)
        {
        // notification LED blinks when datalogging is ongoing
        case CMD_STATUS_LED_BLINK:
            led_value = !led_value;
            loop_delay = 500;
            break;
        // notification LED blinks fast when calibration is ongoing
        case CMD_STATUS_LED_FAST_BLINK:
            led_value = !led_value;
            loop_delay = 100;
            break;
        //
        case CMD_STATUS_LED_ON:
            led_value = 1;
            loop_delay = 200;
            break;
        // notification LED is off when datalogging is stopped
        default:
            led_value = 0;
            loop_delay = 200;
            break;
        }
        set_datalogging_led(led_value);
        // ESP_LOGI("LED_module", "%d\n", status_led_state);

        vTaskDelay(loop_delay / portTICK_PERIOD_MS);
    }
}
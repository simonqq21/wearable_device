#include "TaskGPIO.h"

void task_GPIO(void *params)
{
    uint32_t cmd;
    cmd_task_status_led_t status_led_state = STATUS_LED_OFF;
    uint8_t led_value = 0;
    unsigned int loop_delay = 10;

    params_task_gpio_t *params_task_gpio = (params_task_gpio_t *)params;
    // /* get task handles from task params */
    // TaskHandle_t task_handle_GPIO = params_task_gpio->task_handle_GPIO;
    // TaskHandle_t task_handle_sensors = params_task_gpio->task_handle_sensors;
    // TaskHandle_t task_handle_main_datalogging = params_task_gpio->task_handle_main_datalogging;
    // TaskHandle_t task_handle_SD_card_datalogger = params_task_gpio->task_handle_SD_card_datalogger;

    // /* set gpio task handles for the button callbacks */
    // set_gpio_task_handles(task_handle_GPIO,
    //                       task_handle_sensors,
    //                       task_handle_main_datalogging,
    //                       task_handle_SD_card_datalogger);

    /* set button callbacks */
    buttons_set_callbacks();

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
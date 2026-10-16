#ifndef TASK_GPIO_H
#define TASK_GPIO_H

#include "../include/gpio_module.h"

/**
 * @brief GPIO task params
 *
 * @param
 * @param streambuffer_imu
 *
 */
typedef struct
{
    // TaskHandle_t task_handle_GPIO;
    // TaskHandle_t task_handle_sensors;
    // TaskHandle_t task_handle_main_datalogging;
    // TaskHandle_t task_handle_SD_card_datalogger;
} params_task_gpio_t;

void task_GPIO(void *params);

#endif
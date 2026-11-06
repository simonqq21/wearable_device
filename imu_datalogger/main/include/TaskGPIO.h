#ifndef TASK_GPIO_H
#define TASK_GPIO_H

#include "gpio_module.h"

/**
 * @brief GPIO task params
 *
 * @param gpio_cmds pointer to GPIO commands struct
 */
typedef struct params_task_gpio_t
{
    gpio_cmd_struct_t *gpio_cmds;
    sensors_cmd_struct_t *sensors_cmds;
    sensor_fusion_cmd_struct_t *sensor_fusion_cmds;
    sd_card_cmd_struct_t *sd_card_cmds;
} params_task_gpio_t;

void task_GPIO(void *params);

#endif
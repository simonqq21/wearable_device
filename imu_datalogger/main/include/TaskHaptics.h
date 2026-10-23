#ifndef TASK_HAPTICS_H
#define TASK_HAPTICS_H

#include "haptics_lib.h"

/**
 * @brief task parameters for haptics
 *
 * @param queue for haptics command play builtin sequence
 * @param queue for haptics command play custom sequence
 */
typedef struct
{
    haptic_actuator_array_t *haptics_hw;
    QueueHandle_t queue_haptics_BLE;
} params_task_haptics_t;

void task_haptics(void *params);

#endif
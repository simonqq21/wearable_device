#include "haptics_lib.h"
#include "TaskHaptics.h"

/**
 * sample haptic pulse sequences
 */

const int master_delay = 500;

haptic_pulse_t pulse_null[1] = {
    {master_delay, {0, 0, 0, 0}}};

/* simple pulsing on-off */
haptic_pulse_t pulse_seq_1[6] = {
    {master_delay, {1, 1, 1, 1}},
    {master_delay, {0, 0, 0, 0}},
    {master_delay, {1, 1, 1, 1}},
    {master_delay, {0, 0, 0, 0}},
    {master_delay, {1, 1, 1, 1}},
    {master_delay, {0, 0, 0, 0}}};

haptic_pulse_t pulse_seq_2[4] = {
    {master_delay, {1, 0, 0, 0}},
    {master_delay, {0, 1, 0, 0}},
    {master_delay, {0, 0, 1, 0}},
    {master_delay, {0, 0, 0, 1}}};

haptic_pulse_t pulse_seq_3[4] = {
    {master_delay, {0, 0, 0, 1}},
    {master_delay, {0, 0, 1, 0}},
    {master_delay, {0, 1, 0, 0}},
    {master_delay, {1, 0, 0, 0}}};

haptic_pulse_t pulse_seq_4[2] = {
    {master_delay * 2, {1, 0, 0, 1}},
    {master_delay * 2, {0, 1, 1, 0}},
};

haptic_pulse_t pulse_seq_5[2] = {
    {master_delay * 2, {0, 1, 1, 0}},
    {master_delay * 2, {1, 0, 0, 1}},
};

/**
 * Haptics FreeRTOS task
 *
 */
void task_haptics(void *params)
{

    uint8_t testing;

    /* get the haptics params */
    params_task_haptics_t *params_task_haptics = (params_task_haptics_t *)params;

    QueueHandle_t queue_haptics_sequence_BLE = params_task_haptics->queue_haptics_sequence_BLE;
    haptic_actuator_array_t *haptics_hw = params_task_haptics->haptics_hw;

    while (1)
    {
        if (queue_haptics_sequence_BLE != NULL)
        {
            if (xQueueReceive(queue_haptics_sequence_BLE, &testing, 100 / portTICK_PERIOD_MS) == pdTRUE)
            {
                ESP_LOGI(HAPTICS_TAG, "haptics playy");
                haptics_load_pulse_seq(haptics_hw, pulse_seq_1, 6);
            }
        }
    }
}
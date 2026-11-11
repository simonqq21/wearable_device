#include "haptics_lib.h"
#include "TaskHaptics.h"

/**
 * sample haptic pulse sequences
 */

const int master_delay = 500;

haptic_pulse_t haptic_all_off[1] = {
    {.duration_ms = master_delay,
     .values = {0, 0, 0, 0}}};

haptic_pulse_t haptic_1_on[1] = {
    {.duration_ms = master_delay,
     .values = {1, 0, 0, 0}}};

haptic_pulse_t haptic_2_on[1] = {
    {.duration_ms = master_delay,
     .values = {0, 1, 0, 0}}};

haptic_pulse_t haptic_3_on[1] = {
    {.duration_ms = master_delay,
     .values = {0, 0, 1, 0}}};

haptic_pulse_t haptic_4_on[1] = {
    {.duration_ms = master_delay,
     .values = {0, 0, 0, 1}}};

haptic_pulse_t haptic_all_on[1] = {
    {.duration_ms = master_delay,
     .values = {1, 1, 1, 1}}};

/* simple pulsing on-off */
haptic_pulse_t pulse_seq_1[6] = {
    {.duration_ms = master_delay, .values = {1, 1, 1, 1}},
    {master_delay, 0, {0, 0, 0, 0}},
    {master_delay, 0, {1, 1, 1, 1}},
    {master_delay, 0, {0, 0, 0, 0}},
    {master_delay, 0, {1, 1, 1, 1}},
    {master_delay, 0, {0, 0, 0, 0}}};

haptic_pulse_t pulse_seq_2[4] = {
    {master_delay, 0, {1, 0, 0, 0}},
    {master_delay, 0, {0, 1, 0, 0}},
    {master_delay, 0, {0, 0, 1, 0}},
    {master_delay, 0, {0, 0, 0, 1}}};

haptic_pulse_t pulse_seq_3[4] = {
    {master_delay, 0, {0, 0, 0, 1}},
    {master_delay, 0, {0, 0, 1, 0}},
    {master_delay, 0, {0, 1, 0, 0}},
    {master_delay, 0, {1, 0, 0, 0}}};

haptic_pulse_t pulse_seq_4[2] = {
    {master_delay * 2, 0, {1, 0, 0, 1}},
    {master_delay * 2, 0, {0, 1, 1, 0}},
};

haptic_pulse_t pulse_seq_5[2] = {
    {master_delay * 2, 0, {0, 1, 1, 0}},
    {master_delay * 2, 0, {1, 0, 0, 1}},
};

/**
 * Haptics FreeRTOS taskhaptics playy
 *
 */
void task_haptics(void *params)
{

    uint8_t testing;

    /* get the haptics params */
    params_task_haptics_t *params_task_haptics = (params_task_haptics_t *)params;

    QueueHandle_t queue_haptics_BLE = params_task_haptics->queue_haptics_BLE;
    haptic_actuator_array_t *haptics_hw = params_task_haptics->haptics_hw;

    while (1)
    {
        if (queue_haptics_BLE != NULL)
        {
            if (xQueueReceive(queue_haptics_BLE, &testing, 100 / portTICK_PERIOD_MS) == pdTRUE)
            {
                ESP_LOGI(HAPTICS_TAG, "recv %d", testing);
                switch (testing)
                {
                case 1:
                    haptics_load_pulse_seq(haptics_hw, haptic_1_on, 1);
                    break;
                case 2:
                    haptics_load_pulse_seq(haptics_hw, haptic_2_on, 1);
                    break;
                case 3:
                    haptics_load_pulse_seq(haptics_hw, haptic_3_on, 1);
                    break;
                case 4:
                    haptics_load_pulse_seq(haptics_hw, haptic_4_on, 1);
                    break;
                case 5:
                    haptics_load_pulse_seq(haptics_hw, haptic_all_on, 1);
                    break;
                case 6:
                    haptics_load_pulse_seq(haptics_hw, haptic_all_off, 1);
                    break;
                default:
                    haptics_stop(haptics_hw);
                }
            }
        }
    }
}
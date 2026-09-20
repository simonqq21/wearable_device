#include "haptic_lib.h"

static void timer_callback(void *arg);

#define CHANNEL_0_PIN_1 (2)
#define CHANNEL_0_PIN_2 (0)
#define CHANNEL_1_PIN_1 (5)
#define CHANNEL_1_PIN_2 (0)
#define CHANNEL_2_PIN_1 (32)
#define CHANNEL_2_PIN_2 (0)
#define CHANNEL_3_PIN_1 (33)
#define CHANNEL_3_PIN_2 (0)
#define EN_PIN (26)

haptic_actuator_array_t haptics_hw;
haptic_channel_pins_t haptics_channels[4] = {
    {CHANNEL_0_PIN_1, CHANNEL_0_PIN_2},
    {CHANNEL_1_PIN_1, CHANNEL_1_PIN_2},
    {CHANNEL_2_PIN_1, CHANNEL_2_PIN_2},
    {CHANNEL_3_PIN_1, CHANNEL_3_PIN_2}};

/**
 * sample haptic pulse sequences
 */

const int master_delay = 400;

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
    {master_delay * 2, {1, 0, 0, 0}},
    {master_delay * 2, {0, 1, 0, 0}},
    {master_delay * 2, {0, 0, 1, 0}},
    {master_delay * 2, {0, 0, 0, 1}}};

haptic_pulse_t pulse_seq_3[4] = {
    {master_delay * 2, {0, 0, 0, 1}},
    {master_delay * 2, {0, 0, 1, 0}},
    {master_delay * 2, {0, 1, 0, 0}},
    {master_delay * 2, {1, 0, 0, 0}}};

haptic_pulse_t pulse_seq_4[2] = {
    {master_delay * 2, {1, 0, 0, 1}},
    {master_delay * 2, {0, 1, 1, 0}},
};

haptic_pulse_t pulse_seq_5[2] = {
    {master_delay * 2, {0, 1, 1, 0}},
    {master_delay * 2, {1, 0, 0, 1}},
};

// haptic_pulse_t pulse_seq_4[4] = {
//     {master_delay, {1, 1, 0, 0}},
//     {master_delay, {0, 0, 1, 1}},
//     {master_delay, {1, 0, 1, 0}},
//     {master_delay, {0, 1, 0, 1}}};

volatile int i = 0;

// Callback function executed when the timer fires
static void timer_callback(void *arg)
{
    // i = (i + 1) % 4;
    // haptics_play_pulse(&haptics_hw, &pulse_seq_1[i]);
    haptics_ISR_callback(&haptics_hw);
}

void app_main(void)
{
    haptics_init(&haptics_hw,
                 EN_PIN,
                 haptics_channels);
    haptics_disable_all(&haptics_hw);

    // Define timer configuration
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "asdf timer",
    };

    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));

    // Start periodic timer with a 1 ms period (1000 microseconds)
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 1000));

    // ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 500000));

    haptics_load_pulse_seq(&haptics_hw, pulse_null, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1)
    {
        // for (uint8_t i = 0; i < 4; i++)
        // {
        //     haptic_set_val(haptics_channels[i], 1);
        // }
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // for (uint8_t i = 0; i < 4; i++)
        // {
        //     haptic_set_val(haptics_channels[i], 0);
        // }

        // haptics_load_pulse_seq(&haptics_hw, pulse_seq_1, 1);

        haptics_load_pulse_seq(&haptics_hw, pulse_seq_1, 6);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        haptics_load_pulse_seq(&haptics_hw, pulse_seq_2, 4);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        haptics_load_pulse_seq(&haptics_hw, pulse_seq_3, 4);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        haptics_load_pulse_seq(&haptics_hw, pulse_seq_4, 2);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        haptics_load_pulse_seq(&haptics_hw, pulse_seq_5, 2);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        vTaskDelay(400 / portTICK_PERIOD_MS);

        // haptics_play_pulse(&haptics_hw, &pulse_seq_1[0]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_1[1]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_1[2]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_1[3]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);

        // vTaskDelay(1800 / portTICK_PERIOD_MS);

        // haptics_play_pulse(&haptics_hw, &pulse_seq_2[0]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_2[1]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_2[2]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_2[3]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);

        // vTaskDelay(1800 / portTICK_PERIOD_MS);

        // haptics_play_pulse(&haptics_hw, &pulse_seq_3[0]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_3[1]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_3[2]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // haptics_play_pulse(&haptics_hw, &pulse_seq_3[3]);
        // vTaskDelay(500 / portTICK_PERIOD_MS);

        // vTaskDelay(1800 / portTICK_PERIOD_MS);
    }
}

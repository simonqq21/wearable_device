#include "haptic_lib.h"

// esp_timer_get_time() / 1000;
// uint16_t haptic_sequence_len = 0;
// volatile uint16_t haptic_last_idx = 0;
// volatile uint16_t haptic_sequence_idx = 0;
// volatile uint8_t haptic_paused = false;
// volatile uint32_t haptic_time = 0;
// volatile uint32_t haptic_last_time = 0;

/**
 * @brief initialize the haptic actuator array hardware
 *
 * @param haptics pointer to haptic actuator array
 * @param channels pointer to array of haptic channels
 */
void haptics_init(haptic_actuator_array_t *haptics,
                  uint8_t en_pin,
                  haptic_channel_pins_t *channels)
{
    uint64_t haptic_actuators_pin_bitmask = 0;

    haptics->state = HAPTICS_STATE_IDLE;
    haptics->cmd = HAPTICS_CMD_STOP;
    haptics->head_idx = 0;
    haptics->tail_idx = 0;
    haptics->played_idx = 0;
    haptics->time = 0;
    haptics->last_time = 0;

    /* generate the GPIO pin bitmask */
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        haptics->channels[i] = channels[i];
        haptic_actuators_pin_bitmask |= (1ULL << channels[i].pin_1) | (1ULL << channels[i].pin_2);
    }
    haptics->en_pin = en_pin;
    haptic_actuators_pin_bitmask |= (1ULL << en_pin);

    // zero-initialize the config structure.
    gpio_config_t io_conf = {};

    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;

    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = haptic_actuators_pin_bitmask;

    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    // configure GPIO with the given settings
    gpio_config(&io_conf);
}

/**
 * @brief set value for a haptic actuator
 *
 * @param channel channel to set
 * @param val value to set
 */
void haptic_set_val(haptic_channel_pins_t channel, uint8_t val)
{

    if (val)
    {
        gpio_set_level(channel.pin_1, val);
        gpio_set_level(channel.pin_2, 0);
    }
    else
    {
        gpio_set_level(channel.pin_1, 1);
        gpio_set_level(channel.pin_2, 1);
    }
}

/**
 * @brief disable all haptics
 *
 * This function is used when stopping haptics.
 *
 * @param
 */
void haptics_disable_all(haptic_actuator_array_t *haptics)
{
    gpio_set_level(haptics->en_pin, 0);
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        haptic_set_val(haptics->channels[i], 0);
    }
}

/**
 * @brief play a single multi-channel haptic actuator pulse
 *
 * @param haptics pointer to haptic actuator array
 * @param pulse haptic pulse to play
 */
void haptics_play_pulse(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse)
{
    gpio_set_level(haptics->en_pin, 1);

    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        haptic_set_val(haptics->channels[i], pulse->values[i]);
    }
}

/**
 * @brief load a sequence of pulses into the circular buffer for playback
 *
 * @param haptics pointer to haptic actuator array
 * @param pulse_seq pointer to the start of the haptic pulse sequence
 */
void haptics_load_pulse_seq(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse_seq, uint8_t len)
{
    if (len > HAPTIC_SEQ_BUF_LEN)
    {
        return;
    }
    /* memcpy */
    for (int i = 0; i < len; i++)
    {
        haptics->pulses_buffer[haptics->head_idx] = pulse_seq[i];
        haptics->head_idx = (haptics->head_idx + 1) % HAPTIC_SEQ_BUF_LEN;
    }
}

// /**
//  * @brief play a haptic actuator pulse sequence without blocking the rest of the code
//  *
//  * This function sets the pointer to the start of the haptic pulse sequence and sets
//  * the command to play.
//  *
//  * @param haptics pointer to haptic actuator array
//  * @return
//  * @note
//  */
// void haptics_play_pulse(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse)
// {
//     /* set command to play */
//     haptics->cmd = HAPTICS_CMD_PLAY;

//     for (uint8_t i = 0; i < pulse->num_channels; i++)
//     {
//         haptic_set_val(haptics.channels[i], pulse->values[i]);
//     }
//     vTaskDelay(pdMS_TO_TICKS(pulse->duration_ms));
//     for (uint8_t i = 0; i < pulse->num_channels; i++)
//     {
//         haptic_set_val(haptics.channels[i], 0);
//     }
//     vTaskDelay(pdMS_TO_TICKS(pulse->pause_ms));
// }

/**
 * @brief stop playing the haptic sequence it is currently playing
 */
void haptics_stop(haptic_actuator_array_t *haptics)
{
    /* set command to stop */
    haptics->cmd = HAPTICS_CMD_STOP;
}

/**
 * @brief check if haptic actuators are playing a sequence
 */
uint8_t haptics_busy(haptic_actuator_array_t *haptics)
{
    if (haptics->state == HAPTICS_STATE_PLAYING)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief haptic actuator loop
 */
void haptics_ISR_callback(haptic_actuator_array_t *haptics)
{
    haptics->time = esp_timer_get_time() / 1000;

    switch (haptics->state)
    {
    case HAPTICS_STATE_IDLE:
        haptics_disable_all(haptics);
        /* if the pulse sequence circular buffer is not empty, start playing
        and consuming the pulses in the circular buffer. */
        if (haptics->head_idx != haptics->tail_idx)
        {
            haptics->state = HAPTICS_STATE_PLAYING;
            haptics->cmd = HAPTICS_CMD_PLAY;
            haptics->last_time = haptics->time;
        }
        break;
    case HAPTICS_STATE_PLAYING:
        /* stop playing */
        if (haptics->cmd == HAPTICS_CMD_STOP)
        {
            haptics->state = HAPTICS_STATE_STOPPED;
        }

        /* if there are pulses to play, */
        if (haptics->tail_idx != haptics->head_idx)
        {
            /* play the sequence */
            if (haptics->played_idx != haptics->tail_idx)
            {
                haptics->played_idx = haptics->tail_idx;
                haptics_play_pulse(haptics, &haptics->pulses_buffer[haptics->tail_idx]);
            }

            /* increment the pulse sequence */
            if (haptics->time - haptics->last_time >= haptics->pulses_buffer[haptics->tail_idx].duration_ms)
            {
                haptics->last_time = haptics->time;
                haptics->tail_idx = (haptics->tail_idx + 1) % HAPTIC_SEQ_BUF_LEN;
            }
        }
        /* else, go into the stopped state. */
        else
        {
            haptics->state = HAPTICS_STATE_STOPPED;
        }

        break;
    case HAPTICS_STATE_STOPPED:
        /* Disable all haptics before going to idle state */
        haptics_disable_all(haptics);
        /* go to idle state */
        haptics->state = HAPTICS_STATE_IDLE;
        break;

    default:
        break;
    }
}

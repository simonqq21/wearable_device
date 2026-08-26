#ifndef HAPTIC_LIB_H
#define HAPTIC_LIB_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define NUM_CHANNELS (4)
#define HAPTIC_SEQ_BUF_LEN (20)

/**
 * @enum STATES of the haptic actuator array
 */
typedef enum
{
    HAPTICS_STATE_IDLE,
    HAPTICS_STATE_PLAYING,
    HAPTICS_STATE_STOPPED,
} haptics_states_enum;

/**
 * @enum COMMANDS sent to the haptic actuator array
 */
typedef enum
{
    HAPTICS_CMD_STOP,
    HAPTICS_CMD_PLAY,
} haptics_cmd_enum;

/**
 * @struct DATA a single multi-channel haptic pulse
 * Sequences of haptic pulses played in sequence represents a haptic
 * message.
 */
typedef struct
{
    uint32_t duration_ms;
    uint8_t values[NUM_CHANNELS];
} haptic_pulse_t;

/**
 * @struct DEVICE pins for one haptic actuator channel
 */
typedef struct
{
    int pin_1;
    int pin_2;
} haptic_channel_pins_t;

/**
 * @struct DEVICE entire haptic actuator array
 *
 */
typedef struct
{
    haptic_channel_pins_t channels[NUM_CHANNELS]; /* each channel in the actuator array */
    haptics_states_enum state;                    /* current state of the haptic array */
    haptics_cmd_enum cmd;                         /* command sent to the haptic array */

    haptic_pulse_t pulses_buffer[HAPTIC_SEQ_BUF_LEN]; /* ring buffer for pulses */
    uint32_t head_idx, tail_idx, played_idx;          /* haptic pulse ring buffer head and tail pointer */

    uint32_t time;      /* current elapsed time */
    uint32_t last_time; /* last time */
    uint8_t en_pin;

} haptic_actuator_array_t;

void haptics_init(haptic_actuator_array_t *haptics,
                  uint8_t en_pin,
                  haptic_channel_pins_t *channels);
void haptic_set_val(haptic_channel_pins_t channel, uint8_t val);
void haptics_disable_all(haptic_actuator_array_t *haptics);
void haptics_play_pulse(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse);
void haptics_load_pulse_seq(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse_seq, uint8_t len);
uint8_t haptics_busy(haptic_actuator_array_t *haptics);
void haptics_ISR_callback(haptic_actuator_array_t *haptics);
#endif
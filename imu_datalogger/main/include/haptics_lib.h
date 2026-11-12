#ifndef HAPTIC_LIB_H
#define HAPTIC_LIB_H

#include "common.h"

#define HAPTICS_TAG "HAPTICS_TAG"

#define HAPTIC_ACTIVE_LEVEL (1)
#define NUM_CHANNELS (4)
#define HAPTIC_SEQ_BUF_LEN (20)

/**
 * @enum STATES of the haptic actuator array
 */
typedef enum
{
    HAPTICS_STATE_IDLE,
    HAPTICS_STATE_PLAYING,
    HAPTICS_STATE_STOPPED
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
 *
 * Sequences of haptic pulses played in sequence represents a haptic
 * message.
 *
 * @param duration_ms duration of this haptic pulse
 * @param fade_to_ms duration to fade the haptic actuators from the previous values
 * @param values array of values for the haptic actuators
 *
 */
typedef struct
{
    uint16_t duration_ms;
    uint16_t fade_to_ms;
    uint8_t values[NUM_CHANNELS];
} haptic_pulse_t;

/**
 * @struct struct for a whole sequence of haptic pulses
 *
 * @param len length of haptic sequence
 * @param haptic_sequence pointer to array of haptic sequences
 */
typedef struct
{
    uint8_t len;
    haptic_pulse_t *haptic_sequence;
} haptic_sequence_t;

/**
 * @struct DEVICE pins for one haptic actuator channel
 */
typedef struct
{
    int pin_1;
    // int pin_2;
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

/**
 * @brief struct for playing built-in haptic sequences
 *
 * @param command command to play or stop
 * @param preset_id  preset ID of builtin haptic sequence to play
 * @param repeat_count number of times to repeat the sequence
 */
typedef struct
{
    uint8_t command;
    uint16_t preset_id;
    uint8_t repeat_count;
} haptics_command_play_builtin_sequence_t;

// /**
//  * @brief haptic command struct for playing custom haptic sequences sent
//  * over BLE.
//  *
//  * @param total_steps total number of haptic sequence steps to play
//  * @param repeat_count
//  */
// typedef struct
// {
//     uint8_t total_steps;
//     uint8_t repeat_count;
//     haptic_pulse_t haptic_pulse_sequence[];
// } haptics_command_play_custom_sequence_t;

// /**
//  * @brief haptics status struct sent over BLE
//  *
//  * @param status haptic hardware status
//  */
// typedef struct
// {
//     haptics_states_enum status;
// } haptics_status_t;

// /**
//  * @brief haptics channels config sent over BLE
//  *
//  * @param mask channel mask, 1 is enabled and 0 is disabled
//  */
// typedef struct
// {
//     uint8_t channel_mask;
// } haptics_command_config_t;

/**
 * @brief initialize the haptic actuator array hardware
 *
 * @param haptics pointer to haptic actuator array
 * @param channels pointer to array of haptic channels
 */
void haptics_init(haptic_actuator_array_t *haptics,
                  uint8_t en_pin,
                  haptic_channel_pins_t *channels);

/**
 * @brief set value for a haptic actuator
 *
 * @param channel channel to set
 * @param val value to set
 */
void haptic_set_val(haptic_channel_pins_t channel, uint8_t val);

/**
 * @brief disable all haptics
 *
 * This function is used when stopping haptics.
 *
 * @param
 */
void haptics_disable_all(haptic_actuator_array_t *haptics);

/**
 * @brief play a single multi-channel haptic actuator pulse
 *
 * @param haptics pointer to haptic actuator array
 * @param pulse haptic pulse to play
 */
void haptics_play_pulse(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse);

/**
 * @brief load a sequence of pulses into the circular buffer for playback
 *
 * @param haptics pointer to haptic actuator array
 * @param pulse_seq pointer to the start of the haptic pulse sequence
 * @param len the length of the haptic sequence
 */
void haptics_load_pulse_seq(haptic_actuator_array_t *haptics, haptic_pulse_t *pulse_seq, uint8_t len);

/**
 * @brief stop playing the haptic sequence it is currently playing
 *
 * @param haptics pointer to haptic actuator array
 */
void haptics_stop(haptic_actuator_array_t *haptics);

/**
 * @brief check if haptic actuators are playing a sequence
 *
 * @param haptics pointer to haptic actuator array
 */
uint8_t haptics_busy(haptic_actuator_array_t *haptics);

/**
 * @brief play a haptic sequence file from the built-in flash
 *
 * @param haptics pointer to haptic actuator array
 * @param filename filename of the haptic sequence binary file in littlefs
 */
void haptics_play_file(haptic_actuator_array_t *haptics, char *filename);

/**
 * @brief haptic actuator loop
 *
 * @param haptics pointer to haptic actuator array
 */
void haptics_ISR_callback(haptic_actuator_array_t *haptics);

#endif
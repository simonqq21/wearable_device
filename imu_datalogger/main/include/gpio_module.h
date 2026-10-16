#ifndef GPIO_MODULE_H
#define GPIO_MODULE_H

#include "common.h"

// ESP32 pins config
#define BUTTON_TAG "BUTTON"
#define GPIO_OUTPUT_PIN_SEL ((1ULL << LED1_PIN) | (1ULL << LED2_PIN))

/**
 * @brief notification LED task
 *
 * The notification LED has the following states: STATUS_LED_OFF, STATUS_LED_ON, STATUS_LED_BLINK, and STATUS_LED_FAST_BLINK.
 * The notification LED state is set via a task notification.
 *
 * @param params
 *
 */
void task_GPIO(void *pvParameters);
/**
 * @brief initialize GPIO
 */
void gpio_init(); // button_handle_t *btn1
/**
 * @brief initialize buttons
 */
esp_err_t buttons_init(void);
/**
 * @brief set button callbacks.
 */
void buttons_set_callbacks(void);
/**
 * @brief set the datalogging LED value
 */
void set_datalogging_led(uint8_t led_value);
/*
notify all relevant tasks to pause datalogging and initiate calibration
*/
void button_toggle_datalogging_cb(void *arg, void *data);
/**
 * @brief notify IMU task to toggle datalogging
 */
void button_trigger_calibration_cb(void *arg, void *data);
// /**
//  * @brief set local taskhandles from the GPIO task.
//  *
//  * @param ptask_handle_GPIO
//  * @param ptask_handle_sensors
//  * @param ptask_handle_main_datalogging
//  * @param ptask_handle_SD_card_datalogger
//  */
// void set_gpio_task_handles(TaskHandle_t ptask_handle_GPIO,
//                            TaskHandle_t ptask_handle_sensors,
//                            TaskHandle_t ptask_handle_main_datalogging,
//                            TaskHandle_t ptask_handle_SD_card_datalogger);
#endif

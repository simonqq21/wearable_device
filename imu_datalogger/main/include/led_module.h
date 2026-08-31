#ifndef LED_MODULE_H
#define LED_MODULE_H

#include "common.h"

// ESP32 pins config

#define GPIO_OUTPUT_PIN_SEL ((1ULL << LED1_PIN) | (1ULL << LED2_PIN))

void gpio_init(void);
void set_datalogging_led(uint8_t led_value);
void task_notification_LED(void *pvParameters);

#endif
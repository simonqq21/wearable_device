#ifndef LED_MODULE_H
#define LED_MODULE_H

#include "common.h"

// ESP32 pins config
#define LED1_PIN 25 // power/status LED
#define LED2_PIN 4  // recording LED
#define GPIO_OUTPUT_PIN_SEL ((1ULL << LED1_PIN) | (1ULL << LED2_PIN))

void gpio_init(void);
void task_notification_LED(void *pvParameters);

#endif
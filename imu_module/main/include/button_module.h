#ifndef BUTTON_MODULE_H
#define BUTTON_MODULE_H

#include "common.h"

#define BUTTON_TAG "BUTTON"

void button_toggle_datalogging_cb(void *arg, void *data);
void button_trigger_calibration_cb(void *arg, void *data);
esp_err_t buttons_init(void);

#endif
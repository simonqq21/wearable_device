#include "heart_rate.h"

void heart_rate_init(i2c_master_dev_handle_t heart_rate_dev_handle)
{
    i2c_MAX30102_init(i2c_master_bus_handle_t bus_handle,
                      const i2c_MAX30102_config_t *MAX30102_config,
                      i2c_MAX30102_handle_t *MAX30102_handle);
}
#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

#include "common.h"
#include "driver/i2c_master.h"

#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

/**
 * @brief i2c master bus initialization
 *
 * @param bus_handle i2c bus handle pointer
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle);
/**
 * @brief i2c master create device on i2c bus
 *
 * @param bus_handle i2c bus handle pointer
 * @param dev_handle i2c device handle pointer
 * @param dev_i2c_addr i2c address of the i2c device
 */
void i2c_master_add_device(i2c_master_bus_handle_t *bus_handle,
                           i2c_master_dev_handle_t *dev_handle,
                           uint8_t dev_i2c_addr);
#endif
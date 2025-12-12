#ifndef INA219_DRIVER_H
#define INA219_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// INA219 I2C address
#define INA219_ADDR             0x40

// Register addresses
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNT_VOLT   0x01
#define INA219_REG_BUS_VOLT     0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// Gain settings
#define INA219_GAIN_0_125   0  // Gain 1/8, ±40mV range
#define INA219_GAIN_0_25    1  // Gain 1/4, ±80mV range  
#define INA219_GAIN_0_5     2  // Gain 1/2, ±160mV range
#define INA219_GAIN_1       3  // Gain 1, ±320mV range

typedef struct {
    i2c_port_t i2c_port;
    uint8_t address;
    float current_lsb;  // Current LSB in Amps
    float power_lsb;    // Power LSB in Watts
} ina219_dev_t;

// Initialize INA219
bool ina219_init(ina219_dev_t *dev, i2c_port_t port, uint8_t addr, 
                 int sda_pin, int scl_pin, float shunt_resistor_ohms, 
                 float max_expected_current_amps);

// Read bus voltage (V)
float ina219_read_bus_voltage(ina219_dev_t *dev);

// Read shunt voltage (mV)
float ina219_read_shunt_voltage(ina219_dev_t *dev);

// Read current (mA)
float ina219_read_current(ina219_dev_t *dev);

// Read power (mW)
float ina219_read_power(ina219_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif // INA219_DRIVER_H
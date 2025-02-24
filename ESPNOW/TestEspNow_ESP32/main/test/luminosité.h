#ifndef BH1750_H
#define BH1750_H

#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

#define BH1750_ADDR 0x23

#define BH1750_CMD 0x10  

void bh1750_init();

esp_err_t bh1750_write(uint8_t cmd);

esp_err_t bh1750_read(uint16_t *lux);

uint16_t get_luminosity();

#endif 

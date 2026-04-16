#pragma once

#include <Arduino.h>
#include <SparkFun_I2C_Expander_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_I2C_Expander_Arduino_Library

extern SFE_PCA95XX io_bur; // Global instance of the PCA95XX I2C expander

#define PCA9554_ADDRESS_BUR_20 0x20 // I2C address for PCA9534 expander

#define BUR_EN 6
#define CAM_SDA_PIN 39
#define CAM_SCL_PIN 43

extern void burning_init(void);
extern void burning(int side_num, int delay_ms);
#pragma once

#include <Arduino.h>
#include <SparkFun_I2C_Expander_Arduino_Library.h>

extern SFE_PCA95XX io; // Global instance of the PCA95XX I2C expander

#define PCA9554_ADDRESS_21 0x21 // I2C address for PCA9554 expander

#define CAM_ADP_EN  1

#define CAM_SDA_PIN 39
#define CAM_SCL_PIN 43

#define CAM_TAKE_PHOTO_TIMES 6

extern void camadapter_init(void);
extern void camadapter_pwr_on(void);
extern void camadapter_pwr_off(void);
extern void turnoncam(uint8_t cam);
extern void turnoffcam(uint8_t cam);
extern void turnoffallcams(void);

extern void flush_buffer(void);
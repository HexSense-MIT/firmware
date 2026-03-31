#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_heap_caps.h>

#include "cam_adapter.h"

#include "SD.h"
#include "SPI.h"
#include "FS.h"

#define CAM_RX_PIN 13
#define CAM_TX_PIN 12

#define SD_SCK   9
#define SD_MISO  40
#define SD_MOSI  41
#define SD_CS    42

extern SFE_PCA95XX io_bur; // Global instance of the PCA95XX I2C expander

#define PCA9554_ADDRESS_BUR_20 0x20 // I2C address for PCA9534 expander

#define BUR_EN 6

extern void system_init(void);



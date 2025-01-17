#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define SPI1_MISO 19
#define SPI1_MOSI 23
#define SPI1_SCK  18

#define SPI2_MISO 35
#define SPI2_MOSI 34
#define SPI2_SCK  39

void init(void);
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi3;

void MX_SPI1_Init(void);
void MX_SPI3_Init(void);

#ifdef __cplusplus
}
#endif


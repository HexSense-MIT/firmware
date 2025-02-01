#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "system.h"
#include "RFM95C.h"

void setup() {
  HAL_Init();
  SystemClock_Config();
  IO_init();
}

void loop() {
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(500);
}


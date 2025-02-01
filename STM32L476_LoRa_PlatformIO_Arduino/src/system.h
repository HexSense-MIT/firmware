#include <Arduino.h>

#include "stm32l4xx_hal.h"

#define LED_PIN_O PC4
#define LED_PIN_B PA3

extern void IO_init(void);
extern void SystemClock_Config(void);

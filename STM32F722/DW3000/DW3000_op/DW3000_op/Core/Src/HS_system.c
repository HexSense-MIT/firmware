/*
 * HS_system.c
 *
 *  Created on: Jan 30, 2025
 *      Author: l1534
 */

#include "HS_system.h"

/**
 * @brief blink a LED
 * 
 * @param GPIOx GPIO port
 * @param GPIO_Pin GPIO pin
 * @param interval LED on duration in ms. If 0, always on.
 */
void blink_led(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int interval) {
    // if interval > 0, blink
	if (interval) {
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(interval);
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	}
	// if interval == 0, always on
	else {
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	}
}

void Delay_us(uint32_t us)
{
  uint64_t delayus;
	unsigned int i;

  delayus = us * 10;
  for (i = 0; i < delayus; i++)
  {
    __NOP();
  }
}

// void test_run_info(unsigned char *data)
// {
//   uint16_t data_length;

//   data_length = strlen((const char *)data);
//   CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
//   CDC_Transmit_FS((uint8_t *)"\n\r", 2); /*Transmit end of line through USB - Virtual port*/
// }
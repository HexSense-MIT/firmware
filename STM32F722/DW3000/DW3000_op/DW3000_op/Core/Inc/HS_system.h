/*
 * HS_system.h
 *
 *  Created on: Jan 30, 2025
 *      Author: l1534
 */

#ifndef INC_HS_SYSTEM_H_
#define INC_HS_SYSTEM_H_

#include "main.h"

extern void blink_led(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, int interval);
extern void Delay_us(uint32_t us);

#endif /* INC_HS_SYSTEM_H_ */

// extern void test_run_info(unsigned char *data);


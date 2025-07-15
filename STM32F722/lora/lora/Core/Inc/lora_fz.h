/*
 * lora_fz.h
 *
 *  Created on: Jul 15, 2025
 *      Author: liufangzheng
 */

#ifndef INC_LORA_FZ_H_
#define INC_LORA_FZ_H_

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "stm32f7xx_hal.h"
#include "lora_reg.h"

typedef enum {
    ROLE_NONE = 0,
    ROLE_TX,
    ROLE_RX
} Role;

extern volatile bool RF95_TX_DONE_FLAG;
extern volatile bool RF95_RX_DONE_FLAG;
extern volatile bool RF95_CAD_DONE_FLAG;

uint8_t RF95_ReadReg(uint8_t addr);
void    RF95_WriteReg(uint8_t addr, uint8_t data);

void RF95_set_freq(unsigned int frequency);
void RF95_setBW(int BW);
void RF95_setTxPower(int power, int outputPin);
void RF95_setOCP(uint8_t mA);
bool RF95_isTransmitting(void);

extern int RF95_Init(Role role, unsigned int frequency, unsigned int BW, int power);
extern void RF95_sleep(void);
extern void RF95_idle(void);
extern void RF95_reset(void);
extern int RF95_beginPacket(int implicitHeader, uint8_t *data2send, int size);
extern int RF95_sendPacket(bool async);

extern void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* INC_LORA_FZ_H_ */

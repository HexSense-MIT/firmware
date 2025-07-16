#pragma once

#include "DW3000_FZ.h"

extern uint8_t data2send[10];

extern void DW3000_writetxdata_FZ(uint8_t *data, uint16_t len);
extern void DW3000_txcmd_FZ(uint32_t delay);
extern uint8_t DW3000_TXdone_FZ(void);


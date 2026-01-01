/*
 * DW3000_send_test_FZ.h
 *
 *  Created on: Dec 29, 2025
 *      Author: liufangzheng
 */

#ifndef INC_DW3000_SEND_TEST_FZ_H_
#define INC_DW3000_SEND_TEST_FZ_H_

#include "DW3000_FZ.h"

extern uint8_t data2send[10];

extern void DW3000_writetxdata_FZ(uint8_t *data, uint16_t len);
extern void DW3000_txcmd_FZ(uint32_t delay);
extern uint8_t DW3000_TXdone_FZ(void);



#endif /* INC_DW3000_SEND_TEST_FZ_H_ */

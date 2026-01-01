/*
 * DW3000_send_test_FZ.c
 *
 *  Created on: Dec 29, 2025
 *      Author: liufangzheng
 */

#include "DW3000_send_test_FZ.h"

uint8_t data2send[10] = {0x00, 0x11, 0x23, 0x45, 0xAA, 0x01, 0xFE, 0x0A, 0xEE, 0x0B};

/**
 * @brief Write data2send to TX buffer
 * 
 * @param data data2send
 * @param len length of data2send
 * @return uint8_t 
 */
void DW3000_writetxdata_FZ(uint8_t *data, uint16_t len) {
  DW3000writereg(TX_BUFFER_ID, data, len);
}

/**
 * @brief write a TX fast command
 * 
 * @param delay 
 */
void DW3000_txcmd_FZ(uint32_t delay) {
  DW3000_writefastCMD_FZ(CMD_TX);
}

/**
 * @brief check the status register to see if the TX is done
 * 
 * @return uint8_t 
 */
uint8_t DW3000_TXdone_FZ(void) {
  return DW3000readreg(SYS_STATUS_ID, 4) & SYS_STATUS_TXFRS_BIT_MASK;
}




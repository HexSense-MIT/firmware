/*
 * DW3000_recv_test_FZ.c
 *
 *  Created on: Jul 17, 2025
 *      Author: liufangzheng
 */

#include "DW3000_recv_test_FZ.h"

/**
 * @brief write a RX fast command
 * 
 */
void DW3000_start_receiver_FZ(void) {
  DW3000_writefastCMD_FZ(CMD_RX);
}

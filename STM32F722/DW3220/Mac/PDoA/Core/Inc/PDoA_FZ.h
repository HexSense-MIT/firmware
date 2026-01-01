/*
 * DSWR_FZ.h
 *
 *  Created on: Dec 29, 2025
 *      Author: liufangzheng
 */

#ifndef INC_PDOA_FZ_H_
#define INC_PDOA_FZ_H_

#include <stdint.h>
#include <stdbool.h>

#include "DW3000_FZ.h"
#include "DW3000_send_test_FZ.h"
#include "DW3000_recv_test_FZ.h"

#define M_PI		3.14159265358979323846

#define FCS_LEN 2

typedef enum {
  PDOA_ROLE_ANCHOR = 0,
  PDOA_ROLE_TAG    = 1,
} pdoa_role_t;

void pdoa_tx_frame(uint8_t *buf, uint16_t len);
void pdoa_rx_frame(int16_t *pdoa_val);

#endif /* INC_PDOA_FZ_H_ */

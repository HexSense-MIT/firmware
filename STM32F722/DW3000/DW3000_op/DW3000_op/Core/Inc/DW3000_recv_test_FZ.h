/*
 * DW3000_recv_test_FZ.h
 *
 *  Created on: Jul 17, 2025
 *      Author: liufangzheng
 */

#ifndef INC_DW3000_RECV_TEST_FZ_H_
#define INC_DW3000_RECV_TEST_FZ_H_

#include "DW3000_FZ.h"

#define FRAME_LEN_MAX      (127)
/* All RX errors mask. */
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE_BIT_MASK | SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK | SYS_STATUS_RXSTO_BIT_MASK \
                                | SYS_STATUS_ARFE_BIT_MASK | SYS_STATUS_CIAERR_BIT_MASK)

extern void DW3000_start_receiver_FZ(void);

#endif /* INC_DW3000_RECV_TEST_FZ_H_ */

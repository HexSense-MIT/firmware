/*
 * DSWR_FZ.h
 *
 *  Created on: Dec 29, 2025
 *      Author: liufangzheng
 */

#ifndef INC_DSWR_FZ_H_
#define INC_DSWR_FZ_H_

#include <stdint.h>
#include <stdbool.h>

#include "DW3000_FZ.h"
#include "DW3000_send_test_FZ.h"
#include "DW3000_recv_test_FZ.h"

typedef enum {
  DSWR_ROLE_ANCHOR = 0,
  DSWR_ROLE_TAG    = 1,
} dswr_role_t;

typedef struct {
  uint8_t  seq;
  uint32_t tof_dtu;
  float    distance_m;
  bool     valid;
} dswr_result_t;

void dswr_set_role(dswr_role_t role);
dswr_role_t dswr_get_role(void);

/**
 * @brief Run one double-sided ranging exchange (poll->resp) using current role.
 *
 * Blocking helper; assumes DW3000 is already initialised/IDLE and interrupts
 * drive DW3000_IRQ_flag. When role is anchor the function sends a poll,
 * waits for the response, and calculates distance. When role is tag it waits
 * for a poll and replies with timestamps (no distance calculation).
 *
 * @param result Optional output for anchor results (ignored for tag). May be NULL.
 * @return 0 on success, negative errno-style on failure/timeout.
 */
int dswr_run_once(dswr_result_t *result);

void dswr_print_result(const dswr_result_t *result);

#endif /* INC_DSWR_FZ_H_ */

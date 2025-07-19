/*
 * dw3000_device_api.c
 *
 *  Created on: Jul 18, 2025
 *      Author: liufangzheng
 */

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>

#include "dw3000_types.h"
#include "dw3000_regs.h"
#include "dw3000_device_api.h"
#include "dw3000_version.h"
#include "dw3000.h"


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function checks if the DW3000 is in IDLE_RC state
 *
 * input parameters
 *
 * output parameters
 *
 * return value is 1 if the IDLE_RC bit is set and 0 otherwise
 */
uint8_t dwt_checkidlerc(void) {
  //deca_sleep(2); /* wait 2 ms for DW IC to get into IDLE_RC state */
  /* Poll DW IC until IDLE_RC event set. This means that DW IC is in IDLE_RC state and ready */
  uint32_t reg = ((uint32_t)dwt_read16bitoffsetreg(SYS_STATUS_ID, 2) << 16);
  return ((reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}
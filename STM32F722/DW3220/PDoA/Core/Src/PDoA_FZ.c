/*
 * DSWR_FZ.c
 *
 * Double-sided wireless ranging helper (poll/resp) with selectable role.
 */
#include <math.h>
#include <errno.h>

#include "PDoA_FZ.h"
#include "dw3000_deca_regs.h"

static int wait_tx_done(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while (1) {
    if (DW3000_IRQ_flag) {
      DW3000_IRQ_flag = false;
      printf("TX done");
      uint32_t status = DW3000readreg(SYS_STATUS_ID, 4);
      if (status & SYS_STATUS_TXFRS_BIT_MASK) {
        DW3000_clear_IRQ();
        return 0;
      }
      DW3000_clear_IRQ();
    }
    if ((HAL_GetTick() - start) > timeout_ms) {
      return ETIMEDOUT;
    }
  }
}

static int send_frame(uint8_t *buf, uint16_t len) {
  DW3000_clear_IRQ();
  dwt_writetxdata(len, buf, 0);
  dwt_writetxfctrl(len + FCS_LEN, 0, 0); /* Zero offset in TX buffer, no ranging. */
  DW3000_txcmd_FZ(0);
  return wait_tx_done(10);
}

void pdoa_tx_frame(uint8_t *buf, uint16_t len) {
  if (send_frame(buf, len)) {
	  printf("TX overtime \r\n");
  }
}

void pdoa_rx_frame(int16_t *pdoa_val) {
  // if (DW3000_IRQ_flag) {
  //   DW3000_IRQ_flag = false;
  //   int16_t cpqual;

  //   /* Checking STS quality see note 4 */
  //   if (dwt_readstsquality(&cpqual)) {
  //     *pdoa_val = dwt_readpdoa();
  //   }
  //   dwt_writefastCMD(CMD_RX);

  //   printf("PDoA value: %d\r\n", *pdoa_val);
  //   frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  //   dwt_readrxdata(pdoa_datareceived, frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */
  //   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  //   printf("RX OK, length %d bytes: ", frame_len);
  //   for (size_t i = 0; i < frame_len; i++) {
  //     printf("0x%02X ", pdoa_datareceived[i]);
  //   }
  //   printf("\r\n");
  // }
}

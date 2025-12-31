/*
 * DSWR_FZ.c
 *
 * Double-sided wireless ranging helper (poll/resp) with selectable role.
 */
#include <math.h>
#include <errno.h>
#include <PDoA.h>

#include "dw3000_deca_regs.h"

// Frame types
#define DSWR_FRAME_POLL     0xE1
#define DSWR_FRAME_RESP     0xE2
#define DSWR_FRAME_RESP_TS  0xE3

// Basic poll/resp payload lengths (excludes 2B FCS).
#define DSWR_POLL_LEN     (2)   // type + seq
#define DSWR_RESP_LEN     (2)   // type + seq
#define DSWR_RESP_TS_LEN  (10)  // type + seq + poll_rx_ts(4) + resp_tx_ts(4)

// Time conversion: DW timestamp tick (~15.65ps) to meters.
#define DWT_TIME_UNITS   (1.0/499.2e6/128.0)
#define SPEED_OF_LIGHT   (299702547.0f)

static dswr_role_t dswr_role = DSWR_ROLE_ANCHOR;
static uint8_t dswr_seq = 0;

static uint32_t read_tx_ts(void) {
  return dwt_read32bitoffsetreg(TX_TIME_LO_ID, 0);
}

static uint32_t read_rx_ts(void) {
  return dwt_read32bitoffsetreg(RX_TIME_0_ID, 0);
}

static uint16_t get_frame_len(void) {
  return dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
}

static void pack_u32le(uint8_t *dst, uint32_t v) {
  dst[0] = (uint8_t)(v);
  dst[1] = (uint8_t)(v >> 8);
  dst[2] = (uint8_t)(v >> 16);
  dst[3] = (uint8_t)(v >> 24);
}

static uint32_t unpack_u32le(const uint8_t *src) {
  return ((uint32_t)src[0]) |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

static int wait_tx_done(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while (1) {
    if (DW3000_IRQ_flag) {
      DW3000_IRQ_flag = false;
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
  dwt_writetxfctrl(len + 2, 0, 1); // +2 for FCS
  DW3000_txcmd_FZ(0);
  return wait_tx_done(50);
}

static int wait_for_rx(uint8_t *rx_buf,
                       uint16_t rx_buf_len,
                       uint16_t *out_len,
                       uint32_t *rx_ts,
                       uint32_t timeout_ms)
{
  DW3000_clear_IRQ();
  DW3000_start_receiver_FZ();

  uint32_t start = HAL_GetTick();
  while (1) {
    if (DW3000_IRQ_flag) {
      DW3000_IRQ_flag = false;
      uint32_t status = DW3000readreg(SYS_STATUS_ID, 4);
      if (status & SYS_STATUS_RXFCG_BIT_MASK) {
        uint16_t frame_len = get_frame_len();
        if (frame_len < 2 || frame_len > rx_buf_len + 2) {
          DW3000_clear_IRQ();
          return -EMSGSIZE;
        }
        dwt_readrxdata(rx_buf, frame_len - 2, 0);
        if (out_len) *out_len = frame_len - 2;
        if (rx_ts) *rx_ts = read_rx_ts();
        DW3000_clear_IRQ();
        return 0;
      }
      // RX error of some kind
      DW3000_clear_IRQ();
      return EIO;
    }
    if (timeout_ms > 0 && (HAL_GetTick() - start) > timeout_ms) {
      return -ETIMEDOUT;
    }
  }
}

void dswr_set_role(dswr_role_t role) {
  dswr_role = role;
}

dswr_role_t dswr_get_role(void) {
  return dswr_role;
}

static int anchor_exchange(dswr_result_t *result) {
  uint8_t poll[DSWR_POLL_LEN] = {DSWR_FRAME_POLL, dswr_seq};
  uint8_t resp[DSWR_RESP_LEN] = {0};
  uint8_t resp_ts[DSWR_RESP_TS_LEN] = {0};
  uint16_t resp_len = 0;
  uint16_t resp_ts_len = 0;
  uint32_t poll_tx_ts = 0;
  uint32_t resp_rx_ts = 0;

  dwt_configuretxrf(0x34, 0xfdfdfdfd, 0x00);

  if (send_frame(poll, sizeof(poll)) != 0) {
    return -EIO;
  }
  poll_tx_ts = read_tx_ts();

  // First response just signals back (type + seq) so we can capture RX timestamp.
  int rx_status = wait_for_rx(resp, sizeof(resp), &resp_len, &resp_rx_ts, 200);
  if (rx_status != 0) {
    return rx_status;
  }

  if (resp_len < DSWR_RESP_LEN || resp[0] != DSWR_FRAME_RESP || resp[1] != dswr_seq) {
    return -EINVAL;
  }

  // Second frame carries responder timestamps for the first response.
  rx_status = wait_for_rx(resp_ts, sizeof(resp_ts), &resp_ts_len, NULL, 50);
  if (rx_status != 0) {
    return rx_status;
  }

  if (resp_ts_len < DSWR_RESP_TS_LEN || resp_ts[0] != DSWR_FRAME_RESP_TS || resp_ts[1] != dswr_seq) {
    return -EINVAL;
  }

  uint32_t poll_rx_ts_tag = unpack_u32le(&resp_ts[2]);
  uint32_t resp_tx_ts_tag = unpack_u32le(&resp_ts[6]);

  uint32_t Ra = resp_rx_ts - poll_tx_ts;
  uint32_t Db = resp_tx_ts_tag - poll_rx_ts_tag;

  uint32_t tof_dtu = (Ra - Db) / 2U;
  if (result) {
    result->seq = dswr_seq;
    result->tof_dtu = tof_dtu;
    result->distance_m = (float)(tof_dtu * DWT_TIME_UNITS * SPEED_OF_LIGHT);
    result->valid = true;
  }

  dswr_seq++;
  return 0;
}

static int tag_exchange(void) {
  uint8_t poll_buf[DSWR_POLL_LEN] = {0};
  uint16_t poll_len = 0;
  uint32_t poll_rx_ts = 0;
  uint8_t resp_buf[DSWR_RESP_LEN] = {0};
  uint8_t resp_ts_buf[DSWR_RESP_TS_LEN] = {0};

  int rx_status = wait_for_rx(poll_buf, sizeof(poll_buf), &poll_len, &poll_rx_ts, 0);
  if (rx_status != 0) {
    return rx_status;
  }

  if (poll_len < DSWR_POLL_LEN || poll_buf[0] != DSWR_FRAME_POLL) {
    return -EINVAL;
  }

  uint8_t seq = poll_buf[1];

  Delay_us(10);

  // Quick response so anchor can timestamp arrival.
  resp_buf[0] = DSWR_FRAME_RESP;
  resp_buf[1] = seq;
  int tx_status = send_frame(resp_buf, DSWR_RESP_LEN);
  if (tx_status != 0) {
    return tx_status;
  }

  uint32_t resp_tx_ts = read_tx_ts();

  HAL_Delay(2);

  // Send the timestamps captured for the previous response frame.
  resp_ts_buf[0] = DSWR_FRAME_RESP_TS;
  resp_ts_buf[1] = seq;
  pack_u32le(&resp_ts_buf[2], poll_rx_ts);
  pack_u32le(&resp_ts_buf[6], resp_tx_ts);

  tx_status = send_frame(resp_ts_buf, DSWR_RESP_TS_LEN);
  if (tx_status != 0) {
    return tx_status;
  }

  dswr_seq = seq + 1;
  return 0;
}

int dswr_run_once(dswr_result_t *result) {
  if (dswr_role == DSWR_ROLE_ANCHOR) {
    return anchor_exchange(result);
  }
  return tag_exchange();
}

void dswr_print_result(const dswr_result_t *result) {
  if (!result || !result->valid) {
    printf("DSWR: no valid result\r\n");
    return;
  }
  printf("distance: %.3f m\r\n", result->distance_m);
}

#pragma once

#include <stdint.h>
#include <stdlib.h>

typedef enum {
  COMM_CMD_ID_PING         = 0x00,
  COMM_CMD_ID_TURN_ON_CAM  = 0x01,
  COMM_CMD_ID_TURN_OFF_CAM = 0x02,
  COMM_CMD_ID_TAKE_PHOTO   = 0x03,
  COMM_CMD_ID_SEND_DATA    = 0x04,
} comm_cmd_id_t;

typedef enum {
  COMM_RADIO_LORA = 0x00,
  COMM_RADIO_FLRC = 0x01,
} comm_radio_t;

struct comm_cmd_t {
  uint8_t  node_id;
  uint8_t  seq_num;
  uint8_t  cmd_id;
  uint32_t cmd_param;
  uint8_t  next_radio;
  uint8_t  checksum;
} __attribute__((packed));
typedef struct comm_cmd_t comm_cmd_t;

struct comm_ack_t {
  uint8_t  node_id;
  uint8_t  seq_num;
  uint8_t  ack_id;
  uint32_t cmd_param;
  uint8_t  next_radio;
  uint8_t  checksum;
} __attribute__((packed));
typedef struct comm_ack_t comm_ack_t;

struct comm_data_t {
  uint8_t       node_id;
  uint16_t      frame_seq_num;
  uint16_t      frame_left;
  uint32_t      bytes_left;
  uint8_t       payload[500];
  uint8_t       checksum;
} __attribute__((packed));
typedef struct comm_data_t comm_data_t;





#pragma once

#include <Arduino.h>

#include "lr2021.h"

#define CMD_LEN_MAX 50
#define CMD_PARAM_LEN 11
#define DATA_PAYLOAD_MAX 500

enum CMD_ID : uint8_t {
  PING            = 0x00,
  TURN_ON_CAMERA  = 0x01,
  TURN_OFF_CAMERA = 0x02,
  TAKE_PICTURE    = 0x03,
  GET_DATA        = 0x14,
};

struct __attribute__((packed)) hs_cmd_t {
  uint8_t node_id;
  uint8_t seq_num;
  uint8_t cmd_id;
  uint8_t cmd_param[CMD_PARAM_LEN];
  uint8_t checksum;
};

struct __attribute__((packed)) hs_ack_t {
  uint8_t node_id;
  uint8_t seq_num;
  uint8_t ack_id;
  uint8_t ack_param[CMD_PARAM_LEN];
  uint8_t checksum;
};

struct __attribute__((packed)) hs_data_t {
  uint8_t  node_id;
  uint8_t  frame_seq_num;
  uint16_t frame_left;
  uint32_t bytes_left;
  uint8_t  payload[DATA_PAYLOAD_MAX];
  uint8_t  checksum;
};

static_assert(sizeof(hs_cmd_t) == 15, "hs_cmd_t must match STM32 comm_cmd_t");
static_assert(sizeof(hs_ack_t) == 15, "hs_ack_t must match STM32 comm_ack_t");
static_assert(sizeof(hs_data_t) == 509, "hs_data_t must match STM32 comm_data_t");

extern hs_cmd_t  current_cmd;
extern hs_ack_t  current_ack;
extern hs_data_t current_data;

uint16_t cobs_encode(const uint8_t* input, uint16_t length, uint8_t* output);
uint16_t cobs_decode(const uint8_t* input, uint16_t length,
                     uint8_t* output, uint16_t output_capacity);

extern LR2021 radio;
extern const lr2021_flrc_config_t FLRC_CFG;
extern const lr2021_lora_config_t LORA_CFG;

enum class RadioMode { FLRC, LORA };

extern RadioMode current_mode;

extern uint8_t tx_seq;
extern uint8_t rx_buf[LR2021_MAX_FLRC_PAYLOAD];
extern uint8_t tx_buf[100];

extern void flrc_tx_test(const uint8_t *data, uint16_t len);
extern void lora_tx_test(const uint8_t *data, uint16_t len);

void rx_test();

extern void update_comm(void);
void handle_cmd(const hs_cmd_t& cmd, const uint8_t* encoded_cmd,
                uint16_t encoded_len);

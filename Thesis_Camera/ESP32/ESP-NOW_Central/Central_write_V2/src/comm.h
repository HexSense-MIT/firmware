#pragma once

// central mac address A0:85:E3:E3:56:40

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "node_addr.h"

#define MAC_ADDR_INDEX 4
#define MAC_ADDR_SIZE  6

int cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length);
int cobs_encode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length);

extern void comm_init(void);
extern void update_comm(uint8_t *cmd_buf, size_t &count);
extern void handle_cmd(const uint8_t *cmd_buf, size_t count);

#define ACK_BUFFER_SIZE 250

extern volatile bool received_ack_flag;
extern uint8_t ack_buffer[ACK_BUFFER_SIZE];
extern volatile int ack_length;

struct GUICommandPacket {
  uint8_t header[2];
  uint8_t cmd;
  uint8_t camera_index;
  uint8_t mac[6];
} __attribute__((packed));

struct CommandPacket {
  uint8_t header[2] = {0xEB, 0x90};
  uint8_t cmd;
  uint8_t camera_index;
  uint8_t crc;
} __attribute__((packed));

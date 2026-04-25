#pragma once

// central mac address A0:85:E3:E3:56:40

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "node_addr.h"

#define MAC_ADDR_INDEX 4
#define MAC_ADDR_SIZE  6

#define IMAGE_BUFFER_SIZE     (500 * 1024) // Example image size (500KB)

int cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length);
int cobs_encode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length);

extern void comm_init(void);

extern void update_comm(uint8_t *cmd_buf, size_t &count);
extern void handle_cmd(const uint8_t *cmd_buf, size_t count);

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

struct AckPacket {
  uint8_t header[2];
  uint8_t cmd;
  uint8_t camera_index;
  uint8_t image_size[4];
  uint8_t crc;
} __attribute__((packed));

struct DataPacket {
  uint8_t  header[2] = {0xEB, 0x92};
  uint16_t seq;
  uint8_t  mac[6];
  uint8_t  camera_index;
  uint16_t bytes_left;
  uint8_t  payload[200];
  uint16_t crc;
} __attribute__((packed));

enum CMD_TYPE {
  CAM_ON = 0x01,
  CAM_OFF,
  TAKE_PHOTO,
  SEND_DATA,
  IGNORE = 0xFF,
  IDLE
};

#define ESPNOW_RECV_BUF_SIZE 250
#define ESPNOW_ACK_TIMEOUT_MS 5000

extern volatile bool   espnow_recv_ready;
extern volatile int    espnow_recv_len;
extern uint8_t         espnow_recv_buf[ESPNOW_RECV_BUF_SIZE];


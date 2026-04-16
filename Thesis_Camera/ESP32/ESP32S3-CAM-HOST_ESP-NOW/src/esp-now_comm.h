#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "central_address.h"

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
  uint8_t mac[6];
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

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len);

extern void ESPNOW_comm_init(void);
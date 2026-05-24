#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <HardwareSerial.h>
#include <esp_heap_caps.h>

#include "central_address.h"

struct CommandPacket {
  uint8_t header[2] = {0xEB, 0x90};
  uint8_t cmd;
  uint8_t camera_index;
  uint8_t crc;
} __attribute__((packed));

struct AckPacket {
  uint8_t header[2] = {0xEB, 0x91};
  uint8_t cmd;
  uint8_t camera_index;
  uint8_t image_size[4];
  uint8_t crc;
} __attribute__((packed));

struct DataPacket {
  uint8_t  header[2] = {0xEB, 0x92};
  uint16_t seq;
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

extern CMD_TYPE cmd_recv;

#define CMD_LENGTH sizeof(CommandPacket)

extern volatile bool received_cmd;
extern uint8_t cmd_raw[sizeof(CommandPacket) + 1];

extern CommandPacket cmdpkg_recv;

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len);

extern void parse_cmd(uint8_t *data, CommandPacket *cmd);
extern void ESPNOW_comm_init(void);

#define CMD_LEN           10

#define WRONG_CMD_CODE    0xFF

#define CAPTURE_CMD       0x01
#define SEND_CAM_DATA_CMD 0x02

#define TURN_ON_CAM_CODE    0x01
#define TURN_OFF_CAM_CODE   0x02
#define TAKE_PHOTO_CAM_CODE 0x03
#define GRAB_DATA_CAM_CODE  0x04

#define CAM_TAKE_PHOTO_TIMES 5

#define IMG_SIZE 1024 * 500  // 100 KB buffer for image data

extern int update_comm(void);
extern void handle_cmd(CommandPacket *cmdpck);

void pack_ack(uint8_t cmd, uint8_t camera_index, uint32_t img_size);
void pack_error(uint8_t error_code);
void pack_data(uint8_t* data, uint64_t len);

void send_ack(AckPacket* ack);
void send_data(DataPacket* data);
void send_reply(uint8_t* data, uint64_t len);
void send_photo(uint8_t* data, uint64_t len);

void turn_on_a_camera(uint8_t cam_num);
uint64_t take_photos(int times);
void turn_off_a_camera(uint8_t cam_num);

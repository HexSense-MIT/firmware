#include "comm.h"

extern uint8_t LED_TX_PIN;
extern uint8_t LED_RX_PIN;

uint8_t ack_buffer[ACK_BUFFER_SIZE];
volatile int ack_length = 0;
volatile bool received_ack_flag = false;

int cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length) {
  size_t in_index  = 0;
  size_t out_index = 0;

  while (in_index < length) {
    uint8_t code = input[in_index++];
    if (code == 0 || in_index + code - 1 > length) {
      output_length = 0;
      return 0;
    }
    for (uint8_t i = 1; i < code; i++) {
      output[out_index++] = input[in_index++];
    }
    if (code < 0xFF && in_index < length) {
      output[out_index++] = 0;
    }
  }
  output_length = out_index;
  return 1;
}

int cobs_encode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length) {
  size_t in_index  = 0;
  size_t out_index = 0;
  size_t code_pos  = out_index++;
  uint8_t code     = 1;

  while (in_index < length) {
    if (input[in_index] == 0x00) {
      output[code_pos] = code;
      code_pos = out_index++;
      code = 1;
      in_index++;
    } else {
      output[out_index++] = input[in_index++];
      code++;
      if (code == 0xFF) {
        output[code_pos] = code;
        code_pos = out_index++;
        code = 1;
      }
    }
  }
  output[code_pos] = code;
  output[out_index++] = 0x00;
  output_length = out_index;
  return 1;
}

void update_comm(uint8_t *cmd_buf, size_t &count) {
  static uint8_t temp_buf[20];
  static size_t temp_count = 0;

  if (Serial.available()) {
    uint8_t b = Serial.read();
    if (b == 0x00) {
      if (cobs_decode(temp_buf, temp_count, cmd_buf, count) != 1) {
        count = 0;
      }
      temp_count = 0;
    } else {
      temp_buf[temp_count++] = b;
    }
  }
}

void handle_cmd(const uint8_t *cmd_buf, size_t count) {
  if (count != sizeof(GUICommandPacket) || cmd_buf[0] != 0xEB || cmd_buf[1] != 0x90) {
    return;
  }

  uint8_t dest_mac[6] = {0};
  memcpy(dest_mac, cmd_buf + MAC_ADDR_INDEX, 6);

  CommandPacket cmd;
  memcpy(&cmd, cmd_buf, 4);
  uint8_t crc = 0;
  for (size_t i = 0; i < 4; i++) {
    crc += cmd_buf[i];
  }
  cmd.crc = crc;

  uint8_t encoded_buf[20];
  size_t  encoded_length = 0;
  cobs_encode((uint8_t*)&cmd, sizeof(cmd), encoded_buf, encoded_length);
  esp_now_send(dest_mac, encoded_buf, encoded_length);
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  memcpy(ack_buffer, data, len);
  ack_length = len;
  received_ack_flag = true;
}

void comm_init(void) {
  WiFi.mode(WIFI_STA);

  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer_1 = {};
  esp_now_peer_info_t peer_2 = {};
  esp_now_peer_info_t peer_3 = {};
  esp_now_peer_info_t peer_4 = {};

  memcpy(peer_1.peer_addr, HexSense_addr_1, 6); esp_now_add_peer(&peer_1);
  memcpy(peer_2.peer_addr, HexSense_addr_2, 6); esp_now_add_peer(&peer_2);
  memcpy(peer_3.peer_addr, HexSense_addr_3, 6); esp_now_add_peer(&peer_3);
  memcpy(peer_4.peer_addr, HexSense_addr_4, 6); esp_now_add_peer(&peer_4);
}

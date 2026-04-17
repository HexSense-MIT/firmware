#include "esp-now_comm.h"
#include <cstdio>

int cobs_decode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length) {
  size_t in_index  = 0;
  size_t out_index = 0;

  while (in_index < length) {
    uint8_t code = input[in_index++];
    if (code == 0 || in_index + code - 1 > length) {
      // Invalid COBS encoding
      output_length = 0;
      return 0;
    }
    for (uint8_t i = 1; i < code; i++) {
      output[out_index++] = input[in_index++];
    }
    if (code < 0xFF && in_index < length) {
      output[out_index++] = 0; // Insert zero byte
    }
  }
  output_length = out_index;
  return 1;
}

int cobs_encode(const uint8_t *input, size_t length, uint8_t *output, size_t &output_length) {
  size_t in_index  = 0;
  size_t out_index = 0;
  size_t code_pos  = out_index++;  // reserve slot for first code byte
  uint8_t code     = 1;

  while (in_index < length) {
    if (input[in_index] == 0x00) {
      output[code_pos] = code;     // write code at its actual position
      code_pos = out_index++;      // reserve next code byte slot
      code = 1;
      in_index++;
    } else {
      output[out_index++] = input[in_index++];
      code++;
      if (code == 0xFF) {          // max run length reached
        output[code_pos] = code;
        code_pos = out_index++;
        code = 1;
      }
    }
  }
  output[code_pos] = code;        // write final code byte
  output[out_index++] = 0x00;     // frame delimiter
  output_length = out_index;
  return 1;
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  int i = 0;

  printf("Get cmd from mac: ");
  for (i = 0; i < 6; i ++) {
    printf("0x%02X ", *(mac_addr+i));
  }
  printf("\n");

  printf("Get data length = %d\n", len);
  printf("Get cmd: ");
  for (i = 0; i < len; i ++) {
    printf("0x%02X ", *(data+i));
  }
  printf("\n");
}

void ESPNOW_comm_init(void) {
  WiFi.mode(WIFI_STA);

  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, central_addr, 6);
  esp_now_add_peer(&peer);
}

#include "comm.h"

uint8_t *image_data_buf = (uint8_t*) heap_caps_malloc(IMAGE_BUFFER_SIZE*(sizeof(uint8_t)), MALLOC_CAP_SPIRAM);

CMD_TYPE cmd_recv = IDLE;
AckPacket short_ack;
DataPacket long_ack;

volatile uint32_t receivedBytes = 0;
volatile uint16_t lastSeq       = 0;
volatile uint16_t lastBytesLeft = 0;
volatile bool     newPacket     = false;
volatile bool     allDone       = false;

volatile bool  espnow_recv_ready = false;
volatile int   espnow_recv_len   = 0;
uint8_t        espnow_recv_buf[ESPNOW_RECV_BUF_SIZE];

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

void update_comm(uint8_t *cmd_buf, size_t &count) {
  static uint8_t temp_buf[20];
  static size_t temp_count = 0;

  if (Serial.available()) {
    uint8_t b = Serial.read();
    if (b == 0x00) {
      if (cobs_decode(temp_buf, temp_count, cmd_buf, count) != 1) {
        // Handle decoding error
      }
      temp_count = 0;
    } else {
      temp_buf[temp_count++] = b;
    }
  }
}

void handle_short_ack(CMD_TYPE &cmd, AckPacket* ack) {
  uint32_t start = millis();

  while (!espnow_recv_ready) {
    if (millis() - start > ESPNOW_ACK_TIMEOUT_MS) {
      printf("handle_short_ack: timeout\n");
      cmd = IDLE;
      return;
    }
  }

  if (espnow_recv_len < (int)sizeof(AckPacket)) {
    printf("handle_short_ack: packet too short (%d bytes)\n", espnow_recv_len);
    espnow_recv_ready = false;
    cmd = IDLE;
    return;
  }

  uint8_t recv_buf_cobs_encoded[ESPNOW_RECV_BUF_SIZE] = {0};
  uint8_t recv_buf_cobs_decoded[ESPNOW_RECV_BUF_SIZE] = {0};
  size_t  recv_buf_decoded_len = 0;
  memcpy(recv_buf_cobs_encoded, espnow_recv_buf, espnow_recv_len);
  cobs_decode(recv_buf_cobs_encoded, espnow_recv_len, recv_buf_cobs_decoded, recv_buf_decoded_len);
  memcpy(ack, recv_buf_cobs_decoded, sizeof(AckPacket));

  // for (size_t i = 0; i < sizeof(AckPacket) - 1; i++) {
  //   printf("%02X ", ((uint8_t*)ack)[i]);
  // }

  espnow_recv_ready = false;

  printf("handle_short_ack: cmd=0x%02X cam=%d\n", ack->cmd, ack->camera_index);

  if (ack->cmd == TAKE_PHOTO) {
    uint32_t image_size = (ack->image_size[3] << 24) | (ack->image_size[2] << 16) | (ack->image_size[1] << 8) | ack->image_size[0];
    printf("Image size: %u bytes\n", image_size);
  }
}

void handle_long_ack(DataPacket* ack) {
  if (!image_data_buf) {
    printf("handle_long_ack: PSRAM buffer not allocated\n");
    return;
  }

  receivedBytes     = 0;
  allDone           = false;
  newPacket         = false;
  espnow_recv_ready = false;

  uint32_t total_written = 0;
  uint32_t start = millis();

  while (!allDone) {
    if (millis() - start > 2 * ESPNOW_ACK_TIMEOUT_MS) {
      printf("handle_long_ack: timeout after %u bytes\n", total_written);
      return;
    }

    if (!newPacket) continue;

    newPacket         = false;
    espnow_recv_ready = false;

    memcpy(ack, espnow_recv_buf, sizeof(DataPacket));

    if (ack->header[0] != 0xEB || ack->header[1] != 0x92) {
      printf("handle_long_ack: bad header at seq %d\n", ack->seq);
      continue;
    }

    uint32_t offset = (uint32_t)ack->seq * sizeof(ack->payload);
    if (offset + sizeof(ack->payload) <= IMAGE_BUFFER_SIZE) {
      memcpy(image_data_buf + offset, ack->payload, sizeof(ack->payload));
      total_written = offset + sizeof(ack->payload);
    } else {
      printf("handle_long_ack: buffer overflow at seq %d (offset=%u)\n", ack->seq, offset);
    }

    start = millis();  // reset timeout on each received packet
    printf("handle_long_ack: seq=%d bytes_left=%d\n", ack->seq, ack->bytes_left);
  }

  // Prefer the image size reported by the camera during TAKE_PHOTO, fall back to bytes written
  uint32_t image_size = ((uint32_t)short_ack.image_size[3] << 24) |
                        ((uint32_t)short_ack.image_size[2] << 16) |
                        ((uint32_t)short_ack.image_size[1] <<  8) |
                         (uint32_t)short_ack.image_size[0];
  uint32_t print_size = (image_size > 0 && image_size <= IMAGE_BUFFER_SIZE) ? image_size : total_written;

  printf("handle_long_ack: done, sending %u bytes via serial\n", print_size);
  // Serial.write(image_data_buf, print_size);
  // print all the bytes in image_data_buf
  // for (uint32_t i = 0; i < print_size; i++) {
  //   printf("%02X", image_data_buf[i]);
  // }
  // printf("\n\n");
}

void handle_cmd(const uint8_t *cmd_buf, size_t count) {
  if (count != sizeof(GUICommandPacket) || cmd_buf[0] != 0xEB || cmd_buf[1] != 0x90) {
    // Invalid command packet
    cmd_recv = IDLE;
    return;
  }
  else {
    uint8_t dest_mac[6] = {0};
    memcpy(dest_mac, cmd_buf + MAC_ADDR_INDEX, 6);
    CommandPacket cmd;
    memcpy(&cmd, cmd_buf, 4);
    uint8_t crc = 0;
    for (size_t i = 0; i < 4; i++) {
      crc += cmd_buf[i];
    }
    cmd.crc = crc;

    cmd_recv = (CMD_TYPE) cmd.cmd;

    uint8_t encoded_buf[20];
    size_t  encoded_length = 0;

    cobs_encode((uint8_t*)&cmd, sizeof(cmd), encoded_buf, encoded_length);

    esp_err_t status = esp_now_send(dest_mac, encoded_buf, encoded_length);

    // if (status == ESP_OK)                        printf("ESP_OK\n");
    // else if (status == ESP_ERR_ESPNOW_NOT_INIT)  printf("ESP_ERR_ESPNOW_NOT_INIT\n");
    // else if (status == ESP_ERR_ESPNOW_ARG)       printf("ESP_ERR_ESPNOW_ARG\n");
    // else if (status == ESP_ERR_ESPNOW_NO_MEM)    printf("ESP_ERR_ESPNOW_NO_MEM\n");
    // else if (status == ESP_ERR_ESPNOW_FULL)      printf("ESP_ERR_ESPNOW_FULL\n");
    // else if (status == ESP_ERR_ESPNOW_NOT_FOUND) printf("ESP_ERR_ESPNOW_NOT_FOUND\n");
    // else if (status == ESP_ERR_ESPNOW_INTERNAL)  printf("ESP_ERR_ESPNOW_INTERNAL\n");
    // else if (status == ESP_ERR_ESPNOW_EXIST)     printf("ESP_ERR_ESPNOW_EXIST\n");
    // else                                         printf("nothing\n");

    if (cmd_recv != SEND_DATA) handle_short_ack(cmd_recv, &short_ack);
    else                       handle_long_ack(&long_ack);
  }
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  int copy_len = (len < ESPNOW_RECV_BUF_SIZE) ? len : ESPNOW_RECV_BUF_SIZE;
  memcpy(espnow_recv_buf, data, copy_len);
  espnow_recv_len   = copy_len;
  espnow_recv_ready = true;

  if (copy_len >= (int)sizeof(DataPacket)) {
    DataPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    receivedBytes += 200;
    lastSeq       = pkt.seq;
    lastBytesLeft = pkt.bytes_left;
    newPacket     = true;
    if (pkt.bytes_left == 0) allDone = true;
  }
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











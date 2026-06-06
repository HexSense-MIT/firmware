// #include <FS.h>
// #include <SD.h>

#include <WiFi.h>
#include <esp_now.h>

#include "cam_system.h"
#include "cam_comm.h"
#include "cam_adapter.h"
#include <cstdio>

CMD_TYPE cmd_recv = IDLE;
volatile bool received_cmd = false;
uint8_t cmd_raw[CMD_LENGTH + 1];

#define CHUNK_SIZE 100

volatile bool recv_cmd_flag = false;
CommandPacket cmdpkg_recv;

uint8_t recv_cmd[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t reply_data[10] = {0xEB, 0x91, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t cam_num = 0;
uint8_t cam_data = 0;

uint8_t *img_buffer = (uint8_t*) heap_caps_malloc(IMG_SIZE*(sizeof(uint8_t)), MALLOC_CAP_SPIRAM);

uint64_t data_len = 0; // Length of the data to be sent

uint64_t recv_data_i = 0;
uint64_t img_data_len = 0;

AckPacket ackpkg_send;
DataPacket datapkg_send;

uint16_t data_seq = 0;
uint8_t data_camera_index = 0;

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

void parse_cmd(uint8_t *data, CommandPacket *cmd) {
  uint8_t cmd_buffer[CMD_LENGTH];
  size_t  cmd_decode_len = 0;

  cobs_decode(data, CMD_LENGTH + 1, cmd_buffer, cmd_decode_len);

  memcpy((uint8_t*)cmd, cmd_buffer, cmd_decode_len);
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (len != (CMD_LENGTH + 2)) return;

  memcpy(cmd_raw, data, len - 1); // get rid of the 0x00 at the end
  received_cmd = true;
}

void ESPNOW_comm_init(void) {
  WiFi.mode(WIFI_STA);

  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, central_addr, 6);
  esp_now_add_peer(&peer);
}

void pack_ack(uint8_t cmd, uint8_t camera_index, uint32_t img_size) {
  ackpkg_send.header[0] = 0xEB;
  ackpkg_send.header[1] = 0x91;
  ackpkg_send.cmd = cmd;
  ackpkg_send.camera_index = camera_index;
  *(uint32_t*)ackpkg_send.image_size = img_size;
  // calculate checksum
  uint8_t crc = 0;
  uint8_t* ptr = (uint8_t*)&ackpkg_send;
  for (size_t i = 0; i < sizeof(ackpkg_send) - 1; i++) {
    crc ^= ptr[i];
  }
  ackpkg_send.crc = crc;
}

void pack_error(uint8_t error_code) {
  reply_data[2] = error_code;
}

void send_ack(AckPacket*ack) {
  // COBS encode the ackpkg_send and send it via ESP-NOW
  uint8_t encoded_buf[sizeof(AckPacket) * 2]; // Worst case buffer size for COBS encoding
  size_t encoded_length = 0;
  cobs_encode((uint8_t*)ack, sizeof(AckPacket), encoded_buf, encoded_length);
  esp_now_send(central_addr, encoded_buf, encoded_length);
}

void pack_data(uint8_t* data, uint64_t len, uint64_t remaining_after) {
  size_t payload_len = min((size_t)len, sizeof(datapkg_send.payload));

  datapkg_send.header[0] = 0xEB;
  datapkg_send.header[1] = 0x92;
  datapkg_send.seq = data_seq++;
  datapkg_send.camera_index = data_camera_index;
  datapkg_send.bytes_left = (remaining_after > 0xFFFF) ? 0xFFFF : (uint16_t)remaining_after;
  memset(datapkg_send.payload, 0, sizeof(datapkg_send.payload));
  memcpy(datapkg_send.payload, data, payload_len);

  // calculate checksum
  uint16_t crc = 0;
  uint8_t* ptr = (uint8_t*)&datapkg_send;
  for (size_t i = 0; i < sizeof(datapkg_send) - sizeof(datapkg_send.crc); i++) {
    crc ^= ptr[i];
  }
  datapkg_send.crc = crc;
}

void send_data(DataPacket*data) {
  // COBS encode the datapkg_send and send it via ESP-NOW
  // uint8_t encoded_buf[sizeof(DataPacket) * 2]; // Worst case buffer size for COBS encoding
  // size_t encoded_length = 0;
  // cobs_encode((uint8_t*)data, sizeof(DataPacket), encoded_buf, encoded_length);
  // esp_now_send(central_addr, encoded_buf, encoded_length);

  // send data without COBS encoding (since we have a crc and fixed packet size, we can rely on that for integrity)
  esp_now_send(central_addr, (uint8_t*)data, sizeof(DataPacket));
}

void send_photo(uint8_t* data, uint64_t len) {
  data_seq = 0;

  uint64_t data_i = 0;
  while (data_i < len) {
    size_t packet_len = min((uint64_t)sizeof(datapkg_send.payload), len - data_i);
    uint64_t remaining_after = len - data_i - packet_len;
    pack_data(data + data_i, packet_len, remaining_after);
    send_data(&datapkg_send);
    data_i += packet_len;
    // printf("Sent data packet: seq=%d, camera_index=%d, bytes_left=%d, payload_len=%d\n", datapkg_send.seq, datapkg_send.camera_index, datapkg_send.bytes_left, packet_len);
    delay(10); // Short delay between packets to avoid overwhelming the receiver
    yield();
  }
}

int update_comm(void) {
  if (received_cmd) {
    received_cmd = false;
    parse_cmd(cmd_raw, &cmdpkg_recv);
    handle_cmd(&cmdpkg_recv);
  }

  yield(); // Yield to allow other tasks to run (important for ESP-NOW callbacks)

  return 1;
}

void handle_cmd(CommandPacket *cmdpck) {
  if (cmdpck->cmd == CAM_ON) { // turn on the camera
    turnoffallcams(); // Ensure all cameras are off before turning on a specific one
    delay(10);

    cam_num = cmdpck->camera_index; // Get the camera number from the command

    if (cam_num < 6) {
      // printf("Camera ON: %d\n", cam_num);
      delay(100);
      turnoncam(cam_num + 1); // Call the function to turn on the camera
      pack_ack(CAM_ON, cam_num, 0); // Pack acknowledgment for successful operation
      send_ack(&ackpkg_send); // Send acknowledgment reply
      delay(1000);    // Wait for 1 second to ensure the camera is powered on
      flush_buffer(); // Flush the serial buffer to clear any remaining data
    }
  }
  else if (cmdpck->cmd == CAM_OFF) { // turn off the camera
    cam_num = cmdpck->camera_index; // Get the camera number from the command

    if (cam_num < 6) {
      // printf("Camera OFF: %d\n", cam_num);
      delay(100);
      turnoffallcams(); // Call the function to turn off all cameras
      pack_ack(CAM_OFF, cam_num, 0); // Pack acknowledgment for successful operation
      send_ack(&ackpkg_send); // Send acknowledgment reply
    }
  }
  else if (cmdpck->cmd == TAKE_PHOTO) { // take a photo
    // printf("Take a photo with camera: %d\n", cmdpck->camera_index);
    delay(100);

    for (int i = 0; i < CAM_TAKE_PHOTO_TIMES; i++) {
      Serial1.write(CAPTURE_CMD);
      Serial1.flush();

      uint32_t t0 = millis();
      while (!Serial1.available()) {
        if (millis() - t0 > 5000) {
          Serial.println("Timeout waiting for camera response.");
          return;
        }
      }
      yield();

      recv_data_i = 0;
      while (Serial1.available()) {
        cam_data = Serial1.read();
        reply_data[recv_data_i++] = cam_data; // Store the length of the data
      }
      img_data_len = reply_data[1] | (reply_data[2] << 8) | (reply_data[3] << 16) | (reply_data[4] << 24);
    }
    pack_ack(TAKE_PHOTO, cmdpck->camera_index, img_data_len); // Pack acknowledgment with image size
    send_ack(&ackpkg_send); // Send acknowledgment reply
    // printf("Photo taken. Data length: %llu bytes\n", img_data_len);
    delay(100);
  }
  else if (cmdpck->cmd == SEND_DATA) { // send photo data
    Serial1.setTimeout(150); // slightly above the 100ms inter-chunk gap
    Serial1.write(SEND_CAM_DATA_CMD);
    Serial1.flush();

    uint64_t len = img_data_len;

    recv_data_i = 0;

    while (!Serial1.available()) {yield();}  // wait for first byte

    size_t total = 0;
    while (total < len) {
      size_t chunk = Serial1.readBytes(img_buffer + total, min((size_t)(len - total), (size_t)CHUNK_SIZE));
      if (chunk == 0) break; // timeout — no more data
      total += chunk;
      yield();
    }
    len = total;
    data_camera_index = cmdpck->camera_index;
    send_photo(img_buffer, len);

    // char hex_chunk[2 * CHUNK_SIZE + 1]; // CHUNK_SIZE bytes * 2 hex chars + null terminator
    // while (recv_data_i < len) {
    //   size_t chunk_size = min((uint64_t)CHUNK_SIZE, len - recv_data_i);
    //   for (size_t j = 0; j < chunk_size; j++) {
    //     sprintf(hex_chunk + j * 2, "%02X", img_buffer[recv_data_i++]);
    //   }
    //   hex_chunk[chunk_size * 2] = '\0';
    //   Serial.print(hex_chunk);
    // }

    // Serial.println();
  }
  else {
    // Serial.println("Invalid command received.");
    // printf("Invalid command received.\n");
    pack_error(WRONG_CMD_CODE); // Pack error for invalid command
    // send_reply(reply_data, sizeof(reply_data)); // Send error reply
  }
}

void turn_on_a_camera(uint8_t cam_num) {
  turnoffallcams();           // Turn all cameras off at the start of each loop
  delay(10);                 // Wait for 1 second to ensure the camera is powered on
  turnoncam(cam_num);    // Call the function to turn on the specified camera (cam_num is 1-indexed)
  delay(100);                // Wait for 1 second to ensure the camera is powered on
  flush_buffer();             // Flush the serial buffer to clear any remaining data
  delay(10);                 // Wait for 1 second to ensure the camera is powered on
}

void print_img_for_tst(uint8_t* data, uint64_t len) {
  Serial1.setTimeout(10); // slightly above the 100ms inter-chunk gap
  Serial1.write(SEND_CAM_DATA_CMD);
  Serial1.flush();

  recv_data_i = 0;

  while (!Serial1.available()) {yield();}  // wait for first byte
  size_t total = 0;
  while (total < len) {
    size_t chunk = Serial1.readBytes(data + total, min((size_t)(len - total), (size_t)CHUNK_SIZE));
    if (chunk == 0) break; // timeout — no more data
    total += chunk;
  }
  len = total;

  char hex_chunk[2 * CHUNK_SIZE + 1]; // CHUNK_SIZE bytes * 2 hex chars + null terminator
  while (recv_data_i < len) {
    size_t chunk_size = min((uint64_t)CHUNK_SIZE, len - recv_data_i);
    for (size_t j = 0; j < chunk_size; j++) {
      sprintf(hex_chunk + j * 2, "%02X", data[recv_data_i++]);
    }
    hex_chunk[chunk_size * 2] = '\0';
    Serial.print(hex_chunk);
  }

  // while (recv_data_i < len) {
  //   if (Serial1.available()) {
  //     char hex_str[3];
  //     sprintf(hex_str, "%02X", Serial1.read());
  //     Serial.print(hex_str);
  //     recv_data_i++;
  //   }
  // }

  // recv_data_i = 0;
  // while (recv_data_i < data_len) {
  //   Serial.print(img_buffer[recv_data_i]);
  //   recv_data_i++;
  // }

  Serial.println();
}

uint64_t take_photos(int times) {
  uint64_t img_data_len = 0;
  for (int i = 0; i < times; i++) {
    Serial1.write(CAPTURE_CMD);
    Serial1.flush();

    uint32_t t0 = millis();
    while (!Serial1.available()) {
      if (millis() - t0 > 5000) {
        Serial.println("Timeout waiting for camera response.");
        return 0;
      }
      yield();
    }

    recv_data_i = 0;
    while (Serial1.available()) {
      cam_data = Serial1.read();
      reply_data[recv_data_i++] = cam_data; // Store the length of the data
    }
    img_data_len = reply_data[1] | (reply_data[2] << 8) | (reply_data[3] << 16) | (reply_data[4] << 24);

    // delay(10); // Short delay between captures
  }

  print_img_for_tst(img_buffer, img_data_len); // Print the image data for testing

  return img_data_len;
}

// void store_img2sd(const char* filename) {
//   Serial1.write(SEND_CAM_DATA_CMD);
//   Serial1.flush();

//   recv_data_i = 0;

//   // Grab all image data from the camera module and store it in img_buffer
//   while (recv_data_i < data_len) {
//     if (Serial1.available()) {
//       img_buffer[recv_data_i++] = Serial1.read();
//     }
//   }

//   img2sd(filename, img_buffer, data_len);
// }

void turn_off_a_camera(uint8_t cam_num) {
  turnoffallcams();
}
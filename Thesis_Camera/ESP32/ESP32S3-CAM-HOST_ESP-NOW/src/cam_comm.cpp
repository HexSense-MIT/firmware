// #include <FS.h>
// #include <SD.h>

#include <WiFi.h>
#include <esp_now.h>

#include "cam_system.h"
#include "cam_comm.h"
#include "cam_adapter.h"
#include <cstdio>

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

void img2sd(const char* filename, uint8_t* data, uint64_t len) {
  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    // Write raw JPEG bytes
    file.write(data, len);  // raw JPEG buffer to SD
    file.close();
  }
}

void pack_ack(uint8_t ack_code) {
  reply_data[2] = ack_code;
  reply_data[3] = cam_num; 
}

void pack_error(uint8_t error_code) {
  reply_data[2] = error_code;
}

void pack_data(uint8_t* data, uint64_t len) {
  memcpy(reply_data + 1, data, len);
}

void send_reply(uint8_t* data, uint64_t len) {
  Serial.write(reply_data, len); // Send the reply data including the header
  Serial.flush(); // Ensure the data is sent immediately
}

int update_comm(void) {
  if (received_cmd) {
    received_cmd = false;
    parse_cmd(cmd_raw, &cmdpkg_recv);
    // cmd_recv = (CMD_TYPE)cmdpkg_recv.cmd;
    handle_cmd(&cmdpkg_recv);
  }

  return 1;
}

// turn_on_a_camera(current_cam_num);
// Serial.print("Camera ");  Serial.print(current_cam_num);
// Serial.println(" turned on. Taking photos...");

// photo_data_len = take_photos(6);
// Serial.print("Photo data length: ");
// Serial.println(photo_data_len);

// turn_off_a_camera(current_cam_num);
// Serial.println("Photo capture complete.");

// delay(1000); // Short delay before the next command

// current_cam_num ++;
// if (current_cam_num > 6) {
//   current_cam_num = 1;
// }

void handle_cmd(CommandPacket *cmdpck) {
  if (cmdpck->cmd == CAM_ON) { // turn on the camera
    turnoffallcams(); // Ensure all cameras are off before turning on a specific one
    delay(10);

    cam_num = cmdpck->camera_index; // Get the camera number from the command

    if (cam_num < 6) {
      turnoncam(cam_num + 1); // Call the function to turn on the camera
      pack_ack(TURN_ON_CAM_CODE); // Pack acknowledgment for successful operation
      send_reply(reply_data, sizeof(reply_data)); // Send acknowledgment reply
      delay(1000);    // Wait for 1 second to ensure the camera is powered on
      flush_buffer(); // Flush the serial buffer to clear any remaining data
    }
    // else { // not allowed to turn on all cameras at once
    //   pack_error(WRONG_CMD_CODE); // Pack error for invalid camera number
    //   send_reply(reply_data, sizeof(reply_data)); // Send error reply
    // }
  }
  else if (cmdpck->cmd == CAM_OFF) { // turn off the camera
    cam_num = cmdpck->camera_index; // Get the camera number from the command

    if (cam_num < 6) {
      turnoffallcams(); // Call the function to turn off all cameras
      pack_ack(TURN_OFF_CAM_CODE); // Pack acknowledgment for successful operation
      send_reply(reply_data, sizeof(reply_data)); // Send acknowledgment reply
    }
    // else {
    //   pack_error(WRONG_CMD_CODE); // Pack error for invalid camera number
    //   send_reply(reply_data, sizeof(reply_data)); // Send error reply
    //   return;
    // }
  }
  else if (cmdpck->cmd == TAKE_PHOTO) { // take a photo
    Serial1.write(CAPTURE_CMD);
    Serial1.flush();

    while (!Serial1.available()) {;}

    recv_data_i = 0;
    while (Serial1.available()) {
      cam_data = Serial1.read();
      reply_data[recv_data_i++] = cam_data; // Store the length of the data
    }
    data_len = reply_data[1] | (reply_data[2] << 8) | (reply_data[3] << 16) | (reply_data[4] << 24);
    send_reply(reply_data, sizeof(reply_data)); // Send acknowledgment reply
  }
  else if (cmdpck->cmd == SEND_DATA) { // send photo data
    Serial1.write(SEND_CAM_DATA_CMD);
    Serial1.flush();

    recv_data_i = 0;

    // Grab all image data from the camera module and store it in img_buffer
    while (recv_data_i < data_len) {
      if (Serial1.available()) {
        img_buffer[recv_data_i++] = Serial1.read();
      }
    }
  }
  else {
    // Serial.println("Invalid command received.");
    pack_error(WRONG_CMD_CODE); // Pack error for invalid command
    send_reply(reply_data, sizeof(reply_data)); // Send error reply
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

  while (!Serial1.available()) {;}  // wait for first byte
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





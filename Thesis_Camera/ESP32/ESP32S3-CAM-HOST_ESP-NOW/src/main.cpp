// comm with GUI through USB.
// Store image data in PSRAM first then into SD card.
#include <Arduino.h>

#include "cam_system.h"
#include "cam_comm.h"
#include "esp-now_comm.h"

// #include <FS.h>
// #include <SD_MMC.h>
// #include <esp_sleep.h>

int     current_cam_num = 1; // Default camera number
uint64_t photo_data_len = 0;

void setup() {
  system_init();
  ESPNOW_comm_init();
}

void loop() {
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
}





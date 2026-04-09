// comm with GUI through USB.
// Store image data in PSRAM first then into SD card.
#include <Arduino.h>

#include "cam_system.h"
#include "cam_comm.h"
#include <FS.h>
#include <SD_MMC.h>
#include <esp_sleep.h>

int     current_cam_num = 1; // Default camera number
int32_t file_name_num   = 1;
constexpr uint64_t LIGHT_SLEEP_US = 10ULL * 1000000ULL; // 10 seconds

uint16_t bat_voltage = 0;
uint8_t bat_v_pin = 5;

uint64_t photo_data_len = 0;

void enter_light_sleep(uint64_t sleep_us) {
  esp_sleep_enable_timer_wakeup(sleep_us);
  esp_light_sleep_start(); // Returns after wake-up
}

void setup() {
  system_init();
}

void loop() {
  update_comm();
  // camadapter_pwr_on(); // Power on the camera adapter
  turn_on_a_camera(current_cam_num);
  Serial.print("Camera ");  Serial.print(current_cam_num);
  Serial.println(" turned on. Taking photos...");
  photo_data_len = take_photos(3);
  Serial.print("Photo data length: ");
  Serial.println(photo_data_len);
  turn_off_a_camera(current_cam_num);
  Serial.println("Photo capture complete.");

  delay(1000); // Short delay before the next command

  current_cam_num ++;
  if (current_cam_num > 6) {
    current_cam_num = 1;
  }
}





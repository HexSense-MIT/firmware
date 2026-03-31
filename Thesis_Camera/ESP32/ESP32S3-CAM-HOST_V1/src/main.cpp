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

void enter_light_sleep(uint64_t sleep_us) {
  esp_sleep_enable_timer_wakeup(sleep_us);
  esp_light_sleep_start(); // Returns after wake-up
}

void setup() {
  system_init();
}

void loop() {
  update_comm();

  // turnoffallcams();
  // enter_light_sleep(LIGHT_SLEEP_US);

  // current_cam_num = 1;

  // while (current_cam_num < 7) {
  //   turn_on_a_camera(current_cam_num);
  //   take_photos(5);

  //   char filename[20];
  //   sprintf(filename, "/%d.jpg", file_name_num);
  //   // Ensure unique filename
  //   while (SD.exists(filename)) {
  //     file_name_num ++;
  //     sprintf(filename, "/%d.jpg", file_name_num);
  //   }

  //   turn_off_a_camera(current_cam_num);

  //   current_cam_num ++;
  // }
}





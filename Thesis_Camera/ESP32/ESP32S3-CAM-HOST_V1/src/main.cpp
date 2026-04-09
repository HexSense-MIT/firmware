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
}





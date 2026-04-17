// comm with GUI through USB.
// Store image data in PSRAM first then into SD card.
#include <Arduino.h>

#include "cam_system.h"
#include "cam_comm.h"
#include "esp-now_comm.h"

void setup() {
  system_init();
  ESPNOW_comm_init();
}

void loop() {
  update_comm();
}





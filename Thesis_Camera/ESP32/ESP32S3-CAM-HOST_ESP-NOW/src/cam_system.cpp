#include "cam_system.h"

void system_init(void) {
  // Serial.begin(921600);
  Serial1.begin(500000, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

  // while (!Serial) {
  //   delay(10);
  //   yield(); // Allow other tasks to run while waiting for Serial to initialize
  // }
  while (!Serial1) {
    delay(10);
    yield();
  }

  // burning_init(); // Initialize the burning system
  // burning(0, 6000); // Burn side 0 for 10 s

  camadapter_init(); // Initialize the camera adapter
  turnoffallcams();  // Ensure all cameras are off at startup
  // camadapter_pwr_off(); // Power off the camera adapter to save power until needed

  // flush Serial1 to clear any initial data
  while (Serial1.available()) {
    Serial1.read(); // Clear any initial data in the buffer
  }

  delay(100); // Short delay to ensure everything is initialized properly

  // Serial.println("System ready. Waiting for commands...");
}





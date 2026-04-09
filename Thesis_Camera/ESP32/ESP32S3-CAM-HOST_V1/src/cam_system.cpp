#include "cam_system.h"

void system_init(void) {
  Serial.begin(500000);
  Serial1.begin(500000, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

  while (!Serial)  delay(10);
  while (!Serial1) delay(10);

  // burning_init(); // Initialize the burning system
  // burning(0, 30000); // Burn side 0 for 10 s

  camadapter_init(); // Initialize the camera adapter
  turnoffallcams();  // Ensure all cameras are off at startup
  // camadapter_pwr_off(); // Power off the camera adapter to save power until needed

  // flush Serial1 to clear any initial data
  while (Serial1.available()) {
    Serial1.read(); // Clear any initial data in the buffer
  }

  Serial.println("System ready. Waiting for commands...");
}




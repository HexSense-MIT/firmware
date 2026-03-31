#include "cam_system.h"

SFE_PCA95XX io_bur; // Global instance of the PCA95XX I2C expander

static void burning_ON(void) {
  digitalWrite(BUR_EN, HIGH);
  delay(100); // Allow some time for the burning system to power up
}

static void burning_OFF(void) {
  digitalWrite(BUR_EN, LOW);
  delay(100); // Allow some time for the burning system to power down
}

static void burning(int side_num, int delay_ms) {
  burning_ON();
  io_bur.digitalWrite(side_num, HIGH);
  delay(delay_ms);
  io_bur.digitalWrite(side_num, LOW);
  burning_OFF();
}

void burning_init(void) {
  pinMode(BUR_EN, OUTPUT); // Set the burning enable pin as OUTPUT
  burning_ON(); // Power on the burning system

  Wire.begin(); // Initialize I2C with default SDA and SCL pins

  // Initialize the PCA95XX I2C expander for burning
  if (io_bur.begin(PCA9554_ADDRESS_BUR_20) == false) {
    Serial.println("Burning PCA95xx not detected. Please check wiring. Freezing...");
    while (true);
  }

  Serial.println("Burning system initialized.");
}

// void SD_init(void) {
//   // Enable internal pull-ups on SPI pins
//   pinMode(SD_SCK, INPUT_PULLUP);
//   pinMode(SD_MISO, INPUT_PULLUP);
//   pinMode(SD_MOSI, INPUT_PULLUP);
//   pinMode(SD_CS, INPUT_PULLUP);

//   SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

//   if (!SD.begin(SD_CS)) {
//     // Serial.println("SD init failed");
//     while (1);
//   }
// }

void system_init(void) {
  Serial.begin(921600);
  Serial1.begin(921600, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);

  while (!Serial)  delay(10);
  while (!Serial1) delay(10);

  burning_init(); // Initialize the burning system
  burning(0, 5000); // Burn side 0 for 5 s

  camadapter_init(); // Initialize the camera adapter
  turnoffallcams();  // Ensure all cameras are off at startup
  camadapter_pwr_off(); // Power off the camera adapter to save power until needed

  // flush Serial1 to clear any initial data
  while (Serial1.available()) {
    Serial1.read(); // Clear any initial data in the buffer
  }

  Serial.println("System ready. Waiting for commands...");
}




#include "cam_adapter.h"

SFE_PCA95XX io; // Global instance of the PCA95XX I2C expander

extern void camadapter_pwr_on(void) {
  digitalWrite(CAM_ADP_EN, HIGH); // Set the enable pin HIGH to power on the camera adapter
  delay(100); // Allow some time for the camera adapter to power up
}

extern void camadapter_pwr_off(void) {
  digitalWrite(CAM_ADP_EN, LOW); // Set the enable pin LOW to power off the camera adapter
  delay(100); // Allow some time for the camera adapter to power down
}

extern void camadapter_init(void) {
  pinMode(CAM_ADP_EN, OUTPUT); // Set the enable pin as OUTPUT
  camadapter_pwr_on(); // Power on the camera adapter

  Wire.begin(CAM_SDA_PIN, CAM_SCL_PIN); // Initialize I2C with specified SDA and SCL pins

  // Initialize the PCA95XX I2C expander
  if (io.begin(PCA9554_ADDRESS_21) == false) {
    Serial.println("camera adapter PCA95xx not detected. Please check wiring. Freezing...");
    while (true);
  }

  // Set all camera pins to OUTPUT mode
  for (int i = 0; i < 8; i++) {
    io.pinMode(i, OUTPUT);
  }

  Serial.println("Camera adapter initialized.");
}

extern void turnoncam(uint8_t cam) {
  if (cam < 0 || cam >= 8) {
    // Serial.println("Invalid camera index. Must be between 0 and 7.");
    return;
  }

  // Set the corresponding pin to HIGH to turn on the camera
  io.digitalWrite(cam, HIGH);
}

extern void turnoffcam(uint8_t cam) {
  if (cam < 0 || cam >= 8) {
    // Serial.println("Invalid camera index. Must be between 0 and 7.");
    return;
  }

  // Set the corresponding pin to LOW to turn off the camera
  io.digitalWrite(cam, LOW);
}

extern void turnoffallcams(void) {
  // Turn off all cameras by setting all pins to LOW
  for (int i = 0; i < 8; i++) {
    io.digitalWrite(i, LOW);
  }
}

extern void flush_buffer(void) {
  while (Serial1.available()) {
    Serial1.read(); // Read and discard all available bytes in the serial buffer
  }
}
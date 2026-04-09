#include "sys_bur.h"

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

  Wire.begin(CAM_SDA_PIN, CAM_SCL_PIN); // Initialize I2C with custom SDA and SCL pins

  // Initialize the PCA95XX I2C expander for burning
  if (io_bur.begin(PCA9554_ADDRESS_BUR_20) == false) {
    Serial.println("Burning PCA95xx not detected. Please check wiring. Freezing...");
    while (true);
  }

  Serial.println("Burning system initialized.");
}
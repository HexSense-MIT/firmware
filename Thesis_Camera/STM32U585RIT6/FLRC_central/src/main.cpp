#include <Arduino.h>

#include "comm.h"

// ============================================================
void setup() {
  Serial.begin(921600);
  while (!Serial) {
    delay(10);
  }

  radio.begin();
  current_mode = RadioMode::LORA; // for start, select LoRa

  radio.configLoRa(LORA_CFG);
  // if (current_mode == RadioMode::FLRC) {
  //   radio.configFLRC(FLRC_CFG);
  //   Serial.println("[OK] Radio initialised in FLRC mode");
  // } else {
  //   radio.configLoRa(LORA_CFG);
  //   Serial.println("[OK] Radio initialised in LoRa mode");
  // }
}

// ============================================================
void loop() {
  update_comm();
  // rx_test();
  // lora_tx_test();
  // delay(1000);
}

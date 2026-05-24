#include <Arduino.h>
#include "lr2021.h"

// ============================================================
//  Pin mapping — Seeed XIAO RP2040
//  SPI0 default: SCK=D8(GPIO2), MISO=D9(GPIO4), MOSI=D10(GPIO3)
//
//  LR2021 module wiring:
//    NSS   → D1 (GPIO27)
//    RESET → D2 (GPIO28)
//    BUSY  → D3 (GPIO29)
//    DIO5  → D0 (GPIO26)   ← IRQ output of LR2021
//    SCK   → D8
//    MISO  → D9
//    MOSI  → D10
//    VCC   → 3.3V
//    GND   → GND
// ============================================================

LR2021 radio;   // uses LR2021_DEFAULT_PIN_* from lr2021.h

// ---- FLRC config — 650 kbps, 915 MHz, CRC on ----
static const lr2021_flrc_config_t FLRC_CFG = {
  .freq_hz      = 915000000UL,
  .br_bw        = LR2021_FLRC_BR_0650_BW_0667,
  .cr           = LR2021_FLRC_CR_1_2,
  .shape        = LR2021_FLRC_SHAPE_BT05,
  .syncword     = {0xDE, 0xAD, 0xBE, 0xEF},
  .crc          = true,
  .tx_power_dbm = 14,
};

// ---- LoRa config — SF7, 125 kHz, 915 MHz ----
static const lr2021_lora_config_t LORA_CFG = {
  .freq_hz          = 915000000UL,
  .sf               = LR2021_LORA_SF7,
  .bw               = LR2021_LORA_BW_125_KHZ,
  .cr               = LR2021_LORA_CR_4_5,
  .preamble_symbols = 8,
  .syncword         = LR2021_LORA_SYNCWORD_PRIVATE,
  .crc              = true,
  .iq_inverted      = false,
  .implicit_hdr     = false,
  .tx_power_dbm     = 14,
};

// ---- Demo state ----
enum class RadioMode { FLRC, LORA };
static RadioMode current_mode = RadioMode::FLRC;

static uint8_t  tx_seq  = 0;
static uint8_t  rx_buf[255];
static uint8_t  tx_buf[16];

// ============================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial) {delay(10);}

  Serial.println("LR2021 driver demo — Seeed XIAO RP2040");
  Serial.println("Mode: FLRC 650 kbps / LoRa SF7 125 kHz, 915 MHz");

  if (!radio.begin()) {
      Serial.println("[ERROR] LR2021 init failed — check wiring");
      while (true) delay(500);
  }

  // Start in FLRC mode
  radio.configFLRC(FLRC_CFG);
  Serial.println("[OK] Radio initialised in FLRC mode");
}

// ============================================================
void loop() {
  // ----- Transmit a small packet -----
  tx_buf[0] = tx_seq++;
  tx_buf[1] = (current_mode == RadioMode::FLRC) ? 0xF0 : 0xA0;
  for (uint8_t i = 2; i < 8; i++) tx_buf[i] = i;

  Serial.printf("[TX] seq=%u mode=%s  ", tx_buf[0],
                (current_mode == RadioMode::FLRC) ? "FLRC" : "LoRa");

  bool ok = radio.transmit(tx_buf, 8, 3000);
  Serial.println(ok ? "OK" : "TIMEOUT");

  // ----- Listen for a reply -----
  lr2021_rx_status_t status;
  int rx_len = radio.receive(rx_buf, sizeof(rx_buf), &status, 2000);

  if (rx_len > 0) {
    Serial.printf("[RX] %d bytes  RSSI=%d dBm", rx_len, (int)status.rssi_dbm);
    if (current_mode == RadioMode::LORA) {
      Serial.printf("  SNR=%d dB", (int)status.snr);
    }
    Serial.printf("  CRC=%s\r\n", status.crc_ok ? "OK" : "ERR");
    Serial.print("     data:");
    for (int i = 0; i < rx_len && i < 16; i++) {
      Serial.printf(" %02X", rx_buf[i]);
    }
    Serial.println();
  } else {
      Serial.println("[RX] timeout / no packet");
  }

  // ----- Toggle mode every 3 packets -----
  if ((tx_seq % 3) == 0) {
    if (current_mode == RadioMode::FLRC) {
      current_mode = RadioMode::LORA;
      radio.configLoRa(LORA_CFG);
      Serial.println("--- Switched to LoRa mode ---");
    } else {
      current_mode = RadioMode::FLRC;
      radio.configFLRC(FLRC_CFG);
      Serial.println("--- Switched to FLRC mode ---");
    }
  }

  delay(500);
}

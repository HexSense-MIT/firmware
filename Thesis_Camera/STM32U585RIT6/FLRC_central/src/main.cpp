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

// ---- FLRC config — 650 kbps / 888 kHz BW, 915 MHz, CRC on ----
static const lr2021_flrc_config_t FLRC_CFG = {
  .freq_hz      = 915000000UL,
  .br_bw        = LR2021_FLRC_BR_0650_BW_0888,
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

enum class RadioMode { FLRC, LORA };
static RadioMode current_mode = RadioMode::FLRC;

static uint8_t tx_seq      = 0;
static uint8_t mode_count  = 0;   // packets sent in current mode
static uint8_t rx_buf[255];
static uint8_t tx_buf[16];

// ============================================================
static void flrc_tx_test() {
  tx_buf[0] = tx_seq++;
  tx_buf[1] = 0xF0;
  for (uint8_t i = 2; i < 8; i++) tx_buf[i] = i;

  Serial.printf("[FLRC TX] seq=%u  ", tx_buf[0]);
  bool ok = radio.transmit(tx_buf, 8, 3000);
  Serial.println(ok ? "OK" : "TIMEOUT");

  lr2021_rx_status_t status;
  int rx_len = radio.receive(rx_buf, sizeof(rx_buf), &status, 500);

  if (rx_len > 0) {
    Serial.printf("[FLRC RX] %d bytes  RSSI=%d dBm  CRC=%s\r\n",
                  rx_len, (int)status.rssi_dbm, status.crc_ok ? "OK" : "ERR");
    Serial.print("          data:");
    for (int i = 0; i < rx_len && i < 16; i++) Serial.printf(" %02X", rx_buf[i]);
    Serial.println();
  } else {
    Serial.println("[FLRC RX] timeout / no packet");
  }
}

// ============================================================
static void lora_tx_test() {
  tx_buf[0] = tx_seq++;
  tx_buf[1] = 0xA0;
  for (uint8_t i = 2; i < 8; i++) tx_buf[i] = i;

  Serial.printf("[LoRa TX] seq=%u  ", tx_buf[0]);
  bool ok = radio.transmit(tx_buf, 8, 5000);
  Serial.println(ok ? "OK" : "TIMEOUT");

  lr2021_rx_status_t status;
  int rx_len = radio.receive(rx_buf, sizeof(rx_buf), &status, 500);

  if (rx_len > 0) {
    Serial.printf("[LoRa RX] %d bytes  RSSI=%d dBm  SNR=%d dB  CRC=%s\r\n",
                  rx_len, (int)status.rssi_dbm, (int)status.snr,
                  status.crc_ok ? "OK" : "ERR");
    Serial.print("          data:");
    for (int i = 0; i < rx_len && i < 16; i++) Serial.printf(" %02X", rx_buf[i]);
    Serial.println();
  } else {
    Serial.println("[LoRa RX] timeout / no packet");
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("LR2021 driver demo — Seeed XIAO RP2040");
  Serial.println("FLRC 650 kbps / LoRa SF7 125 kHz, 915 MHz — 3 packets per mode");

  if (!radio.begin()) {
    Serial.println("[ERROR] LR2021 init failed — check wiring");
    while (true) delay(500);
  }

  radio.configFLRC(FLRC_CFG);
  Serial.println("[OK] Radio initialised in FLRC mode");
}

// ============================================================
void loop() {
  if (current_mode == RadioMode::FLRC) {
    flrc_tx_test();
  } else {
    lora_tx_test();
  }

  mode_count++;
  if (mode_count >= 3) {
    mode_count = 0;
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

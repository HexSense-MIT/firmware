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

// ---- FLRC config: 650 kbps / 740 kHz BW, 915 MHz, CRC on ----
static const lr2021_flrc_config_t FLRC_CFG = {
  .freq_hz      = 915000000UL,
  .br_bw        = LR2021_FLRC_BR_0650_BW_0740,
  .cr           = LR2021_FLRC_CR_1_2,
  .shape        = LR2021_FLRC_SHAPE_BT1,
  .syncword     = {0xDE, 0xAD, 0xBE, 0xEF},
  .crc          = true,
  .tx_power_dbm = 14,
};

// ---- LoRa config — SF7, 250 kHz, 915 MHz ----
static const lr2021_lora_config_t LORA_CFG = {
  .freq_hz          = 915000000UL,
  .sf               = LR2021_LORA_SF7,
  .bw               = LR2021_LORA_BW_250_KHZ,
  .cr               = LR2021_LORA_CR_4_5,
  .preamble_symbols = 8,
  .syncword         = LR2021_LORA_SYNCWORD_PRIVATE,
  .crc              = true,
  .iq_inverted      = false,
  .implicit_hdr     = false,
  .payload_len      = 0,
  .tx_power_dbm     = 14,
};

enum class RadioMode { FLRC, LORA };
static RadioMode current_mode = RadioMode::FLRC;

static uint8_t tx_seq      = 0;
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

static void rx_test() {
  lr2021_rx_status_t status;
  int rx_len = radio.receive(rx_buf, sizeof(rx_buf), &status, 5000);

  const char* mode_name = (current_mode == RadioMode::FLRC) ? "FLRC" : "LoRa";

  if (rx_len > 0) {
    Serial.printf("[%s RX] %d bytes RSSI=%d dBm", mode_name, rx_len, (int)status.rssi_dbm);
    if (current_mode == RadioMode::LORA) {
      Serial.printf(" SNR=%d dB", (int)status.snr);
    } else {
      Serial.printf(" SYNC=%s", status.flrc_sync_ok ? "OK" : "ERR");
    }
    Serial.printf(" CRC=%s ", status.crc_ok ? "OK" : "ERR");
    Serial.print(" data:");
    for (int i = 0; i < rx_len; i++) {
      // print in char
      Serial.printf("%c", rx_buf[i]);
    }
    Serial.println();
  } else {
    Serial.printf("[%s RX] no payload  len=%u", mode_name, (unsigned)status.length);
    if (status.irq_flags == LR2021_IRQ_NONE) {
      Serial.print("  host timeout");
    } else {
      Serial.printf("  IRQ=0x%08lX", (unsigned long)status.irq_flags);
      if (status.irq_flags & LR2021_IRQ_TIMEOUT) Serial.print(" TIMEOUT");
      if (status.irq_flags & LR2021_IRQ_CRC_ERROR) Serial.print(" CRC_ERROR");
      if (status.irq_flags & LR2021_IRQ_LEN_ERROR) Serial.print(" LEN_ERROR");
      if (status.irq_flags & LR2021_IRQ_LORA_HEADER_ERROR) Serial.print(" HEADER_ERROR");
      if (status.irq_flags & LR2021_IRQ_CMD_ERROR) Serial.print(" CMD_ERROR");
      if (status.irq_flags & LR2021_IRQ_ERROR) Serial.print(" RADIO_ERROR");
      if (status.system_errors != 0) {
        Serial.printf(" SYSERR=0x%04X", (unsigned)status.system_errors);
        if (status.system_errors & LR2021_SYSERR_CHIP_BUSY) Serial.print(" CHIP_BUSY");
        if (status.system_errors & LR2021_SYSERR_RXFREQ_NO_FE_CALIB) Serial.print(" NO_FE_CALIB");
        if (status.system_errors & LR2021_SYSERR_PLL_LOCK) Serial.print(" PLL_LOCK");
        if (status.system_errors & LR2021_SYSERR_HF_XOSC_START) Serial.print(" HF_XOSC");
      }
      if (current_mode == RadioMode::FLRC && (status.irq_flags & LR2021_IRQ_RX_DONE)) {
        Serial.printf("  SYNC=%s", status.flrc_sync_ok ? "OK" : "ERR");
      }
      Serial.printf("  RSSI=%d dBm", (int)status.rssi_dbm);
    }
    Serial.println();
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("LR2021 driver demo — Seeed XIAO RP2040");
  Serial.println("Press F for FLRC or L for LoRa within 5 seconds.");
  Serial.println("Default is FLRC.");

  if (!radio.begin()) {
    Serial.println("[ERROR] LR2021 init failed — check wiring");
    while (true) delay(500);
  }

  uint32_t start_ms = millis();
  while ((millis() - start_ms) < 5000UL) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'L' || c == 'l') {
        current_mode = RadioMode::LORA;
        break;
      }
      if (c == 'F' || c == 'f') {
        current_mode = RadioMode::FLRC;
        break;
      }
    }
    delay(10);
  }

  if (current_mode == RadioMode::FLRC) {
    radio.configFLRC(FLRC_CFG);
    Serial.println("[OK] Radio initialised in FLRC mode");
  } else {
    radio.configLoRa(LORA_CFG);
    Serial.println("[OK] Radio initialised in LoRa mode");
  }
}

// ============================================================
void loop() {
  rx_test();
  // if (current_mode == RadioMode::FLRC) {
  //   flrc_tx_test();
  // } else {
  //   lora_tx_test();
  // }
  // delay(1500);
}

#include "comm.h"

hs_cmd_t  current_cmd  = {};
hs_ack_t  current_ack  = {};
hs_data_t current_data = {};

uint16_t cobs_encode(const uint8_t* input, uint16_t length, uint8_t* output) {
  uint16_t read_index = 0;
  uint16_t write_index = 1;
  uint8_t code = 1;

  while (read_index < length) {
    if (input[read_index] == 0) {
      output[write_index - code] = code;
      code = 1;
      write_index++;
      read_index++;
    } else {
      output[write_index] = input[read_index];
      write_index++;
      read_index++;
      code++;
      if (code == 0xFF) {
        output[write_index - code] = code;
        code = 1;
        write_index++;
      }
    }
  }
  output[write_index - code] = code;
  return write_index;
}

uint16_t cobs_decode(const uint8_t* input, uint16_t length,
                     uint8_t* output, uint16_t output_capacity) {
  uint16_t read_index = 0;
  uint16_t write_index = 0;

  while (read_index < length) {
    uint8_t code = input[read_index++];
    if (code == 0 || read_index + code - 1U > length) {
      return 0; // Invalid COBS data
    }

    for (uint8_t i = 1; i < code; i++) {
      if (write_index >= output_capacity) {
        return 0;
      }
      output[write_index] = input[read_index];
      write_index++;
      read_index++;
    }

    if (code != 0xFF && read_index < length) {
      if (write_index >= output_capacity) {
        return 0;
      }
      output[write_index] = 0;
      write_index++;
    }
  }
  return write_index;
}

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
const lr2021_flrc_config_t FLRC_CFG = {
  .freq_hz      = 915000000UL,
  .br_bw        = LR2021_FLRC_BR_2600_BW_2666,
  .cr           = LR2021_FLRC_CR_1_2,
  .shape        = LR2021_FLRC_SHAPE_BT1,
  .syncword     = {0xDE, 0xAD, 0xBE, 0xEF},
  .crc          = true,
  .tx_power_dbm = 14,
};

// ---- LoRa config — SF7, 250 kHz, 915 MHz ----
const lr2021_lora_config_t LORA_CFG = {
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

RadioMode current_mode = RadioMode::FLRC;

uint8_t tx_seq = 0;
uint8_t rx_buf[LR2021_MAX_FLRC_PAYLOAD];
uint8_t tx_buf[100];

// ============================================================
void flrc_tx_test(const uint8_t *data, uint16_t len) {
  if (data && len > 0) {
    radio.transmit(data, len, 100);
    return;
  } else {
    tx_buf[0] = tx_seq++;
    tx_buf[1] = 0xF0;
    for (uint8_t i = 2; i < 8; i++) tx_buf[i] = i;
    len = 8;
  }

  bool ok = radio.transmit(tx_buf, len, 100);
}

// ============================================================
void lora_tx_test(const uint8_t *data, uint16_t len) {
  if (data && len > 0) {
    radio.transmit(data, len, 100);
    return;
  } else {
    tx_buf[0] = tx_seq ++;
    tx_buf[1] = 0xA0;
    for (uint8_t i = 2; i < 100; i++) tx_buf[i] = i;
    len = 100;
  }

  bool ok = radio.transmit(tx_buf, len, 100);
}

void rx_test() {
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

void update_comm(void) {
  // keep listening to serialport for commands from host
  // the commands are COBS encoded hs_cmd_t structs, terminated by a zero byte
  static uint8_t serial_buf[CMD_LEN_MAX + 1U];
  static uint16_t serial_buf_len = 0;
  static bool serial_overflow = false;

  while (Serial.available() > 0) {
    int value = Serial.read();
    if (value < 0) {
      break;
    }
    uint8_t byte = (uint8_t)value;

    if (byte == 0) { // end of COBS packet
      if (!serial_overflow && serial_buf_len > 0) {
        hs_cmd_t cmd = {};
        uint16_t decoded_len = cobs_decode(serial_buf, serial_buf_len,
                                           (uint8_t*)&cmd, sizeof(cmd));

        if (decoded_len == sizeof(hs_cmd_t)) {
          current_cmd = cmd;
          serial_buf[serial_buf_len] = 0;
          handle_cmd(current_cmd, serial_buf, serial_buf_len + 1U);
        }
      }

      serial_buf_len = 0; // reset buffer for next packet
      serial_overflow = false;
      continue;
    }

    if (serial_overflow) {
      continue;
    }

    if (serial_buf_len < CMD_LEN_MAX) {
      serial_buf[serial_buf_len++] = byte;
    } else {
      serial_buf_len = 0;
      serial_overflow = true;
    }
  }
}

void handle_cmd(const hs_cmd_t& cmd, const uint8_t* encoded_cmd, uint16_t encoded_len) {
  switch (cmd.cmd_id) {
    case CMD_ID::PING:
    case CMD_ID::TURN_ON_CAMERA:
    case CMD_ID::TURN_OFF_CAMERA:
    case CMD_ID::TAKE_PICTURE:
      current_mode = RadioMode::LORA; // for start, select LoRa
      radio.configLoRa(LORA_CFG);
      lora_tx_test(encoded_cmd, encoded_len);
      break;

    case CMD_ID::GET_DATA:
      current_mode = RadioMode::FLRC; // for start, select FLRC
      radio.configFLRC(FLRC_CFG);
      flrc_tx_test(encoded_cmd, encoded_len);
      break;

    default:
      break;
  }

}

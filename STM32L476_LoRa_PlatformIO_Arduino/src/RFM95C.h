#pragma once

#include <Arduino.h>

#include "system.h"

// Register definitions (partial; add more as needed)
#define RFM95_REG_OP_MODE         0x01
#define RFM95_REG_FRF_MSB         0x06
#define RFM95_REG_FRF_MID         0x07
#define RFM95_REG_FRF_LSB         0x08
#define RFM95_REG_PA_CONFIG       0x09
#define RFM95_REG_FIFO            0x00
#define RFM95_REG_FIFO_ADDR_PTR   0x0D
#define RFM95_REG_FIFO_TX_BASE    0x0E
#define RFM95_REG_FIFO_RX_CURRENT 0x10
#define RFM95_REG_RX_NB_BYTES     0x13
#define RFM95_REG_PKT_RSSI_VALUE  0x1A
#define RFM95_REG_IRQ_FLAGS       0x12
#define RFM95_REG_VERSION         0x42  // Expected version for RFM95 (example value)

// Operation mode values (for LoRa mode)
// Standby: 0x81, TX: 0x83, RX Continuous: 0x85, Sleep: 0x80 (these values are typical)
#define RFM95_MODE_SLEEP          0x80
#define RFM95_MODE_STANDBY        0x81
#define RFM95_MODE_TX             0x83
#define RFM95_MODE_RX_CONTINUOUS  0x85

// define an enum for LoRa frequency
enum LoRaFreq {
  FREQ_433 = 433,
  FREQ_868 = 868,
  FREQ_915 = 915
};

class RFM95C {
  public:
    // Constructor: provide chip-select, reset and DIO0 pins
    RFM95C(SPI_HandleTypeDef* spiHandle, uint8_t csPin, uint8_t resetPin, uint8_t dio0Pin);

    // Initialize the module; set frequency (in MHz) and perform a basic check
    bool begin(float frequency);

    // Set carrier frequency (in MHz)
    void setFrequency(float frequency);

    // Set transmission power (in dBm); if useRFO is true, use RFO pin (rarely used)
    void setTxPower(int8_t power, bool useRFO = false);

    // Set the module to standby mode
    void standby();

    // Set the module to sleep mode
    void sleep();

    // Read the received signal strength indicator (RSSI)
    int8_t readRSSI();

    // Transmit a payload (blocking until TX done or timeout)
    // Returns true on success
    bool transmit(const uint8_t* data, uint8_t length);

    // Receive a payload (blocking until a packet is received or timeout)
    // 'length' will be set to the number of bytes received.
    // Returns true on success.
    bool receive(uint8_t* buffer, uint8_t* length);

  private:
    // SPI handle
    SPI_HandleTypeDef* _spi;

    uint8_t _csPin;
    uint8_t _resetPin;
    uint8_t _dio0Pin;

    // Helper functions for SPI read/write
    uint8_t readRegister(uint8_t reg);
    void    writeRegister(uint8_t reg, uint8_t value);

    // Perform a hardware reset on the module
    void explicitReset();
};

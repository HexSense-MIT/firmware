#include "RFM95C.h"

RFM95C::RFM95C(SPI_HandleTypeDef* spiHandle, uint8_t csPin, uint8_t resetPin, uint8_t dio0Pin)
  : _spi(spiHandle), _csPin(csPin), _resetPin(resetPin), _dio0Pin(dio0Pin) {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH); // deselect SPI

  pinMode(_resetPin, OUTPUT);
  digitalWrite(_resetPin, HIGH);

  pinMode(_dio0Pin, INPUT);
}

bool RFM95C::begin(float frequency) {
  // Perform a hardware reset
  explicitReset();

  // Give the chip a moment to initialize
  delay(10);

  // Check the version register; if it does not match, initialization fails.
  uint8_t ver = readRegister(RFM95_REG_VERSION);
  if (ver != 0x12) {  // 0x12 is an example expected value for RFM95 modules
    return false;
  }

  // Set frequency and switch to standby mode
  setFrequency(frequency);
  standby();

  return true;
}

void RFM95C::setFrequency(float frequency) {
  // The RFM95 frequency registers expect a 24-bit value:
  //   FRF = frequency_in_Hz / Fstep, where Fstep ≈ 61.035 Hz (for a 32 MHz crystal)
  uint32_t frf = (uint32_t)((frequency * 1e6) / 61.035);
  writeRegister(RFM95_REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(RFM95_REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(RFM95_REG_FRF_LSB, (uint8_t)(frf));
}

void RFM95C::setTxPower(int8_t power, bool useRFO) {
  // For most RFM95 modules, you use the PA_BOOST pin.
  // The lower 4 bits generally set power; here we assume power is given in dBm (2 to 17).
  // This is a simplified example.
  if (useRFO) {
    writeRegister(RFM95_REG_PA_CONFIG, 0x70 | (power & 0x0F));
  } else {
    writeRegister(RFM95_REG_PA_CONFIG, 0x80 | (power & 0x0F));
  }
}

void RFM95C::standby() {
  writeRegister(RFM95_REG_OP_MODE, RFM95_MODE_STANDBY);
}

void RFM95C::sleep() {
  writeRegister(RFM95_REG_OP_MODE, RFM95_MODE_SLEEP);
}

int8_t RFM95C::readRSSI() {
  // Read RSSI register; adjust offset as specified in datasheet (example offset: 137)
  return (int8_t)readRegister(RFM95_REG_PKT_RSSI_VALUE) - 137;
}

bool RFM95C::transmit(const uint8_t* data, uint8_t length) {
  // Put the module in standby mode to prepare for transmission.
  standby();

  // Set FIFO pointer to TX base address
  uint8_t txBase = readRegister(0x0E);  // RegFifoTxBaseAddr
  writeRegister(RFM95_REG_FIFO_ADDR_PTR, txBase);

  // Write payload into FIFO.
  digitalWrite(_csPin, LOW);
  HAL_SPI_Transmit(_spi, RFM95_REG_FIFO | 0x80, 1, 1000);

  _spi.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  _spi.transfer(RFM95_REG_FIFO | 0x80);  // Write mode

  for (uint8_t i = 0; i < length; i++) {
    SPI.transfer(data[i]);
  }

  SPI.endTransaction();
  digitalWrite(_csPin, HIGH);

  // Set payload length.
  writeRegister(0x22, length);  // RegPayloadLength

  // Enter TX mode.
  writeRegister(RFM95_REG_OP_MODE, RFM95_MODE_TX);

  // Wait for TX done (DIO0 goes high).
  unsigned long start = millis();
  while (digitalRead(_dio0Pin) == LOW) {
    if (millis() - start > 1000) {
      return false; // timeout
    }
  }

  // Clear IRQ flags.
  writeRegister(RFM95_REG_IRQ_FLAGS, 0xFF);
  standby();

  return true;
}

bool RFM95C::receive(uint8_t* buffer, uint8_t* length) {
  // Enter continuous RX mode.
  writeRegister(RFM95_REG_OP_MODE, RFM95_MODE_RX_CONTINUOUS);

  // Wait for a packet (DIO0 goes high).
  unsigned long start = millis();
  while (digitalRead(_dio0Pin) == LOW) {
    if (millis() - start > 2000) {
      return false; // timeout
    }
  }

  // Check for errors (e.g., CRC error) in the IRQ flags.
  uint8_t irqFlags = readRegister(RFM95_REG_IRQ_FLAGS);
  if (irqFlags & 0x20) {  // Example: bit 5 set indicates CRC error
    writeRegister(RFM95_REG_IRQ_FLAGS, 0xFF);
    return false;
  }

  // Get the number of received bytes.
  uint8_t receivedLength = readRegister(RFM95_REG_RX_NB_BYTES);

  // Set FIFO pointer to the current RX address.
  uint8_t currentAddr = readRegister(0x10);  // RegFifoRxCurrentAddr
  writeRegister(RFM95_REG_FIFO_ADDR_PTR, currentAddr);

  // Read the FIFO.
  digitalWrite(_csPin, LOW);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(RFM95_REG_FIFO & 0x7F);  // Read mode

  for (uint8_t i = 0; i < receivedLength; i++) {
    buffer[i] = SPI.transfer(0);
  }
  SPI.endTransaction();
  digitalWrite(_csPin, HIGH);

  *length = receivedLength;

  // Clear IRQ flags.
  writeRegister(RFM95_REG_IRQ_FLAGS, 0xFF);

  return true;
}

uint8_t RFM95C::readRegister(uint8_t reg) {
  uint8_t value;
  digitalWrite(_csPin, LOW);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg & 0x7F);  // MSB = 0 for read
  value = SPI.transfer(0);
  SPI.endTransaction();

  digitalWrite(_csPin, HIGH);
  return value;
}

void RFM95C::writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(_csPin, LOW);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg | 0x80);  // MSB = 1 for write
  SPI.transfer(value);
  SPI.endTransaction();

  digitalWrite(_csPin, HIGH);
}

void RFM95C::explicitReset() {
  digitalWrite(_resetPin, LOW);
  delay(10);
  digitalWrite(_resetPin, HIGH);
  delay(10);
}

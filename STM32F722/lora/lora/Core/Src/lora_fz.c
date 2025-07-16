/*
 * lora_fz.c
 *
 *  Created on: Jul 15, 2025
 *      Author: liufangzheng
 */

#include "lora_fz.h"

volatile bool RF95_TX_DONE_FLAG  = false;
volatile bool RF95_RX_DONE_FLAG  = false;
volatile bool RF95_CAD_DONE_FLAG = false;

// make sure you defined the LORA_SPI_HANDLE in main.h
extern SPI_HandleTypeDef LORA_SPI_HANDLE;

void LORA_CS_LOW(void) {
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
}

void LORA_CS_HIGH(void) {
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
}

void LORA_RESET_LOW(void) {
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_RESET);
}

void LORA_RESET_HIGH(void) {
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);
}

uint8_t RF95_ReadReg(uint8_t addr) {
  uint8_t data = 0;
  uint8_t read_address = addr & 0x7F;

  LORA_CS_LOW();
  HAL_SPI_Transmit(&LORA_SPI_HANDLE, &read_address, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&LORA_SPI_HANDLE, &data, 1, HAL_MAX_DELAY);
  LORA_CS_HIGH();

  return data;
}

void RF95_WriteReg(uint8_t addr, uint8_t data) {
  uint8_t write_address = addr | 0x80;

  LORA_CS_LOW();
  HAL_SPI_Transmit(&LORA_SPI_HANDLE, &write_address, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&LORA_SPI_HANDLE, &data, 1, HAL_MAX_DELAY);
  LORA_CS_HIGH();
}

void RF95_sleep(void) {
  RF95_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  HAL_Delay(5); // Allow time for the radio to enter sleep mode
}

void RF95_idle(void) {
  RF95_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void RF95_setOCP(uint8_t mA) {
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  }
  else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  RF95_WriteReg(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void RF95_set_freq(unsigned int frequency) {
  // Convert frequency to register value
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000; // 32 MHz reference frequency

  RF95_WriteReg(REG_FRF_MSB, (frf >> 16) & 0xFF);
  RF95_WriteReg(REG_FRF_MID, (frf >> 8) & 0xFF);
  RF95_WriteReg(REG_FRF_LSB, frf & 0xFF);
}

void RF95_setBW(int BW) {
  uint8_t config = RF95_ReadReg(REG_MODEM_CONFIG_1);

  switch (BW) {
    case 125: // 125 kHz
      config = (config & 0x0F) | 0x70;
      break;
    case 250: // 250 kHz
      config = (config & 0x0F) | 0x80;
      break;
    case 500: // 500 kHz
      config = (config & 0x0F) | 0x90;
      break;
  }

  RF95_WriteReg(REG_MODEM_CONFIG_1, config);
}

void RF95_setTxPower(int power, int outputPin) {
  if (outputPin == PA_OUTPUT_RFO_PIN) {
    if (power < 0) {
      power = 0;
    }
    else if (power > 14) {
      power = 14;
    }

    RF95_WriteReg(REG_PA_CONFIG, 0x70 | power);
  }
  else {
    if (power > 17) {
      if (power > 20) {
        power = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      power -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      RF95_WriteReg(REG_PA_DAC, 0x87);
      RF95_setOCP(140);
    }
    else {
      if (power < 2) {
        power = 2;
      }
      //Default value PA_HF/LF or +17dBm
      RF95_WriteReg(REG_PA_DAC, 0x84);
      RF95_setOCP(100);
    }

    RF95_WriteReg(REG_PA_CONFIG, PA_BOOST | (power - 2));
  }
}

void RF95_reset(void) {
  LORA_RESET_LOW();
  HAL_Delay(10); // Hold reset for 10 ms
  LORA_RESET_HIGH();
  HAL_Delay(10); // Allow time for the radio to reset
}

int RF95_Init(unsigned int frequency, unsigned int BW, int power) {
  RF95_reset();

  // set SPI speed
  LORA_SPI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // Adjust as needed

  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }

  uint8_t version = RF95_ReadReg(REG_VERSION);

  if (version != 0x12) {
    return 0;
  }

  RF95_sleep();
  RF95_set_freq(frequency);

  // set base addresses
  RF95_WriteReg(REG_FIFO_TX_BASE_ADDR, 0);
  RF95_WriteReg(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  RF95_WriteReg(REG_LNA, RF95_ReadReg(REG_LNA) | 0x03);

  // set auto AGC
  RF95_WriteReg(REG_MODEM_CONFIG_3, 0x04);

  // set bandwidth to 500kHz
  RF95_setBW(BW);

  // set output power to 17 dBm
  RF95_setTxPower(power, PA_OUTPUT_PA_BOOST_PIN);

  // put in standby mode
  RF95_idle();

  return 1;
}

void RF95_SetTxDoneFlag(void) {
  RF95_TX_DONE_FLAG = true;
}

void RF95_SetRxDoneFlag(void) {
  RF95_RX_DONE_FLAG = true;
}

void RF95_SetCadDoneFlag(void) {
  RF95_CAD_DONE_FLAG = true;
}

bool RF95_isTransmitting(void) {
  if ((RF95_ReadReg(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  if (RF95_ReadReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    RF95_WriteReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

void RF95_implicitHeaderMode(void) {
  RF95_WriteReg(REG_MODEM_CONFIG_1, RF95_ReadReg(REG_MODEM_CONFIG_1) | 0x01);
}

void RF95_explicitHeaderMode(void) {
  RF95_WriteReg(REG_MODEM_CONFIG_1, RF95_ReadReg(REG_MODEM_CONFIG_1) & 0xfe);
}

int RF95_beginPacket(int implicitHeader, uint8_t *data2send, int size) {
  if (RF95_isTransmitting()) {
    return 0;
  }

  // put in standby mode
  RF95_idle();

  if (implicitHeader) {
    RF95_implicitHeaderMode();
  }
  else {
    RF95_explicitHeaderMode();
  }

  RF95_WriteReg(REG_FIFO_ADDR_PTR, RF95_ReadReg(REG_FIFO_TX_BASE_ADDR));

  // Write payload data to FIFO
  for (int i = 0; i < size; i++) {
    RF95_WriteReg(REG_FIFO, data2send[i]);
  }

  RF95_WriteReg(REG_PAYLOAD_LENGTH, size);

  return 1;
}

int RF95_sendPacket(bool async) {
  if (async) {
    RF95_WriteReg(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE
  }

  // put in TX mode
  RF95_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (!async) {
    while ((RF95_ReadReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {;}
    RF95_WriteReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}

void RF95_setreceiver(int size) {
  RF95_WriteReg(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

  if (size > 0) {
    RF95_implicitHeaderMode();

    RF95_WriteReg(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    RF95_explicitHeaderMode();
  }

  RF95_WriteReg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

int RF95_get_recvbytenum(void) {
  return RF95_ReadReg(REG_RX_NB_BYTES);
}

void RF95_recv(int size, uint8_t *datarecv) {
  /*
  Set FifoPtrAddr to FifoRxCurrentAddr.
  This sets the FIFO pointer to the the location of the last packet received in the FIFO.
  The payload can then be extracted by reading the RegFifo address RegNbRxBytes times.
  */
  // set FIFO pointer to the current RX address
  RF95_WriteReg(REG_FIFO_ADDR_PTR, RF95_ReadReg(REG_FIFO_RX_CURRENT_ADDR));

  // read all bytes received
  for (int i = 0; i < size; i++) {
    datarecv[i] = RF95_ReadReg(REG_FIFO);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  UNUSED(GPIO_Pin); // Prevent unused variable warning

  if (GPIO_Pin == LORA_IRQ_Pin) {
    uint8_t irqFlags = RF95_ReadReg(REG_IRQ_FLAGS);

    if (irqFlags & IRQ_TX_DONE_MASK) {
      RF95_SetTxDoneFlag();
    }
    if (irqFlags & IRQ_RX_DONE_MASK) {
      RF95_SetRxDoneFlag();
    }
    if (irqFlags & IRQ_CAD_DONE_MASK) {
      RF95_SetCadDoneFlag();
    }

    // Clear all IRQ flags
    RF95_WriteReg(REG_IRQ_FLAGS, irqFlags);
  }
}

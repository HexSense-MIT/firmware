/*
 * ADS131M04.c
 *
 *  Created on: Jul 21, 2025
 *      Author: liufangzheng
 */

#include "ADS131M04.h"

bool ADS_data_ready = false;

// static int32_t signExtend(const uint8_t *dataBytes);
//static uint8_t spiTransmitReceiveByte(uint8_t tx);
static void spiTransmitReceiveBytes(const uint8_t txBytes[], uint8_t rxBytes[], uint8_t byteCount);
//static int32_t twoscom(int32_t value);

// make sure you defined the LORA_SPI_HANDLE in main.h
extern SPI_HandleTypeDef ADS131M04_SPI_HANDLE;
extern TIM_HandleTypeDef ADS131M04_CLKIN_TIM;

const uint16_t ADS_ZEROS_16 = 0x0000;
const uint8_t  ADS_ZEROS_8  = 0x00;

/*
static int32_t signExtend(const uint8_t dataBytes[]) {
  #ifdef ADS131M04_WORD_LENGTH_16BIT_TRUNCATED
  return ((int32_t) (((int32_t) dataBytes[0] << 24) | ((int32_t) dataBytes[1] << 16))) >> 16;
  #elif defined(ADS131M04_WORD_LENGTH_24BIT)
  return (((int32_t) (((int32_t) dataBytes[0] << 24) | ((int32_t) dataBytes[1] << 16) | ((int32_t) dataBytes[2] << 8))) >> 8) & 0x00FFFFFF;
  #elif defined(ADS131M04_WORD_LENGTH_32BIT_SIGN_EXTEND)
  return             ((int32_t) dataBytes[0] << 24) | ((int32_t) dataBytes[1] << 16) | ((int32_t) dataBytes[2] << 8) | ((int32_t) dataBytes[3] << 0);
  return ((int32_t) (((int32_t) dataBytes[0] << 24) | ((int32_t) dataBytes[1] << 16) | ((int32_t) dataBytes[2] << 8))) >> 8;
  #endif
}
*/

static void delay_us(uint32_t us) {
  uint32_t startTicks = SysTick->VAL;
  uint32_t tick = HAL_GetTick();
  uint32_t delayTicks = us * (SystemCoreClock / 1000000);
  if(startTicks < delayTicks) {
    while(HAL_GetTick() == tick);
    while((SystemCoreClock / 1000) + startTicks - delayTicks < SysTick->VAL);
  } else {
    while (HAL_GetTick() == tick && startTicks - delayTicks < SysTick->VAL);
  }
}

void ADS_RST_LOW(void) {
  HAL_GPIO_WritePin(GPIOB, ADS_SYNC_RST_Pin, GPIO_PIN_RESET);
}

void ADS_RST_HIGH(void) {
  HAL_GPIO_WritePin(GPIOB, ADS_SYNC_RST_Pin, GPIO_PIN_SET);
}

void ADS_CS_LOW(void) {
  HAL_GPIO_WritePin(GPIOC, ADS_CS_Pin, GPIO_PIN_RESET);
}

void ADS_CS_HIGH(void) {
  HAL_GPIO_WritePin(GPIOC, ADS_CS_Pin, GPIO_PIN_SET);
}

GPIO_PinState ADS_READ_DRDY(void) {
  return HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin);
}

/*
static uint8_t spiTransmitReceiveByte(uint8_t tx) {
  uint8_t rx;

  HAL_SPI_TransmitReceive(&ADS131M04_SPI_HANDLE, &tx, &rx, 1, 0xFFFF);

  return rx;
}
*/

static void spiTransmitReceiveBytes(const uint8_t txBytes[], uint8_t rxBytes[], uint8_t byteCount) {
  HAL_SPI_TransmitReceive(&ADS131M04_SPI_HANDLE, txBytes, rxBytes, byteCount, 0xFFFF);
}

void ADS131M04_Transmitword(uint16_t word, uint8_t *rxBytes) {
  uint8_t txBytes[ADS131M04_WORD_LENGTH];

  memset(txBytes, 0, ADS131M04_WORD_LENGTH*sizeof(uint8_t));
  memset(rxBytes, 0, ADS131M04_WORD_LENGTH*sizeof(uint8_t));

  txBytes[0] = (uint8_t)(word >> 8);
  txBytes[1] = (uint8_t)(word & 0xFF);

  spiTransmitReceiveBytes(txBytes, rxBytes, ADS131M04_WORD_LENGTH);
}

uint16_t ADS131M04_TransmitCommand(const uint16_t opcode) {
  uint8_t reply[ADS131M04_WORD_LENGTH]       = {0};
  uint8_t dummy_reply[ADS131M04_WORD_LENGTH] = {0};

  ADS131M04_Transmitword(opcode, reply);
  for (uint8_t i = 0; i < FRAME_LEN - 1; i++) {
    ADS131M04_Transmitword(ADS_ZEROS_16, dummy_reply); // dummy read
  }

  uint16_t response = (reply[0] << 8) | reply[1];

  return response;
}

uint16_t ADS131M04_ReadRegister(uint8_t addr) {
  uint16_t data;
  uint16_t opcode = ADS131M04_OPCODE_RREG | (addr << 7);

  ADS_CS_LOW();
  ADS131M04_TransmitCommand(opcode);
  ADS_CS_HIGH();

  delay_us(10);

  ADS_CS_LOW();
  data = ADS131M04_TransmitCommand(ADS131M04_OPCODE_NULL);
  ADS_CS_HIGH();

  return data;
}

uint16_t ADS131M04_WriteRegister(uint8_t addr, uint16_t value, bool with_reply) {
  uint16_t res;
  uint16_t cmd = 0;

  ADS_CS_LOW();
  delay_us(1);

  cmd = (CMD_WRITE_REG) | (addr << 7) | 0;
  uint8_t temp_byte[ADS131M04_WORD_LENGTH] = {0};

  ADS131M04_Transmitword(cmd, temp_byte);
  ADS131M04_Transmitword(value, temp_byte);
  ADS131M04_Transmitword(0x0000, temp_byte);
  ADS131M04_Transmitword(0x0000, temp_byte);
  ADS131M04_Transmitword(0x0000, temp_byte);
  ADS131M04_Transmitword(0x0000, temp_byte);

  ADS_CS_HIGH();
  if (with_reply) {
    delay_us(100);
    ADS_CS_LOW();
    delay_us(1);

    res = ADS131M04_TransmitCommand(CMD_NULL);

    delay_us(1);
    ADS_CS_HIGH();

    return res;
  } else {
    return 0;
  }
}

void ADS131M04_writeRegisterMasked(uint8_t address, uint16_t value, uint16_t mask) {
  uint16_t register_contents = ADS131M04_ReadRegister(address);
  register_contents = register_contents & ~mask;
  register_contents = register_contents | value;
  delay_us(10);
  ADS131M04_WriteRegister(address, register_contents, false);
}

void ADS131M04_Init(void) {
  ADS_CS_HIGH();
  /*
   * However, both slots of the FIFO are full if a sample is missed or if data are
   * not read for a period of time. Either strobe the SYNC/RESET pin to re-synchronize conversions and clear the
   * FIFOs, or quickly read two data packets when data are read for the first time or after a gap in reading data.
   * This process ensures predictable DRDY pin behavior.
   */
  ADS_RST_LOW();
  HAL_Delay(10); // Reset
  ADS_RST_HIGH();
  HAL_Delay(10); // Wait for reset to complete

  // Wait for SPI ready to make sure the ADS131M04 is ready
  while (!ADS_READ_DRDY()) {
    HAL_Delay(1);
  }

  // ADS131M04_setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N); // Set channel 0 to AIN1
  // ADS131M04_setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N); // Set channel 1 to AIN2
  // ADS131M04_setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N); // Set channel 2 to AIN3
  // ADS131M04_setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N); // Set channel 3 to AIN4

  ADS131M04_setOsr(OSR_512); // 8 kSPS

  TIM3->CCR1 = 15; // Set the PWM duty cycle to 50%

  HAL_TIM_PWM_Start(&ADS131M04_CLKIN_TIM, TIM_CHANNEL_1);
}

bool ADS131M04_isDataReady(void) {
  return HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin);
}

int8_t ADS131M04_isDataReadySoft(uint8_t channel) {
  if (channel == 0) {
    return (ADS131M04_ReadRegister(REG_STATUS) & REGMASK_STATUS_DRDY0);
  }
  else if (channel == 1) {
    return (ADS131M04_ReadRegister(REG_STATUS) & REGMASK_STATUS_DRDY1);
  }
  else if (channel == 2) {
    return (ADS131M04_ReadRegister(REG_STATUS) & REGMASK_STATUS_DRDY2);
  }
  else if (channel == 3) {
    return (ADS131M04_ReadRegister(REG_STATUS) & REGMASK_STATUS_DRDY3);
  }
  else {
    return -1;
  }
}

bool ADS131M04_isResetStatus(void) {
  return (ADS131M04_ReadRegister(REG_STATUS) & REGMASK_STATUS_RESET);
}

bool ADS131M04_isLockSPI(void) {
  return (ADS131M04_ReadRegister(REG_STATUS) & REGMASK_STATUS_LOCK);
}

bool ADS131M04_setDrdyFormat(uint8_t drdyFormat) {
  if (drdyFormat > 1) {
    return false;
  } else {
    ADS131M04_writeRegisterMasked(REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT);
    return true;
  }
}

bool ADS131M04_setDrdyStateWhenUnavailable(uint8_t drdyState) {
  if (drdyState > 1) {
    return false;
  } else {
    ADS131M04_writeRegisterMasked(REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ);
    return true;
  }
}

bool ADS131M04_setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3)
  {
    return false;
  }
  else
  {
    ADS131M04_writeRegisterMasked(REG_CLOCK, powerMode, REGMASK_CLOCK_PWR);
    return true;
  }
}

bool ADS131M04_setOsr(uint16_t osr) {
  if (osr > 7) {
    return false;
  } else {
    ADS131M04_writeRegisterMasked(REG_CLOCK, osr << 2 , REGMASK_CLOCK_OSR);
    return true;
  }
}

bool ADS131M04_setChannelEnable(uint8_t channel, uint16_t enable) {
  if (channel > 3) {
    return false;
  }

  if (channel == 0) {
    ADS131M04_writeRegisterMasked(REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN);
    return true;
  } else if (channel == 1) {
    ADS131M04_writeRegisterMasked(REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN);
    return true;
  } else if (channel == 2) {
    ADS131M04_writeRegisterMasked(REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN);
    return true;
  } else if (channel == 3) {
    ADS131M04_writeRegisterMasked(REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN);
    return true;
  }
  else {
    return false;
  }
}

bool ADS131M04_setChannelPGA(uint8_t channel, uint16_t pga) {
  if (channel > 3) {
    return false;
  }

  if (channel == 0) {
    ADS131M04_writeRegisterMasked(REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0);
    return true;
  } else if (channel == 1) {
    ADS131M04_writeRegisterMasked(REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1);
    return true;
  } else if (channel == 2) {
    ADS131M04_writeRegisterMasked(REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2);
    return true;
  } else if (channel == 3) {
    ADS131M04_writeRegisterMasked(REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3);
    return true;
  } else {
    return false;
  }
}

void ADS131M04_setGlobalChop(uint16_t global_chop)
{
  ADS131M04_writeRegisterMasked(REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN);
}

void ADS131M04_setGlobalChopDelay(uint16_t delay)
{
  ADS131M04_writeRegisterMasked(REG_CFG, delay << 9, REGMASK_CFG_GC_DLY);
}

bool ADS131M04_setInputChannelSelection(uint8_t channel, uint8_t input) {
  if (channel > 3) {
    return false;
  }

  if (channel == 0) {
    ADS131M04_writeRegisterMasked(REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  } else if (channel == 1) {
    ADS131M04_writeRegisterMasked(REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  } else if (channel == 2) {
    ADS131M04_writeRegisterMasked(REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  } else if (channel == 3) {
    ADS131M04_writeRegisterMasked(REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX);
    return true;
  } else {
    return false;
  }
}

bool ADS131M04_setChannelOffsetCalibration(uint8_t channel, int32_t offset)
{

  uint16_t MSB = offset >> 8;
  uint8_t LSB = offset;

  if (channel > 3) {
    return false;
  }

  if (channel == 0) {
    ADS131M04_writeRegisterMasked(REG_CH0_OCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  } else if (channel == 1) {
    ADS131M04_writeRegisterMasked(REG_CH1_OCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  } else if (channel == 2) {
    ADS131M04_writeRegisterMasked(REG_CH2_OCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH2_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB);
    return true;
  } else if (channel == 3) {
    ADS131M04_writeRegisterMasked(REG_CH3_OCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH3_OCAL_LSB, LSB << 8 , REGMASK_CHX_OCAL0_LSB);
    return true;
  } else {
    return false;
  }
}

bool ADS131M04_setChannelGainCalibration(uint8_t channel, uint32_t gain) {
  uint16_t MSB = gain >> 8;
  uint8_t LSB = gain;

  if (channel > 3) {
    return false;
  }

  if (channel == 0) {
    ADS131M04_writeRegisterMasked(REG_CH0_GCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  } else if (channel == 1) {
    ADS131M04_writeRegisterMasked(REG_CH1_GCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  } else if (channel == 2) {
    ADS131M04_writeRegisterMasked(REG_CH2_GCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH2_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  } else if (channel == 3) {
    ADS131M04_writeRegisterMasked(REG_CH3_GCAL_MSB, MSB, 0xFFFF);
    ADS131M04_writeRegisterMasked(REG_CH3_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB);
    return true;
  } else {
    return false;
  }
}

bool ADS131M04_readADC(ADS131M04_ADCValue *adcValue) {
  uint8_t rxBytes[ADS131M04_WORD_LENGTH * FRAME_LEN] = {0};
  uint8_t txBytes[ADS131M04_WORD_LENGTH * FRAME_LEN] = {0};

  memset(txBytes, 0, ADS131M04_WORD_LENGTH * FRAME_LEN * sizeof(uint8_t)); // NULL Command

  for (uint8_t i = 0; i < FRAME_LEN; i++) {
    spiTransmitReceiveBytes(txBytes + (i * ADS131M04_WORD_LENGTH), rxBytes + (i * ADS131M04_WORD_LENGTH), ADS131M04_WORD_LENGTH);
  }

  adcValue->status = (rxBytes[0] << 8) | rxBytes[1]; // STATUS Word

  return true;
}

// bool ADS131M04_isDataReady() {
//   if (digitalRead(ADS131M04_DRDY_PIN) == HIGH)
//   {
//     return false;
//   }
//   return true;
// }

/*
bool ADS131M04_ReadADC(ADS131M04_ADCValue *adcValue) {
  uint8_t rxBytes[ADS131M04_WORD_LENGTH];
  uint8_t txBytes[ADS131M04_WORD_LENGTH];

  memset(txBytes, 0, ADS131M04_WORD_LENGTH*sizeof(uint8_t)); // NULL Command

  for(uint8_t i = 0; i < ADS131M04_WORD_LENGTH; i++) {
      rxBytes[i] = spiTransmitReceiveByte(txBytes[i]);
  }

  adcValue->status = (rxBytes[0] << 8) | rxBytes[1]; // STATUS Word
  uint16_t crcWord = calcCRC(rxBytes, ADS131M04_WORD_LENGTH); // Calculate CRC for STATUS Word

  #ifdef ADS131M04_ENABLE_CRC
  uint16_t crcWordIn = calcCRC(txBytes, 2 * ADS131M04_WORD_LENGTH); // NULL Command CRC Word
  txBytes[0] = (uint8_t)((crcWordIn >> 8) & 0xFF);
  txBytes[1] = (uint8_t)(crcWordIn & 0xFF);
  #endif

  for(uint8_t i = 0; i < ADS131M04_WORD_LENGTH; i++) {
    rxBytes[i] = spiTransmitReceiveByte(txBytes[i]);
  }

  adcValue->channel[0] = twoscom(signExtend(rxBytes)); // Channel 0 Data
  crcWord = calcCRCWithInitVal(rxBytes, ADS131M04_WORD_LENGTH, crcWord);

  #ifdef ADS131M04_ENABLE_CRC
  memset(txBytes, 0, ADS131M04_WORD_LENGTH*sizeof(uint8_t));
  #endif

  for(uint8_t i = 0; i < 3; i++) {
    for(uint8_t j = 0; j < ADS131M04_WORD_LENGTH; j++) {
      rxBytes[j] = spiTransmitReceiveByte(txBytes[j]);
    }
    adcValue->channel[i + 1] = twoscom(signExtend(rxBytes)); // Channel 1-3 Data
    crcWord = calcCRCWithInitVal(rxBytes, ADS131M04_WORD_LENGTH, crcWord);
  }

  for(uint8_t i = 0; i < ADS131M04_WORD_LENGTH; i++) { // Read CRC Word
    rxBytes[i] = spiTransmitReceiveByte(txBytes[i]);
  }

  uint16_t crcWordReceived = (rxBytes[0] << 8) | rxBytes[1]; // CRC Word

  return crcWord == crcWordReceived; // Verify CRC
}
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  UNUSED(GPIO_Pin); // Prevent unused variable warning

  if (GPIO_Pin == ADS_DRDY_Pin) {
    ADS_data_ready = true;
  }
}

/*
static int32_t twoscom(int32_t value) {
  if (value > 0x7FFFFF) {
    return ((~(value) & 0x00FFFFFF) + 1) * -1;
  }
  return value;
}
*/

/*
 * DW3000_FZ.c
 *
 *  Created on: Jul 16, 2025
 *      Author: liufangzheng
 */

#include "DW3000_FZ.h"

bool DW3000_IRQ_flag = false; // flag to indicate if the TX is done

// remember put this in the main.h: #define LORA_SPI_HANDLE hspi1
extern SPI_HandleTypeDef UWB_SPI_HANDLE;

unsigned int countBits(unsigned int number) {
  unsigned int count = 0;
  while (number) {
    count++;
    number >>= 1; // Right shift the number by 1 bit
  }
  return count;
}

HAL_StatusTypeDef sendBytes(uint8_t *sendb, uint16_t sendLen) {
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit(&UWB_SPI_HANDLE, sendb, sendLen, 1);

  return status;
}

HAL_StatusTypeDef readBytes(uint8_t *recvb, uint16_t recLen) {
  HAL_StatusTypeDef status = HAL_OK;

  HAL_SPI_Receive(&UWB_SPI_HANDLE, recvb, recLen, 1);

  return status;
}

uint8_t DW3000pack_fast_command(uint8_t cmd) {
  uint8_t b = 0x81;
  b |= ((cmd & 0x1F) << 1);

  return b;
}

// uint8_t address: 5-bit short address
// uint8_t rw: 1-bit read/write
uint8_t DW3000pack_short_address(uint8_t address, uint8_t rw) {
  uint8_t b = 0x00;
  b |= (rw << 7);
  b |= ((address & 0x1F) << 1);

  return b;
}

uint16_t DW3000pack_full_address(uint8_t base, uint8_t sub, uint8_t rw) {
  uint16_t header = 0x4000;

  header |= (rw << 15);
  header |= ((base & 0x1F) << 9);
  header |= ((sub & 0x7F) << 2);

  return header;
}

void DW3000pack_mask_cmd_1octet(uint32_t reg, uint8_t andmask, uint8_t ormask, uint8_t* cmd) {
  uint8_t base = reg >> 16;
  uint8_t sub  = reg & 0xFF;

  uint16_t header = 0xC001;

  header |= ((base & 0x1F) << 9);
  header |= ((sub & 0x7F) << 2);

  cmd[0] = header >> 8;
  cmd[1] = header & 0xFF;
  cmd[2] = andmask;
  cmd[3] = ormask;
}

void DW3000pack_mask_cmd_2octet(uint32_t reg, uint16_t andmask, uint16_t ormask, uint8_t* cmd) {
  uint8_t base = reg >> 16;
  uint8_t sub  = reg & 0xFF;

  uint16_t header = 0xC002;

  header |= ((base & 0x1F) << 9);
  header |= ((sub & 0x7F) << 2);

  cmd[0] = header >> 8;
  cmd[1] = header & 0xFF;
  cmd[2] = andmask & 0xFF;
  cmd[3] = andmask >> 8;
  cmd[4] = ormask & 0xFF;
  cmd[5] = ormask >> 8;
}

void DW3000pack_mask_cmd_4octet(uint32_t reg, uint32_t andmask, uint32_t ormask, uint8_t* cmd) {
  uint8_t base = reg >> 16;
  uint8_t sub  = reg & 0xFF;

  uint16_t header = 0xC003;

  header |= ((base & 0x1F) << 9);
  header |= ((sub & 0x7F) << 2);

  cmd[0] = header >> 8;
  cmd[1] = header & 0xFF;
  cmd[2] = andmask & 0xFF;
  cmd[3] = andmask >> 8;
  cmd[4] = andmask >> 16;
  cmd[5] = andmask >> 24;
  cmd[6] = ormask & 0xFF;
  cmd[7] = ormask >> 8;
  cmd[8] = ormask >> 16;
  cmd[9] = ormask >> 24;
}

void set_bits(uint32_t reg, uint32_t mask, uint8_t reg_width) {
  // longest command is 10 bytes, so allocate 10 bytes
  uint8_t cmd[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t andmask = 0xFFFFFFFFUL;
  uint32_t ormask  = mask;

  if (reg_width == 1)      DW3000pack_mask_cmd_1octet(reg, andmask, ormask, cmd);
  else if (reg_width == 2) DW3000pack_mask_cmd_2octet(reg, andmask, ormask, cmd);
  else if (reg_width == 4) DW3000pack_mask_cmd_4octet(reg, andmask, ormask, cmd);

  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_RESET);
  sendBytes(cmd, 10);
  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_SET);
}

void reset_bits(uint32_t reg, uint32_t mask, uint8_t reg_width) {
  // longest command is 10 bytes, so allocate 10 bytes
  uint8_t cmd[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint32_t andmask = ~mask; // reset bits
  uint32_t ormask  = 0x0UL; // keep the same

  if (reg_width == 1)      DW3000pack_mask_cmd_1octet(reg, andmask, ormask, cmd);
  else if (reg_width == 2) DW3000pack_mask_cmd_2octet(reg, andmask, ormask, cmd);
  else if (reg_width == 4) DW3000pack_mask_cmd_4octet(reg, andmask, ormask, cmd);

  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_RESET);
  sendBytes(cmd, 10);
  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Set the SPI speed to low speed (5 MHz)
 * 
 * @param hspi 
 */
void set_SPI2lowspeed(SPI_HandleTypeDef *hspi) {
  // set SPI speed to 3 MHz
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

  if (HAL_SPI_Init(hspi) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief set the SPI speed to high speed (20 MHz)
 * 
 * @param hspi 
 */
void set_SPI2highspeed(SPI_HandleTypeDef *hspi) {
  // set SPI speed to 24 MHz
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  if (HAL_SPI_Init(hspi) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Enable on the DW3000 3.3V LDO
 * 
 */
void DW3000poweron(void) {
  HAL_GPIO_WritePin(UWB_PWR_EN_GPIO_Port, UWB_PWR_EN_Pin, GPIO_PIN_SET);
}

/**
 * @brief Set the DW3000 SPI to low speed (5MHz)
 * 
 * @param hspi 
 */
void DW3000init(SPI_HandleTypeDef *hspi) {
  set_SPI2lowspeed(hspi);
}

/**
 * @brief Reset the DW3000.
 * The RSTn pin can also be used to reset the device.
 * Host microprocessor can use this pin to reset the device instead of calling dwt_softreset() function.
 * The pin should be driven low (for 10 ns) and then left in open-drain mode.
 * RSTn pin should never be driven high.
 * 
 */
void DW3000hardReset(void) {
  HAL_GPIO_WritePin(UWB_RST_GPIO_Port, UWB_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(UWB_RST_GPIO_Port, UWB_RST_Pin, GPIO_PIN_SET);
}

/**
 * @brief Write data to a register on the DW3000
 * 
 * @param reg register full address (base + sub)
 * @param data data to write
 * @param len length of data to write (1, 2, 4 bytes)
 */
void DW3000writereg(uint32_t reg, uint8_t* data, uint8_t len) {
  uint8_t base = reg >> 16;
  uint8_t sub  = reg & 0xFF;

  uint16_t header = DW3000pack_full_address(base, sub, 1);

  // uint8_t* regBytes = data;
  uint8_t headerBytes[2];
  headerBytes[0] = (header >> 8) & 0xFF;
  headerBytes[1] = header & 0xFF;

  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_RESET);
  sendBytes(headerBytes, 2);
  sendBytes(data, len);
  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Read data from a register on the DW3000
 * 
 * @param reg register full address (base + sub)
 * @param len length of data to read (1, 2, 4 bytes)
 * @return uint32_t data read from the register
 */
uint32_t DW3000readreg(uint32_t reg, uint8_t len) {
  uint8_t base = reg >> 16;
  uint8_t sub  = reg & 0xFF;

  uint16_t header = DW3000pack_full_address(base, sub, 0);

  uint8_t regBytes[4] = {0, 0, 0, 0};
  uint8_t headerBytes[2];
  headerBytes[0] = (header >> 8) & 0xFF;
  headerBytes[1] = header & 0xFF;

  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_RESET);
  sendBytes(headerBytes, 2);
  readBytes(regBytes, len);
  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_SET);

  // Combine the 4 bytes into a single 32-bit integer
  uint32_t regValue =
  ((uint32_t)regBytes[3] << 24) |
  ((uint32_t)regBytes[2] << 16) |
  ((uint32_t)regBytes[1] << 8)  |
  ((uint32_t)regBytes[0]);

  return regValue;
}

/**
 * @brief set the AINIT2IDLE bit in the SEQ_CTRL register to automatically enter the IDLE_PLL state
 * 
 */
void DW3000enter_IDLE_PLL(void) {
  set_bits(SEQ_CTRL_ID, SEQ_CTRL_AINIT2IDLE_BIT_MASK, 4); // set the AINIT2IDLE bit
  Delay_us(10);
}

/**
 * @brief FZ stole from DecaWave API, check if the DW3000 is in the IDLE_RC state
 * 
 * @return uint8_t 
 */
uint8_t DW3000check_IDLE_RC(void) {
  uint32_t reg = DW3000readreg(SYS_STATUS_ID, 4);
  return ((reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK)) ? 1U : 0U;
}

/**
 * @brief check if the DW3000 PLL locked, which shows the DW3000 is in the IDLE_PLL state
 * 
 * @return uint8_t 
 */
uint8_t DW3000check_IDLE_PLL(void) {
  uint32_t reg = DW3000readreg(SYS_STATUS_ID, 4);
  return ((reg & (SYS_STATUS_CP_LOCK_BIT_MASK)) == (SYS_STATUS_CP_LOCK_BIT_MASK)) ? 1U : 0U;
}

/**
 * @brief check if the DW3000 is in the IDLE_PLL state
 * 
 * @return uint8_t 
 */
uint8_t DW3000check_IDLE(void) {
  uint32_t reg = DW3000readreg(SYS_STATE_LO_ID, 4);
  return (reg >> 16 & DW_SYS_STATE_IDLE) == DW_SYS_STATE_IDLE ? 1 : 0;
}

void enable_LED_blink(void) {
  uint32_t current_ctrl = DW3000readreg(LED_CTRL_ID, 4);
  current_ctrl |= (1 << LED_CTRL_BLINK_EN_BIT_OFFSET);
  DW3000writereg(LED_CTRL_ID, (uint8_t*)&current_ctrl, 4);
}

/**
 * @brief Set the DW3000 TXLED pin
 * 
 */
void DW3000set_TXLED(void) {
  enable_LED_blink();

  uint32_t current_mode;

  current_mode = DW3000readreg(GPIO_MODE_ID, 4);
  current_mode &= ~GPIO_MODE_MSGP3_MODE_BIT_MASK; // Clear the bits
  current_mode |= (0x01 << GPIO_MODE_MSGP3_MODE_BIT_OFFSET); // Set the TXLED mode bit

  DW3000writereg(GPIO_MODE_ID, (uint8_t*)&current_mode, 4);
}

/**
 * @brief read DW3000 OTP memory
 * 
 * @param addr OTP address
 * @return uint32_t OTP data
 */
uint32_t DW3000readOTP(uint8_t addr) {
  uint8_t address = addr;
  uint8_t const_02 = 0x02;
  DW3000writereg(OTP_ADDR_ID, &address, 2);
  DW3000writereg(OTP_CFG_ID, &const_02, 2);

  uint32_t reg = DW3000readreg(OTP_RDATA_ID, 4);

  return reg;
}

/**
 * @brief write a fast command to the DW3000
 * 
 * @param cmd 
 */
void DW3000_writefastCMD_FZ(uint8_t cmd) {
  uint8_t cmd2send = DW3000pack_fast_command(cmd);

  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_RESET);
  sendBytes(&cmd2send, 1);
  HAL_GPIO_WritePin(UWB_CS_GPIO_Port, UWB_CS_Pin, GPIO_PIN_SET);
}

// clear the IRQ flags
void DW3000_clear_IRQ(void) {
  uint32_t irqFlags = DW3000readreg(SYS_STATUS_ID, 4);
    // FZ: Clear the interrupt flags
  DW3000writereg(SYS_STATUS_ID, (uint8_t*)&irqFlags, 4);
  DW3000_IRQ_flag = false;
}

void DW3000_clear_all_events(void) {
  DW3000_writefastCMD_FZ(CMD_CLR_IRQS);
}

void DW3000config_CH(uint16_t RX_PCODE, uint16_t TX_PCODE, uint8_t SFD_TYP, channel CH) {
  uint32_t chan_ctrl = 0;

  chan_ctrl |= ((RX_PCODE << CHAN_CTRL_RX_PCODE_BIT_OFFSET) & CHAN_CTRL_RX_PCODE_BIT_MASK) |
               ((TX_PCODE << CHAN_CTRL_TX_PCODE_BIT_OFFSET) & CHAN_CTRL_TX_PCODE_BIT_MASK) |
               ((SFD_TYP  << CHAN_CTRL_SFD_TYPE_BIT_OFFSET) & CHAN_CTRL_SFD_TYPE_BIT_MASK) |
               ((CH       << CHAN_CTRL_RF_CHAN_BIT_OFFSET) & CHAN_CTRL_RF_CHAN_BIT_MASK);

  // FZ: write the channel control register
  DW3000writereg(CHAN_CTRL_ID, (uint8_t*)&chan_ctrl, 4);
}

void DW3000_irq_for_tx_done(void) {
  uint32_t sys_enable = DW3000readreg(SYS_ENABLE_LO_ID, 4);
  sys_enable |= (1 << SYS_ENABLE_LO_TXFRS_ENABLE_BIT_OFFSET); // Enable TX done interrupt
  DW3000writereg(SYS_ENABLE_LO_ID, (uint8_t*)&sys_enable, SYS_ENABLE_LO_LEN);
}

void DW3000_irq_for_rx_done(void) {
  // uint32_t sys_enable = DW3000readreg(SYS_ENABLE_LO_ID, 4);
  // sys_enable |= ((1 << SYS_ENABLE_LO_RXFR_ENABLE_BIT_OFFSET)  |
  //                (1 << SYS_ENABLE_LO_RXPHD_ENABLE_BIT_OFFSET)); // Enable RX done interrupt
  uint32_t sys_enable = SYS_ENABLE_LO_MASK;
  DW3000writereg(SYS_ENABLE_LO_ID, (uint8_t*)&sys_enable, SYS_ENABLE_LO_LEN);
}

void DW3000_disable_RX_timeout(void) {
  ;
}

// FZ: the IRQ will be triggered at the very beginning because
// the SPI ready will cause an interrupt
// so we need to clear the interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  UNUSED(GPIO_Pin); // Prevent unused variable warning

  if (GPIO_Pin == UWB_IRQ_Pin) {
    DW3000_IRQ_flag = true;
  }
}


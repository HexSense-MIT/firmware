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
  hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;

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

void DW3000_set_max_sfd_timeout(void) {
  // FZ: Set the maximum SFD timeout
  uint16_t sfd_timeout = RX_SFD_TOC_MASK;
  DW3000writereg(RX_SFD_TOC_ID, (uint8_t*)&sfd_timeout, 2);
}

void DW3000_debug_reg(uint32_t reg, uint8_t len) {
  uint32_t reg_value = DW3000readreg(reg, len);
  printf("DW3000 Register 0x%08lX: 0x%08lX\r\n", reg, reg_value);
}

/**
 * @brief FZ: from: https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
 * Things which are actually documented but easy to miss
 * - If using a 16MHz PRF (PCODE 3 or 4), set RX_TUNE_EN in DGC_CFG
 * - Always change THR_64 in DGC_CFG to 0x32
 * - Always clear DT0B4 in DTUNE0
 * - Always change COMP_DLY in RX_CAL to 0x2
 * - Always change LDO_RLOAD to 0x14
 * - Always change RF_TX_CTRL_1 to 0x0E
 * - Always change RF_TX_CTRL_2 to 0x1C071134 (ch5) or 0x1C010034 (ch9)
 * - Always change PLL_CFG to 0x1F3C (ch5) or 0x0F3C (ch9)
 * - Always change PLL_CFG_LD in PLL_CAL to 0x8 (documented as 0x81 but that's the whole register)
 */
void DW3000_cfg_FZ(void) {
  // Always change THR_64 in DGC_CFG to 0x32
  uint32_t dgc_cfg = DW3000readreg(DGC_CFG_ID, 4);
  dgc_cfg |= (1 << DGC_CFG_RX_TUNE_EN_BIT_OFFSET) |
             (0x32 << DGC_CFG_THR_64_BIT_OFFSET); // Enable RX tuning
  DW3000writereg(DGC_CFG_ID, (uint8_t*)&dgc_cfg, 4);

  // Always clear DT0B4 in DTUNE0
  uint32_t dtune0 = DW3000readreg(DTUNE0_ID, 4);
  dtune0 &= ~DTUNE0_DT0B4_BIT_MASK; // Clear the DT0B4 bit
  DW3000writereg(DTUNE0_ID, (uint8_t*)&dtune0, 4);

  // Always change COMP_DLY in RX_CAL to 0x2
  uint32_t rx_cal = DW3000readreg(RX_CAL_CFG_ID, 4);
  rx_cal &= ~RX_CAL_CFG_COMP_DLY_BIT_MASK; // Clear the COMP_DLY
  rx_cal |= (0x2 << RX_CAL_CFG_COMP_DLY_BIT_OFFSET); // Set COMP_DLY to 0x2
  DW3000writereg(RX_CAL_CFG_ID, (uint8_t*)&rx_cal, 4);

  // Always change LDO_RLOAD to 0x14
  uint8_t ldo_cfg = 0x14; // LDO_RLOAD value
  DW3000writereg(LDO_RLOAD_ID, &ldo_cfg, 1);

  // Always change RF_TX_CTRL_1 to 0x0E
  uint8_t rf_tx_ctrl_1 = 0x0E;
  DW3000writereg(RF_TX_CTRL_1_ID, &rf_tx_ctrl_1, 1);

  // Always change RF_TX_CTRL_2 to 0x1C071134 (ch5) or 0x1C010034 (ch9)
  uint32_t channel_ctrl = DW3000readreg(CHAN_CTRL_ID, 4);
  uint32_t channel_sel  = channel_ctrl & CHAN_CTRL_RF_CHAN_BIT_MASK; // Get the current channel
  uint32_t rf_tx_ctrl_2 = (channel_sel == 0) ? 0x1C071134 : 0x1C010034; // Set RF_TX_CTRL_2 based on channel
  DW3000writereg(TX_CTRL_HI_ID, (uint8_t*)&rf_tx_ctrl_2, 4);

  // Always change PLL_CFG to 0x1F3C (ch5) or 0x0F3C (ch9)
  uint16_t pll_cfg = (channel_sel == 0) ? 0x1F3C : 0x0F3C; // Set PLL_CFG based on channel
  DW3000writereg(PLL_CFG_ID, (uint8_t*)&pll_cfg, 2);

  // Always change PLL_CFG_LD in PLL_CAL to 0x8
  uint32_t pll_cal = DW3000readreg(PLL_CAL_ID, 4);
  pll_cal &= ~PLL_CAL_PLL_PLL_CFG_LD_MASK; // Clear the PLL_CFG_LD bits
  pll_cal |= (0x8 << PLL_CAL_PLL_PLL_CFG_LD_OFFSET); // Set PLL_CFG_LD to 0x8
  DW3000writereg(PLL_CAL_ID, (uint8_t*)&pll_cal, 4);
}

/**
 * @brief from: https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
 * Receiver calibration (aka "PGF calibration") must run successfully at startup
 * (and after wakeup or 20°C temperature change) for decent performance.
 * The manual describes how to start calibration with RX_CAL and check results
 * in RX_CAL_RESI and RX_CAL_RESQ but misses some details:
 * - Before running calibration, bits 0 (VDDMS1), 2 (VDDMS3), and 8 (VDDIF2) must be set in LDO_CTRL
 * - Before reading RX_CAL_RESI/RESQ, bit 16 in RX_CAL_CFG (the low bit of COMP_DLY) must be set
 * - (After calibration, the previous value of LDO_CTRL should be restored to save power.)
 * - Without these steps, calibration will fail (missing LDOs) and the failure won't be noticed
 *   (result values not being read properly), but the radio will perform very badly.
 */
void DW3000_pgf_cal(void) {
  // pre calibration steps
  uint32_t ldo_ctrl = DW3000readreg(LDO_CTRL_ID, 4);
  uint32_t ldo_ctrl_restore = ldo_ctrl; // Save the original LDO_CTRL value

  // Set VDDMS1, VDDMS3, and VDDIF2 bits in LDO_CTRL
  ldo_ctrl |= (1 << LDO_CTRL_LDO_VDDMS1_EN_BIT_OFFSET) |
              (1 << LDO_CTRL_LDO_VDDMS3_EN_BIT_OFFSET) |
              (1 << LDO_CTRL_LDO_VDDIF2_EN_BIT_OFFSET);
  DW3000writereg(LDO_CTRL_ID, (uint8_t*)&ldo_ctrl, 4);

  // calibration steps
  uint32_t rx_cal_cfg = DW3000readreg(RX_CAL_CFG_ID, 4);
  rx_cal_cfg &= ~RX_CAL_CFG_CAL_MODE_BIT_MASK; // Set the CAL_MODE to 0
  rx_cal_cfg |= (0x01 << RX_CAL_CFG_CAL_MODE_BIT_OFFSET); // set CAL_MODE to calibration mode
  DW3000writereg(RX_CAL_CFG_ID, (uint8_t*)&rx_cal_cfg, 4);
  // Start the calibration
  rx_cal_cfg = DW3000readreg(RX_CAL_CFG_ID, 4);
  rx_cal_cfg |= (0x01 << RX_CAL_CFG_CAL_EN_BIT_OFFSET);
  DW3000writereg(RX_CAL_CFG_ID, (uint8_t*)&rx_cal_cfg, 4);

  // Wait for calibration to complete
  while (!(DW3000readreg(RX_CAL_STS_ID, 1) & 0x01)) {
    HAL_Delay(1); // Delay to avoid busy-waiting
  }

  // back to normal mode
  rx_cal_cfg = DW3000readreg(RX_CAL_CFG_ID, 4);
  rx_cal_cfg &= ~RX_CAL_CFG_CAL_MODE_BIT_MASK; // Set the CAL_MODE to 0
  DW3000writereg(RX_CAL_CFG_ID, (uint8_t*)&rx_cal_cfg, 4);

  // LDO back to restored value to save power
  DW3000writereg(LDO_CTRL_ID, (uint8_t*)&ldo_ctrl_restore, 4);
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


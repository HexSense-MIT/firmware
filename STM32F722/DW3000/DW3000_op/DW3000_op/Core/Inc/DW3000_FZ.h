/*
 * DW3000_FZ.h
 *
 *  Created on: Jul 16, 2025
 *      Author: liufangzheng
 */

#ifndef INC_DW3000_FZ_H_
#define INC_DW3000_FZ_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "stm32f7xx_hal.h"
#include "main.h"

#include "dw3000_deca_regs.h"
#include "dw3000_deca_vals.h"
#include "HS_system.h"

extern bool DW3000_IRQ_flag; // flag to indicate if the TX is done

typedef enum {
  tx_node = 0, //!< TX node
  rx_node = 1  //!< RX node
} node_type;

// FZ stole from DecaWave API
typedef enum {
  DWT_DW3000_DEV_ID       = (int)(0xDECA0302), //!< DW3000 (non PDOA) silicon device ID
  DWT_QM33110_DEV_ID      = (int)(0xDECA0304), //!< QM33110 (non PDOA) silicon device ID
  DWT_DW3000_PDOA_DEV_ID  = (int)(0xDECA0312), //!< DW3000 (with PDOA) silicon device ID
  DWT_QM33120_PDOA_DEV_ID = (int)(0xDECA0314) //!< QM33120 (with PDOA) silicon device ID
} dw_chip_id_e;

typedef enum {
  GPIO0 = 0x1,
  GPIO1 = 0x1 << 1,
  GPIO2 = 0x1 << 2,
  GPIO3 = 0x1 << 3,
  GPIO4 = 0x1 << 4,
  GPIO5 = 0x1 << 5,
  GPIO6 = 0x1 << 6,
  GPIO7 = 0x1 << 7,
  GPIO8 = 0x1 << 8
} dw_gpio;

typedef enum {
  OUTPUT_MODE = 0, //!< GPIO output mode
  INPUT_MODE  = 1  //!< GPIO input mode
} dw_gpio_mode;

typedef enum {
  CH5 = 0, //!< GPIO output mode
  CH9  = 1  //!< GPIO input mode
} channel;

#define GEN_CFG_AES_base 0x00
#define DIG_DIAG_base    0x0F

unsigned int countBits(unsigned int number);

HAL_StatusTypeDef sendBytes(uint8_t *sendb, uint16_t sendLen);
HAL_StatusTypeDef readBytes(uint8_t *recvb, uint16_t recLen);

uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t readWriteBit);

uint32_t read(int base, int sub);

extern void set_SPI2lowspeed(SPI_HandleTypeDef *hspi);
extern void set_SPI2highspeed(SPI_HandleTypeDef *hspi);

extern uint8_t DW3000pack_fast_command(uint8_t cmd);

extern uint8_t DW3000pack_short_address(uint8_t address, uint8_t rw);
extern uint16_t DW3000pack_full_address(uint8_t base, uint8_t sub, uint8_t rw);

extern void DW3000poweron(void);
extern void DW3000init(SPI_HandleTypeDef *hspi);
extern void DW3000hardReset(void);

extern void     DW3000writereg(uint32_t reg, uint8_t* data, uint8_t len);
extern uint32_t DW3000readreg(uint32_t reg, uint8_t len);

extern void DW3000enter_IDLE_PLL(void);

extern uint8_t DW3000check_IDLE_RC(void);
extern uint8_t DW3000check_IDLE_PLL(void);
extern uint8_t DW3000check_IDLE(void);

extern void DW3000config_CH(uint16_t RX_PCODE, uint16_t TX_PCODE, uint8_t SFD_TYP, channel CH);
extern void DW3000set_TXLED(void);

extern uint32_t DW3000readOTP(uint8_t addr);

extern void DW3000_clear_IRQ(void);
extern void DW3000_clear_all_events(void);

extern void DW3000_writefastCMD_FZ(uint8_t cmd);
extern void DW3000_irq_for_tx_done(void);
extern void DW3000_irq_for_rx_done(void);

extern void DW3000_disable_RX_timeout(void);

#endif /* INC_DW3000_FZ_H_ */

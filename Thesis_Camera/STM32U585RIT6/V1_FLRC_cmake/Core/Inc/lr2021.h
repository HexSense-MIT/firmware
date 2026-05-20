#ifndef LR2021_H
#define LR2021_H

#include "stm32u5xx_hal.h"
#include <stdint.h>

/*
 * LR2021 dual-mode radio driver (LoRa + FLRC).
 * Reference: github.com/Lora-net/usp  (smtc_rac_lib/radio_drivers/lr20xx_driver)
 *
 * Usage:
 *   LR2021_Init()              — reset + standby
 *   LR2021_SetMode(LORA)       — configure for commands / ACKs
 *   LR2021_LoRa_Send()
 *   LR2021_LoRa_Receive()
 *   LR2021_SetMode(FLRC)       — configure for bulk data
 *   LR2021_FLRC_Send()
 *
 * SPI2 pin map (configured in MX_SPI2_Init):
 *   SCK  = PB10   MOSI = PC1   MISO = PC2
 *
 * Extra GPIO — ADJUST to match your PCB schematic:
 */
#define LR2021_CS_PORT      GPIOC
#define LR2021_CS_PIN       GPIO_PIN_7

#define LR2021_BUSY_PORT    GPIOB
#define LR2021_BUSY_PIN     GPIO_PIN_2

#define LR2021_RESET_PORT   GPIOB
#define LR2021_RESET_PIN    GPIO_PIN_7

#define LR2021_IRQ_PORT     GPIOC          /* DIO5 — active-high IRQ */
#define LR2021_IRQ_PIN      GPIO_PIN_6

/* RF centre frequency in Hz — same for both modes */
#define LR2021_FREQ_HZ          915000000UL

/* ------------------------------------------------------------------ */
/* LR20xx 16-bit opcodes                                               */
/* ------------------------------------------------------------------ */

/* System */
#define LR2021_CMD_SET_STANDBY          0x0128U
#define LR2021_STANDBY_RC               0x00U
#define LR2021_STANDBY_XOSC             0x01U
/* Radio common */
#define LR2021_CMD_SET_RF_FREQ          0x0200U  /* 4-byte Hz value */
#define LR2021_CMD_SET_TX_PARAMS        0x0203U  /* power(half-dBm), ramp */
#define LR2021_CMD_SET_PACKET_TYPE      0x0207U
#define LR2021_CMD_SET_RX               0x020CU  /* 3-byte RTC timeout */
#define LR2021_CMD_SET_TX               0x020DU  /* 3-byte RTC timeout */
#define LR2021_CMD_GET_RX_PKT_LEN       0x0212U  /* returns 2 bytes */
/* FIFO */
#define LR2021_CMD_READ_FIFO            0x0001U
#define LR2021_CMD_WRITE_FIFO           0x0002U
/* IRQ */
#define LR2021_CMD_CFG_IRQ              0x011AU  /* 10 param bytes */
#define LR2021_CMD_GET_AND_CLR_IRQ      0x012EU  /* returns 4 bytes */
/* LoRa specific */
#define LR2021_CMD_LORA_SET_MOD_PARAMS  0x0220U  /* 2 param bytes */
#define LR2021_CMD_LORA_SET_PKT_PARAMS  0x0221U  /* 4 param bytes */
/* FLRC specific */
#define LR2021_CMD_FLRC_SET_MOD_PARAMS  0x0248U  /* 2 param bytes */
#define LR2021_CMD_FLRC_SET_PKT_PARAMS  0x0249U  /* 4 param bytes */

/* ------------------------------------------------------------------ */
/* Packet types                                                         */
/* ------------------------------------------------------------------ */
#define LR2021_PKT_TYPE_LORA            0x00U
#define LR2021_PKT_TYPE_FLRC            0x05U

/* ------------------------------------------------------------------ */
/* LoRa modulation                                                      */
/* Mod byte 2: (SF<<4)|BW    Mod byte 3: (CR<<4)|PPM                  */
/* ------------------------------------------------------------------ */
/* Spreading factor */
#define LR2021_LORA_SF7                 0x07U
#define LR2021_LORA_SF9                 0x09U
/* Bandwidth */
#define LR2021_LORA_BW_500              0x06U   /* 500 kHz */
#define LR2021_LORA_BW_250              0x05U   /* 250 kHz */
/* Coding rate */
#define LR2021_LORA_CR_4_5             0x01U
#define LR2021_LORA_CR_4_6             0x02U
/* Low data-rate optimise */
#define LR2021_LORA_PPM_OFF             0x00U

/* LoRa packet params (pkt_mode<<2)|(crc<<1)|(iq) */
#define LR2021_LORA_PKT_EXPLICIT        0x00U   /* variable-length header */
#define LR2021_LORA_CRC_ON              0x01U
#define LR2021_LORA_IQ_STANDARD         0x00U
#define LR2021_LORA_PREAMBLE_SYMBOLS    8U      /* 16-bit field */

/* ------------------------------------------------------------------ */
/* FLRC modulation                                                    */
/* Mod byte 2: BR_BW    Mod byte 3: (CR<<4)|pulse_shape               */
/* ------------------------------------------------------------------ */
#define LR2021_FLRC_BR_BW_2_6M_2_6M     0x00U   /* BR = 2600kbps and BW = 2666kHz */
#define LR2021_FLRC_BR_BW_2M_2_2M       0x01U   /* BR = 2080kbps and BW = 2222kHz */
#define LR2021_FLRC_BR_BW_1_3M_1_3M     0x02U   /* BR = 1300kbps and BW = 1333kHz */
#define LR2021_FLRC_BR_BW_1M_1_3M       0x03U   /* BR = 1040kbps and BW = 1333kHz */
#define LR2021_FLRC_BR_BW_650k_0_9M     0x04U   /* BR = 650kbps and BW = 888kHz   */
#define LR2021_FLRC_CR_1_2              0x00U
#define LR2021_FLRC_PULSE_BT1           0x07U

/* FLRC packet params byte 2: (sync_len<<4)|preamble_len */
#define LR2021_FLRC_PREAMBLE_32B        0x07U
#define LR2021_FLRC_SYNC_OFF            0x00U
/* FLRC packet params byte 3: (crc<<6)|(header<<5)|(sync_match<<2)|tx_sw */
#define LR2021_FLRC_HEADER_FIXED        0x01U
#define LR2021_FLRC_CRC_2B              0x01U

/* ------------------------------------------------------------------ */
/* TX / IRQ common settings                                             */
/* ------------------------------------------------------------------ */
/* TX power in half-dBm steps (int8_t cast):  +13 dBm → 26 */
#define LR2021_TX_POWER_HALF_DBM        ((int8_t)(13 * 2))
#define LR2021_RAMP_16US                0x03U

/* IRQ flags (32-bit) */
#define LR2021_IRQ_RX_DONE              0x00040000UL
#define LR2021_IRQ_TX_DONE              0x00080000UL
#define LR2021_IRQ_TIMEOUT              0x00200000UL

/*
 * RTC step ≈ 30.52 µs (LR20xx, same as LR11xx architecture).
 * Converts milliseconds to 24-bit RTC timeout steps.
 */
#define LR2021_MS_TO_RTC(ms)    ((uint32_t)((ms) * 1000UL / 30UL))

#define LR2021_MAX_LORA_PAYLOAD         255U
#define LR2021_MAX_FLRC_PAYLOAD         511U

/* ------------------------------------------------------------------ */
/* Mode                                                                 */
/* ------------------------------------------------------------------ */
typedef enum {
    LR2021_MODE_LORA = 0,
    LR2021_MODE_FLRC
} LR2021_Mode_t;

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef LR2021_Init(SPI_HandleTypeDef *hspi);

/* Switch modulation mode — call before any send/receive */
HAL_StatusTypeDef LR2021_SetMode(SPI_HandleTypeDef *hspi, LR2021_Mode_t mode);

/* LoRa: commands and ACKs */
HAL_StatusTypeDef LR2021_LoRa_Send(SPI_HandleTypeDef *hspi,
                                   const uint8_t *data, uint8_t len);
HAL_StatusTypeDef LR2021_LoRa_Receive(SPI_HandleTypeDef *hspi,
                                      uint8_t *data, uint8_t *rx_len,
                                      uint32_t timeout_ms);

/* FLRC: bulk data */
HAL_StatusTypeDef LR2021_FLRC_Send(SPI_HandleTypeDef *hspi,
                                   const uint8_t *data, uint16_t len);

#endif /* LR2021_H */

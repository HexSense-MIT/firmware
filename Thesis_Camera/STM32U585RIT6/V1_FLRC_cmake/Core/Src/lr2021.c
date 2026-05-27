#include <stdio.h>

#include "lr2021.h"
#include "main.h"

#define SPI_TIMEOUT_MS   100U
#define BUSY_TIMEOUT_MS  2000U

static LR2021_Mode_t s_current_mode = LR2021_MODE_LORA;

/* ------------------------------------------------------------------ */
/* SPI helpers                                                          */
/* ------------------------------------------------------------------ */

static inline void cs_low(void)
{
    HAL_GPIO_WritePin(LR2021_CS_PORT, LR2021_CS_PIN, GPIO_PIN_RESET);
}

static inline void cs_high(void)
{
    HAL_GPIO_WritePin(LR2021_CS_PORT, LR2021_CS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef wait_busy(void)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GPIO_ReadPin(LR2021_BUSY_PORT, LR2021_BUSY_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > BUSY_TIMEOUT_MS)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

/* Write command: 16-bit opcode + optional parameter bytes */
static HAL_StatusTypeDef send_cmd(SPI_HandleTypeDef *hspi, uint16_t opcode,
                                  const uint8_t *params, uint16_t n)
{
    HAL_StatusTypeDef ret = wait_busy();
    if (ret != HAL_OK) return ret;

    uint8_t op[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode) };
    cs_low();
    ret = HAL_SPI_Transmit(hspi, op, 2, SPI_TIMEOUT_MS);
    if (ret == HAL_OK && params && n)
        ret = HAL_SPI_Transmit(hspi, (uint8_t *)params, n, SPI_TIMEOUT_MS);
    cs_high();
    return ret;
}

/* Normal read: command phase, NSS toggle, dummy byte, then response bytes. */
static HAL_StatusTypeDef read_cmd(SPI_HandleTypeDef *hspi, uint16_t opcode,
                                  uint8_t *rx, uint16_t n)
{
    HAL_StatusTypeDef ret = wait_busy();
    if (ret != HAL_OK) return ret;

    uint8_t op[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode) };
    cs_low();
    ret = HAL_SPI_Transmit(hspi, op, 2, SPI_TIMEOUT_MS);
    cs_high();
    if (ret != HAL_OK) return ret;

    ret = wait_busy();
    if (ret != HAL_OK) return ret;

    // uint8_t tx[2] = {0x00, 0x00};
    // uint8_t discard[2] = {0x00, 0x00};
    // uint8_t dummy = 0x00;
    cs_low();
    ret = HAL_SPI_Receive(hspi, (uint8_t *)rx, n, SPI_TIMEOUT_MS);
    // ret = HAL_SPI_TransmitReceive(hspi, &tx, &discard, 2, SPI_TIMEOUT_MS);
    // for (uint16_t i = 0; ret == HAL_OK && i < n; i++) {
    //     ret = HAL_SPI_TransmitReceive(hspi, &dummy, &rx[i], 1, SPI_TIMEOUT_MS);
    // }
    cs_high();
    return ret;
}

/* FIFO read is a one-step command + response transaction. */
static HAL_StatusTypeDef read_fifo(SPI_HandleTypeDef *hspi, uint8_t *rx, uint16_t n)
{
    HAL_StatusTypeDef ret = wait_busy();
    if (ret != HAL_OK) return ret;

    uint8_t op[2] = { (uint8_t)(LR2021_CMD_READ_FIFO >> 8), (uint8_t)(LR2021_CMD_READ_FIFO) };
    uint8_t tx = 0;
    cs_low();
    ret = HAL_SPI_Transmit(hspi, op, 2, SPI_TIMEOUT_MS);
    for (uint16_t i = 0; ret == HAL_OK && i < n; i++) {
        tx = 0;
        ret = HAL_SPI_TransmitReceive(hspi, &tx, &rx[i], 1, SPI_TIMEOUT_MS);
    }
    cs_high();
    return ret;
}

/* ------------------------------------------------------------------ */
/* Shared helpers                                                       */
/* ------------------------------------------------------------------ */

static HAL_StatusTypeDef cmdClearIrq(SPI_HandleTypeDef *hspi, uint32_t mask)
{
    uint8_t p[4] = {
        (uint8_t)(mask >> 24),
        (uint8_t)(mask >> 16),
        (uint8_t)(mask >> 8),
        (uint8_t)(mask)
    };

    HAL_StatusTypeDef ret = send_cmd(hspi, LR2021_CMD_CLEAR_IRQ, p, sizeof(p));
    return ret;
}

static HAL_StatusTypeDef cmdCalibrate(SPI_HandleTypeDef *hspi, uint8_t mask)
{
    HAL_StatusTypeDef ret = send_cmd(hspi, OC_CALIBRATE, &mask, 1);
    HAL_Delay(5);  // calibration takes a few ms
    return ret;
}

static HAL_StatusTypeDef set_standby(SPI_HandleTypeDef *hspi)
{
    uint8_t p = LR2021_STANDBY_XOSC;
    return send_cmd(hspi, LR2021_CMD_SET_STANDBY, &p, 1);
}

static HAL_StatusTypeDef set_rf_freq(SPI_HandleTypeDef *hspi)
{
    uint32_t f = LR2021_FREQ_HZ;
    uint8_t freq[4] = {
        (uint8_t)(f >> 24), (uint8_t)(f >> 16),
        (uint8_t)(f >> 8),  (uint8_t)(f)
    };
    return send_cmd(hspi, LR2021_CMD_SET_RF_FREQ, freq, 4);
}

static HAL_StatusTypeDef set_pa_config(SPI_HandleTypeDef *hspi)
{
    uint8_t p[3] = {
        (uint8_t)((LR2021_PA_SEL_LF << 7) | LR2021_LF_PA_MODE),
        (uint8_t)((LR2021_LF_PA_DUTY << 4) | LR2021_LF_PA_SLICES),
        LR2021_HF_PA_DUTY
    };

    return send_cmd(hspi, LR2021_CMD_SET_PA_CFG, p, sizeof(p));
}

static HAL_StatusTypeDef set_tx_params(SPI_HandleTypeDef *hspi)
{
    uint8_t p[2] = { (uint8_t)LR2021_TX_POWER_HALF_DBM, LR2021_RAMP_16US };
    return send_cmd(hspi, LR2021_CMD_SET_TX_PARAMS, p, 2);
}

static HAL_StatusTypeDef set_lora_syncword(SPI_HandleTypeDef *hspi)
{
    uint8_t syncword = LR2021_LORA_SYNCWORD_PRIVATE;
    return send_cmd(hspi, LR2021_CMD_LORA_SET_SYNCWORD, &syncword, 1);
}

/* Route selected system IRQs to DIO5. */
static HAL_StatusTypeDef cfg_irq(SPI_HandleTypeDef *hspi, uint32_t mask)
{
    uint8_t dio_func[2] = {
        LR2021_DIO5,
        (uint8_t)((LR2021_DIO_FUNC_IRQ << 4) | LR2021_DIO_DRIVE_NONE)
    };
    HAL_StatusTypeDef ret = send_cmd(hspi, LR2021_CMD_SET_DIO_FUNC, dio_func, sizeof(dio_func));
    if (ret != HAL_OK) return ret;

    uint8_t dio_irq[5] = {
        LR2021_DIO5,
        (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
        (uint8_t)(mask >> 8),  (uint8_t)(mask)
    };
    return send_cmd(hspi, LR2021_CMD_SET_DIO_IRQ_CFG, dio_irq, sizeof(dio_irq));
}

static HAL_StatusTypeDef poll_irq(SPI_HandleTypeDef *hspi, uint32_t expect_mask,
                                  uint32_t timeout_ms, uint32_t *irq_out)
{
  uint32_t t0 = HAL_GetTick();
  while (1) {
    if (HAL_GetTick() - t0 > timeout_ms)
        return HAL_TIMEOUT;

    /* Fast GPIO check on DIO5 (PC6) before touching SPI */
    // if (HAL_GPIO_ReadPin(LR2021_IRQ_PORT, LR2021_IRQ_PIN) == GPIO_PIN_RESET)
    //     continue;

    /* IRQ line is high — read and atomically clear via SPI */
    uint8_t raw[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    HAL_StatusTypeDef ret = read_cmd(hspi, LR2021_CMD_GET_AND_CLR_IRQ, raw, sizeof(raw));
    if (ret != HAL_OK) return ret;

    uint32_t irq = ((uint32_t)raw[2] << 24) | ((uint32_t)raw[3] << 16)
                  | ((uint32_t)raw[4] << 8)  |  raw[5];
    if (irq & expect_mask) {
        if (irq_out) {
            *irq_out = irq;
        }
        return HAL_OK;
    }
  }
}

static uint32_t rx_timeout_ms_to_rtc(uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        return LR2021_RX_CONTINUOUS_TIMEOUT_RTC;
    }

    uint32_t rtc = LR2021_MS_TO_RTC(timeout_ms);
    return (rtc > LR2021_RX_CONTINUOUS_TIMEOUT_RTC) ? LR2021_RX_CONTINUOUS_TIMEOUT_RTC : rtc;
}

static HAL_StatusTypeDef configure_lora_rx_packet(SPI_HandleTypeDef *hspi)
{
    uint8_t pkt[4] = {
        (uint8_t)(LR2021_LORA_PREAMBLE_SYMBOLS >> 8),
        (uint8_t)(LR2021_LORA_PREAMBLE_SYMBOLS),
        LR2021_MAX_LORA_PAYLOAD,
        (LR2021_LORA_PKT_EXPLICIT << 2) | (LR2021_LORA_CRC_ON << 1) | LR2021_LORA_IQ_STANDARD
    };
    return send_cmd(hspi, LR2021_CMD_LORA_SET_PKT_PARAMS, pkt, sizeof(pkt));
}

/* ------------------------------------------------------------------ */
/* Public: Init                                                         */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef LR2021_Init(SPI_HandleTypeDef *hspi)
{
  /* Enable LDO for LR2021 (ensure power is on before reset) */
  HAL_GPIO_WritePin(LR_PWR_EN_GPIO_Port, LR_PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  /* Hardware reset */
  HAL_GPIO_WritePin(LR2021_RESET_PORT, LR2021_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(LR2021_RESET_PORT, LR2021_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(20);

  /* Initial standby to ensure clean state before mode config */
  HAL_StatusTypeDef ret = cmdCalibrate(hspi, 0x3F);  // all calibrations
  if (ret != HAL_OK) return ret;

  ret = set_standby(hspi);
  if (ret != HAL_OK) return ret;

  /* Default to LoRa mode (used for commands / ACKs) */
  return LR2021_SetMode(hspi, LR2021_MODE_LORA);
}

/* ------------------------------------------------------------------ */
/* Public: SetMode                                                      */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef LR2021_SetMode(SPI_HandleTypeDef *hspi, LR2021_Mode_t mode)
{
    HAL_StatusTypeDef ret;

    ret = set_standby(hspi);
    if (ret != HAL_OK) return ret;

    if (mode == LR2021_MODE_LORA) {
        /* Packet type: LoRa */
        uint8_t pt = LR2021_PKT_TYPE_LORA;
        ret = send_cmd(hspi, LR2021_CMD_SET_PACKET_TYPE, &pt, 1);
        if (ret != HAL_OK) return ret;

        ret = set_rf_freq(hspi);
        if (ret != HAL_OK) return ret;

        ret = set_pa_config(hspi);
        if (ret != HAL_OK) return ret;

        ret = set_tx_params(hspi);
        if (ret != HAL_OK) return ret;

        /*
         * LoRa modulation params (2 bytes):
         *   byte 0: (SF<<4)|BW  — SF7, BW 250kHz → (0x07<<4)|0x06 = 0x76
         *   byte 1: (CR<<4)|PPM — CR 4/5, PPM off → (0x01<<4)|0x00 = 0x10
         */
        uint8_t mod[2] = {
            (LR2021_LORA_SF7 << 4) | LR2021_LORA_BW_250,
            (LR2021_LORA_CR_4_5 << 4) | LR2021_LORA_PPM_OFF
        };
        ret = send_cmd(hspi, LR2021_CMD_LORA_SET_MOD_PARAMS, mod, 2);
        if (ret != HAL_OK) return ret;

        ret = set_lora_syncword(hspi);

    } else {
        /* Packet type: FLRC */
        uint8_t pt = LR2021_PKT_TYPE_FLRC;
        ret = send_cmd(hspi, LR2021_CMD_SET_PACKET_TYPE, &pt, 1);
        if (ret != HAL_OK) return ret;

        ret = set_rf_freq(hspi);
        if (ret != HAL_OK) return ret;

        ret = set_tx_params(hspi);
        if (ret != HAL_OK) return ret;

        /*
         * FLRC modulation params (2 bytes):
         *   byte 0: BR_BW — 1.0 Mbps / 1.0 MHz BW → 0x04
         *   byte 1: (CR<<4)|pulse_shape — CR 1/2, BT 1.0 → (0x00<<4)|0x07 = 0x07
         */
        uint8_t mod[2] = {
            LR2021_FLRC_BR_BW_2_6M_2_6M,
            (LR2021_FLRC_CR_1_2 << 4) | LR2021_FLRC_PULSE_BT1
        };
        ret = send_cmd(hspi, LR2021_CMD_FLRC_SET_MOD_PARAMS, mod, 2);
    }

    if (ret == HAL_OK)
        s_current_mode = mode;

    return ret;
}

/* ------------------------------------------------------------------ */
/* Public: LoRa send (commands / ACKs)                                 */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef LR2021_LoRa_Send(SPI_HandleTypeDef *hspi,
                                   const uint8_t *data, uint8_t len)
{
    if (!data || len == 0 || len > LR2021_MAX_LORA_PAYLOAD)
        return HAL_ERROR;

    HAL_StatusTypeDef ret;

    cmdClearIrq(hspi, LR2021_IRQ_ALL);  // clear all pending IRQs before TX

    /*
     * LoRa packet params (4 bytes after opcode):
     *   [0:1] preamble symbols (big-endian 16-bit)
     *   [2]   payload length
     *   [3]   (pkt_mode<<2)|(crc<<1)|(iq)
     *         explicit header, CRC on, standard IQ → (0x00<<2)|(0x01<<1)|0x00 = 0x02
     */
    uint8_t pkt[4] = {
        (uint8_t)(LR2021_LORA_PREAMBLE_SYMBOLS >> 8),
        (uint8_t)(LR2021_LORA_PREAMBLE_SYMBOLS),
        len,
        (LR2021_LORA_PKT_IMPLICIT << 2) | (LR2021_LORA_CRC_ON << 1) | LR2021_LORA_IQ_STANDARD
    };
    ret = send_cmd(hspi, LR2021_CMD_LORA_SET_PKT_PARAMS, pkt, 4);
    if (ret != HAL_OK) return ret;

    /* Write payload to TX FIFO */
    ret = wait_busy();
    if (ret != HAL_OK) {
        return ret;
    }
    uint8_t op[2] = { (uint8_t)(LR2021_CMD_WRITE_FIFO >> 8), (uint8_t)(LR2021_CMD_WRITE_FIFO) };
    cs_low();
    ret = HAL_SPI_Transmit(hspi, op, 2, SPI_TIMEOUT_MS);
    if (ret == HAL_OK)
        ret = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, SPI_TIMEOUT_MS);
    cs_high();
    if (ret != HAL_OK) return ret;

    ret = cfg_irq(hspi, LR2021_IRQ_TX_DONE);
    if (ret != HAL_OK) return ret;

    cmdClearIrq(hspi, LR2021_IRQ_ALL);  // clear all pending IRQs before TX

    /* SetTx: single-shot (timeout = 10ms, 32MHz RTC) */
    uint8_t tx_to[3] = { 0x05, 0x00, 0x00 };
    ret = send_cmd(hspi, LR2021_CMD_SET_TX, tx_to, 3);
    if (ret != HAL_OK) return ret;

    // uint32_t irq_read = 0;
    ret = poll_irq(hspi, LR2021_IRQ_TX_DONE, 200U, NULL);
    if (ret != HAL_OK) return ret;

    return set_standby(hspi);
}

/* ------------------------------------------------------------------ */
/* Public: LoRa receive (ACKs / incoming commands)                     */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef LR2021_LoRa_Receive(SPI_HandleTypeDef *hspi,
                                      uint8_t *data, uint8_t *rx_len,
                                      uint32_t timeout_ms)
{
    if (!data || !rx_len) return HAL_ERROR;

    uint32_t rx_mask = LR2021_IRQ_RX_DONE | LR2021_IRQ_TIMEOUT |
                       LR2021_IRQ_CRC_ERROR | LR2021_IRQ_LEN_ERROR;
    HAL_StatusTypeDef ret = LR2021_LoRa_StartReceive(hspi, timeout_ms);
    if (ret != HAL_OK) return ret;

    /* Wait for RX_DONE or TIMEOUT */
    uint32_t irq = 0;
    ret = poll_irq(hspi, rx_mask,
                   timeout_ms + 100U, &irq);
    if (ret != HAL_OK) {
        set_standby(hspi);
        return ret;
    }

    if (irq & LR2021_IRQ_TIMEOUT) {
        set_standby(hspi);
        return HAL_TIMEOUT;
    }
    if (irq & (LR2021_IRQ_CRC_ERROR | LR2021_IRQ_LEN_ERROR)) {
        set_standby(hspi);
        return HAL_ERROR;
    }

    ret = LR2021_LoRa_ReadPayload(hspi, data, rx_len);
    if (ret != HAL_OK) {
        set_standby(hspi);
        return ret;
    }

    return set_standby(hspi);
}

HAL_StatusTypeDef LR2021_LoRa_StartReceive(SPI_HandleTypeDef *hspi,
                                           uint32_t timeout_ms)
{
    HAL_StatusTypeDef ret = configure_lora_rx_packet(hspi);
    if (ret != HAL_OK) return ret;

    ret = cfg_irq(hspi, LR2021_IRQ_RX_DONE | LR2021_IRQ_TIMEOUT |
                        LR2021_IRQ_CRC_ERROR | LR2021_IRQ_LEN_ERROR);
    if (ret != HAL_OK) return ret;

    uint32_t rtc = rx_timeout_ms_to_rtc(timeout_ms);
    uint8_t rx_to[3] = {
        (uint8_t)(rtc >> 16), (uint8_t)(rtc >> 8), (uint8_t)(rtc)
    };
    return send_cmd(hspi, LR2021_CMD_SET_RX, rx_to, sizeof(rx_to));
}

/* ------------------------------------------------------------------ */
/* Public: FLRC send (bulk data)                                       */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef LR2021_FLRC_Send(SPI_HandleTypeDef *hspi,
                                   const uint8_t *data, uint16_t len)
{
    if (!data || len == 0 || len > LR2021_MAX_FLRC_PAYLOAD)
        return HAL_ERROR;

    HAL_StatusTypeDef ret;

    /*
     * FLRC packet params (4 bytes after opcode):
     *   [0] = sync_word_len + (preamble_len << 2)
     *         sync OFF (0x00), 32-bit preamble (0x07) -> 0x1C
     *   [1] = crc + (header << 2) + (sync_match << 3) + (tx_syncword << 6)
     *         CRC 2B (0x01), fixed header (0x01), no sync -> 0x05
     *   [2:3] = payload length big-endian
     */
    uint8_t pkt[4] = {
        LR2021_FLRC_SYNC_WORD_LEN_OFF + (LR2021_FLRC_PREAMBLE_32B << 2),
        LR2021_FLRC_CRC_2B + (LR2021_FLRC_HEADER_FIXED << 2) +
            (LR2021_FLRC_MATCH_SYNCWORD_OFF << 3) + (LR2021_FLRC_TX_SYNCWORD_NONE << 6),
        (uint8_t)(len >> 8),
        (uint8_t)(len)
    };
    ret = send_cmd(hspi, LR2021_CMD_FLRC_SET_PKT_PARAMS, pkt, 4);
    if (ret != HAL_OK) return ret;

    /* Write payload to TX FIFO */
    ret = wait_busy();
    if (ret != HAL_OK) return ret;
    uint8_t op[2] = { (uint8_t)(LR2021_CMD_WRITE_FIFO >> 8), (uint8_t)(LR2021_CMD_WRITE_FIFO) };
    cs_low();
    ret = HAL_SPI_Transmit(hspi, op, 2, SPI_TIMEOUT_MS);
    if (ret == HAL_OK)
        ret = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, SPI_TIMEOUT_MS);
    cs_high();
    if (ret != HAL_OK) return ret;

    ret = cfg_irq(hspi, LR2021_IRQ_TX_DONE);
    if (ret != HAL_OK) return ret;

    uint8_t tx_to[3] = { 0x00, 0x00, 0x00 };
    ret = send_cmd(hspi, LR2021_CMD_SET_TX, tx_to, 3);
    if (ret != HAL_OK) return ret;

    ret = poll_irq(hspi, LR2021_IRQ_TX_DONE, 500U, NULL);
    if (ret != HAL_OK) return ret;

    return set_standby(hspi);
}

/* ------------------------------------------------------------------ */
/* Public: read/clear IRQ flags                                         */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef LR2021_HandleIRQ(SPI_HandleTypeDef *hspi, uint32_t *irq_out)
{
    uint8_t raw[4] = {0};
    HAL_StatusTypeDef ret = read_cmd(hspi, LR2021_CMD_GET_AND_CLR_IRQ, raw, 4);
    if (ret != HAL_OK) return ret;

    uint32_t irq = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16)
                  | ((uint32_t)raw[2] << 8)  |  raw[3];
    if (irq_out) *irq_out = irq;
    return HAL_OK;
}

/* ------------------------------------------------------------------ */
/* Public: read LoRa payload after RX_DONE                              */
/* ------------------------------------------------------------------ */
HAL_StatusTypeDef LR2021_LoRa_ReadPayload(SPI_HandleTypeDef *hspi,
                                          uint8_t *data, uint8_t *rx_len)
{
    if (!data || !rx_len) return HAL_ERROR;

    HAL_StatusTypeDef ret;
    uint8_t len_raw[2] = {0};
    ret = read_cmd(hspi, LR2021_CMD_GET_RX_PKT_LEN, len_raw, 2);
    if (ret != HAL_OK) return ret;

    uint16_t pld_len = ((uint16_t)len_raw[0] << 8) | len_raw[1];
    if (pld_len > LR2021_MAX_LORA_PAYLOAD) pld_len = LR2021_MAX_LORA_PAYLOAD;
    *rx_len = (uint8_t)pld_len;

    ret = read_fifo(hspi, data, *rx_len);
    return ret;
}

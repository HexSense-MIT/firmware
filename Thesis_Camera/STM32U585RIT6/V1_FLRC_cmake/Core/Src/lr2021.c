#include "lr2021.h"

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

/* Read command: 16-bit opcode, then receive n bytes */
static HAL_StatusTypeDef send_recv(SPI_HandleTypeDef *hspi, uint16_t opcode,
                                   uint8_t *rx, uint16_t n)
{
    HAL_StatusTypeDef ret = wait_busy();
    if (ret != HAL_OK) return ret;

    uint8_t op[2] = { (uint8_t)(opcode >> 8), (uint8_t)(opcode) };
    cs_low();
    ret = HAL_SPI_Transmit(hspi, op, 2, SPI_TIMEOUT_MS);
    if (ret == HAL_OK)
        ret = HAL_SPI_Receive(hspi, rx, n, SPI_TIMEOUT_MS);
    cs_high();
    return ret;
}

/* ------------------------------------------------------------------ */
/* Shared helpers                                                       */
/* ------------------------------------------------------------------ */

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

static HAL_StatusTypeDef set_tx_params(SPI_HandleTypeDef *hspi)
{
    uint8_t p[2] = { (uint8_t)LR2021_TX_POWER_HALF_DBM, LR2021_RAMP_16US };
    return send_cmd(hspi, LR2021_CMD_SET_TX_PARAMS, p, 2);
}

/* Configure and poll IRQ flags.  Returns when the expected bit fires or timeout. */
static HAL_StatusTypeDef cfg_irq(SPI_HandleTypeDef *hspi, uint32_t mask)
{
    /* Enable mask globally and on DIO1; DIO2 unused */
    uint8_t cfg[10] = {
        (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
        (uint8_t)(mask >> 8),  (uint8_t)(mask),
        (uint8_t)(mask >> 24), (uint8_t)(mask >> 16),
        (uint8_t)(mask >> 8),  (uint8_t)(mask),
        0x00, 0x00
    };
    return send_cmd(hspi, LR2021_CMD_CFG_IRQ, cfg, 10);
}

static HAL_StatusTypeDef poll_irq(SPI_HandleTypeDef *hspi, uint32_t expect_mask,
                                  uint32_t timeout_ms, uint32_t *irq_out)
{
    uint32_t t0 = HAL_GetTick();
    while (1) {
        if (HAL_GetTick() - t0 > timeout_ms)
            return HAL_TIMEOUT;

        /* Fast GPIO check on DIO5 (PC6) before touching SPI */
        if (HAL_GPIO_ReadPin(LR2021_IRQ_PORT, LR2021_IRQ_PIN) == GPIO_PIN_RESET)
            continue;

        /* IRQ line is high — read and atomically clear via SPI */
        uint8_t raw[4] = { 0 };
        HAL_StatusTypeDef ret = send_recv(hspi, LR2021_CMD_GET_AND_CLR_IRQ, raw, 4);
        if (ret != HAL_OK) return ret;

        uint32_t irq = ((uint32_t)raw[0] << 24) | ((uint32_t)raw[1] << 16)
                     | ((uint32_t)raw[2] << 8)  |  raw[3];
        if (irq & expect_mask) {
            if (irq_out) *irq_out = irq;
            return HAL_OK;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public: Init                                                         */
/* ------------------------------------------------------------------ */

HAL_StatusTypeDef LR2021_Init(SPI_HandleTypeDef *hspi)
{
    /* Hardware reset */
    HAL_GPIO_WritePin(LR2021_RESET_PORT, LR2021_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LR2021_RESET_PORT, LR2021_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(20);

    /* Initial standby to ensure clean state before mode config */
    HAL_StatusTypeDef ret = set_standby(hspi);
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

        ret = set_tx_params(hspi);
        if (ret != HAL_OK) return ret;

        /*
         * LoRa modulation params (2 bytes):
         *   byte 0: (SF<<4)|BW  — SF7, BW 500kHz → (0x07<<4)|0x06 = 0x76
         *   byte 1: (CR<<4)|PPM — CR 4/5, PPM off → (0x01<<4)|0x00 = 0x10
         */
        uint8_t mod[2] = {
            (LR2021_LORA_SF7 << 4) | LR2021_LORA_BW_500,
            (LR2021_LORA_CR_4_5 << 4) | LR2021_LORA_PPM_OFF
        };
        ret = send_cmd(hspi, LR2021_CMD_LORA_SET_MOD_PARAMS, mod, 2);

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
        (LR2021_LORA_PKT_EXPLICIT << 2) | (LR2021_LORA_CRC_ON << 1) | LR2021_LORA_IQ_STANDARD
    };
    ret = send_cmd(hspi, LR2021_CMD_LORA_SET_PKT_PARAMS, pkt, 4);
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

    /* SetTx: single-shot (timeout = 0) */
    uint8_t tx_to[3] = { 0x00, 0x00, 0x00 };
    ret = send_cmd(hspi, LR2021_CMD_SET_TX, tx_to, 3);
    if (ret != HAL_OK) return ret;

    ret = poll_irq(hspi, LR2021_IRQ_TX_DONE, 500U, NULL);
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

    HAL_StatusTypeDef ret;

    /* Packet params: same as TX side, max payload */
    uint8_t pkt[4] = {
        (uint8_t)(LR2021_LORA_PREAMBLE_SYMBOLS >> 8),
        (uint8_t)(LR2021_LORA_PREAMBLE_SYMBOLS),
        LR2021_MAX_LORA_PAYLOAD,
        (LR2021_LORA_PKT_EXPLICIT << 2) | (LR2021_LORA_CRC_ON << 1) | LR2021_LORA_IQ_STANDARD
    };
    ret = send_cmd(hspi, LR2021_CMD_LORA_SET_PKT_PARAMS, pkt, 4);
    if (ret != HAL_OK) return ret;

    ret = cfg_irq(hspi, LR2021_IRQ_RX_DONE | LR2021_IRQ_TIMEOUT);
    if (ret != HAL_OK) return ret;

    /* SetRx with hardware timeout converted to RTC steps */
    uint32_t rtc = LR2021_MS_TO_RTC(timeout_ms);
    uint8_t rx_to[3] = {
        (uint8_t)(rtc >> 16), (uint8_t)(rtc >> 8), (uint8_t)(rtc)
    };
    ret = send_cmd(hspi, LR2021_CMD_SET_RX, rx_to, 3);
    if (ret != HAL_OK) return ret;

    /* Wait for RX_DONE or TIMEOUT */
    uint32_t irq = 0;
    ret = poll_irq(hspi, LR2021_IRQ_RX_DONE | LR2021_IRQ_TIMEOUT,
                   timeout_ms + 100U, &irq);
    if (ret != HAL_OK) return ret;

    if (irq & LR2021_IRQ_TIMEOUT) {
        set_standby(hspi);
        return HAL_TIMEOUT;
    }

    /* Read payload length from chip */
    uint8_t len_raw[2] = { 0 };
    ret = send_recv(hspi, LR2021_CMD_GET_RX_PKT_LEN, len_raw, 2);
    if (ret != HAL_OK) return ret;

    uint16_t pld_len = ((uint16_t)len_raw[0] << 8) | len_raw[1];
    if (pld_len > LR2021_MAX_LORA_PAYLOAD) pld_len = LR2021_MAX_LORA_PAYLOAD;
    *rx_len = (uint8_t)pld_len;

    /* Read payload from RX FIFO */
    ret = send_recv(hspi, LR2021_CMD_READ_FIFO, data, *rx_len);
    if (ret != HAL_OK) return ret;

    return set_standby(hspi);
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
     *   [0] = (sync_len<<4)|preamble_len
     *         sync OFF (0x00), 32-bit preamble (0x07) → 0x07
     *   [1] = (crc<<6)|(header<<5)|(sync_match<<2)|tx_sw
     *         CRC 2B (0x01), fixed header (0x01), no sync → (1<<6)|(1<<5) = 0x60
     *   [2:3] = payload length big-endian
     */
    uint8_t pkt[4] = {
        (LR2021_FLRC_SYNC_OFF << 4) | LR2021_FLRC_PREAMBLE_32B,
        (LR2021_FLRC_CRC_2B << 6) | (LR2021_FLRC_HEADER_FIXED << 5),
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

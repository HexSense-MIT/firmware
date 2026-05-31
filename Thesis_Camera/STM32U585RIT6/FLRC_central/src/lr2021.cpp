#include "lr2021.h"

// ============================================================
//  SPI opcode constants (2-byte opcodes, MSB sent first)
//  Source: Lora-net/usp smtc_rac_lib/radio_drivers/lr20xx_driver
// ============================================================

// System
static constexpr uint16_t OC_SET_SLEEP         = 0x0127;
static constexpr uint16_t OC_SET_STANDBY       = 0x0128;
static constexpr uint16_t OC_CALIBRATE         = 0x0122;
static constexpr uint16_t OC_CALIBRATE_FRONT_END = 0x0123;
static constexpr uint16_t OC_GET_ERRORS        = 0x0110;
static constexpr uint16_t OC_CLEAR_ERRORS      = 0x0111;
static constexpr uint16_t OC_SET_DIO_FUNC      = 0x0112;
static constexpr uint16_t OC_SET_DIO_IRQ_CFG   = 0x0115;
static constexpr uint16_t OC_CLEAR_IRQ         = 0x0116;
static constexpr uint16_t OC_GET_CLR_IRQ       = 0x0117;

// Radio common
static constexpr uint16_t OC_SET_PKT_TYPE      = 0x0207;
static constexpr uint16_t OC_SET_RF_FREQ       = 0x0200;
static constexpr uint16_t OC_SET_RXPATH_CFG    = 0x0201;
static constexpr uint16_t OC_SET_PA_CFG        = 0x0202;
static constexpr uint16_t OC_SET_TX_PARAMS     = 0x0203;
static constexpr uint16_t OC_SET_RX            = 0x020C;
static constexpr uint16_t OC_SET_TX            = 0x020D;
static constexpr uint16_t OC_GET_RX_PKT_LEN   = 0x0212;
static constexpr uint16_t OC_GET_RSSI_INST     = 0x020B;

// LoRa
static constexpr uint16_t OC_LORA_SET_MOD      = 0x0220;
static constexpr uint16_t OC_LORA_SET_PKT      = 0x0221;
static constexpr uint16_t OC_LORA_SET_SYNCWORD = 0x0223;
static constexpr uint16_t OC_LORA_GET_PKT_STAT = 0x022A;

// FLRC
static constexpr uint16_t OC_FLRC_SET_MOD      = 0x0248;
static constexpr uint16_t OC_FLRC_SET_PKT      = 0x0249;
static constexpr uint16_t OC_FLRC_SET_SYNCWORD = 0x024C;
static constexpr uint16_t OC_FLRC_GET_PKT_STAT = 0x024B;

// FIFO
static constexpr uint16_t OC_FIFO_READ         = 0x0001;
static constexpr uint16_t OC_FIFO_WRITE        = 0x0002;
static constexpr uint16_t OC_FIFO_GET_RX_LEVEL = 0x011C;
static constexpr uint16_t OC_FIFO_CLEAR_RX     = 0x011E;

// DIO5 is the general-purpose IRQ output on the LR2021 module
static constexpr uint8_t DIO5 = 0x05;
static constexpr uint8_t DIO_FUNC_IRQ = 0x01;
static constexpr uint8_t DIO_DRIVE_NONE = 0x00;

// FLRC SetFlrcPacketParams byte encoding (Table 18-5, DS.LR2021 Rev1.1)
// Byte2 [7:6]=rfu | [5:2]=agc_pbl_len | [1:0]=sync_len
//   agc_pbl_len=3 → 16-bit preamble (minimum for ≥650 kbps)
//   sync_len=2    → 32-bit (4-byte) syncword
static constexpr uint8_t FLRC_PKT_BYTE2_DEFAULT = (3U << 2) | 2U;

// Byte3 [7:6]=sync_tx | [5:3]=sync_match | [2]=pkt_format | [1:0]=Crc
//   sync_tx=1    → transmit using syncword slot 1
//   sync_match=1 → receive: match syncword 1 (MATCH_1)
//   pkt_format=0 → dynamic (variable-length)
//   Crc bits set by crc_type in cmdFLRCSetPktParams
static constexpr uint8_t FLRC_PKT_BYTE3_DEFAULT = (1U << 6) | (1U << 3);

static constexpr uint8_t  LR2021_FLRC_PREAMBLE_32B        = 0x07;
static constexpr uint8_t  LR2021_FLRC_SYNC_WORD_LEN_4B    = 0x02;
/* FLRC packet params byte 3: crc + (header<<2) + (sync_match<<3) + (tx_sw<<6) */
static constexpr uint8_t  LR2021_FLRC_HEADER_VARIABLE     = 0x00;
static constexpr uint8_t  LR2021_FLRC_CRC_2B              = 0x01;
static constexpr uint8_t  LR2021_FLRC_MATCH_SYNCWORD_1    = 0x01;
static constexpr uint8_t  LR2021_FLRC_TX_SYNCWORD_1       = 0x01;

// ============================================================
//  Constructor
// ============================================================

LR2021::LR2021(int pin_nss, int pin_reset, int pin_busy, int pin_dio, SPIClass* spi)
    : _pin_nss(pin_nss), _pin_reset(pin_reset), _pin_busy(pin_busy),
      _pin_dio(pin_dio), _spi(spi), _spi_freq(8000000UL) {}

// ============================================================
//  SPI HAL
// ============================================================

void LR2021::waitBusy(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while (digitalRead(_pin_busy) == HIGH) {
        if ((millis() - t0) > timeout_ms) return;
        delayMicroseconds(10);
    }
}

// Write cmd[] then optional data[] in a single NSS transaction.
void LR2021::halWrite(const uint8_t* cmd, uint8_t cmdLen,
                      const uint8_t* data, uint16_t dataLen) {
    waitBusy();
    _spi->beginTransaction(SPISettings(_spi_freq, MSBFIRST, SPI_MODE0));
    digitalWrite(_pin_nss, LOW);
    for (uint8_t i = 0; i < cmdLen; i++)  _spi->transfer(cmd[i]);
    for (uint16_t i = 0; i < dataLen; i++) _spi->transfer(data[i]);
    digitalWrite(_pin_nss, HIGH);
    _spi->endTransaction();
}

// LR20xx read: send cmd[] in one NSS transaction, wait BUSY, then
// send two dummy bytes and clock out data[] in a second transaction.
void LR2021::halRead(const uint8_t* cmd, uint8_t cmdLen,
                     uint8_t* data, uint16_t dataLen) {
    waitBusy();
    _spi->beginTransaction(SPISettings(_spi_freq, MSBFIRST, SPI_MODE0));
    digitalWrite(_pin_nss, LOW);
    for (uint8_t i = 0; i < cmdLen; i++) _spi->transfer(cmd[i]);
    digitalWrite(_pin_nss, HIGH);

    if (dataLen > 0) {
        waitBusy();
        digitalWrite(_pin_nss, LOW);
        _spi->transfer(0x00);
        _spi->transfer(0x00);
        for (uint16_t i = 0; i < dataLen; i++) data[i] = _spi->transfer(0x00);
        digitalWrite(_pin_nss, HIGH);
    }

    _spi->endTransaction();
}

// Toggle NSS and read bytes (no command — used for GetStatus).
void LR2021::halDirectRead(uint8_t* data, uint8_t len) {
    waitBusy();
    _spi->beginTransaction(SPISettings(_spi_freq, MSBFIRST, SPI_MODE0));
    digitalWrite(_pin_nss, LOW);
    for (uint8_t i = 0; i < len; i++) data[i] = _spi->transfer(0x00);
    digitalWrite(_pin_nss, HIGH);
    _spi->endTransaction();
}

// Write cmd[], then read data[] — FIFO variant (no extra NOP between).
void LR2021::halDirectReadFifo(const uint8_t* cmd, uint8_t cmdLen,
                                uint8_t* data, uint16_t dataLen) {
    waitBusy();
    _spi->beginTransaction(SPISettings(_spi_freq, MSBFIRST, SPI_MODE0));
    digitalWrite(_pin_nss, LOW);
    for (uint8_t i = 0; i < cmdLen; i++)    _spi->transfer(cmd[i]);
    for (uint16_t i = 0; i < dataLen; i++)   data[i] = _spi->transfer(0x00);
    digitalWrite(_pin_nss, HIGH);
    _spi->endTransaction();
}

// ============================================================
//  System commands
// ============================================================

void LR2021::cmdSetStandby(lr2021_standby_mode_t mode) {
    const uint8_t cmd[3] = { (uint8_t)(OC_SET_STANDBY >> 8),
                              (uint8_t)(OC_SET_STANDBY),
                              (uint8_t)mode };
    halWrite(cmd, 3);
}

void LR2021::cmdSetPacketType(lr2021_pkt_type_t type) {
    const uint8_t cmd[3] = { (uint8_t)(OC_SET_PKT_TYPE >> 8),
                              (uint8_t)(OC_SET_PKT_TYPE),
                              (uint8_t)type };
    halWrite(cmd, 3);
}

void LR2021::cmdSetRfFreq(uint32_t freq_hz) {
    const uint8_t cmd[6] = { (uint8_t)(OC_SET_RF_FREQ >> 8),
                              (uint8_t)(OC_SET_RF_FREQ),
                              (uint8_t)(freq_hz >> 24),
                              (uint8_t)(freq_hz >> 16),
                              (uint8_t)(freq_hz >> 8),
                              (uint8_t)(freq_hz) };
    halWrite(cmd, 6);
}

void LR2021::cmdSetRxPath(uint8_t rx_path, uint8_t rx_boost) {
    const uint8_t cmd[4] = { (uint8_t)(OC_SET_RXPATH_CFG >> 8),
                             (uint8_t)(OC_SET_RXPATH_CFG),
                             (uint8_t)(0x00 | rx_path & 0x01),
                             (uint8_t)(0x00 | rx_boost & 0x03) };
    halWrite(cmd, 4);
}

// Byte2: (pa_sel<<7)|lf_mode  Byte3: (lf_duty<<4)|lf_slices  Byte4: hf_duty
void LR2021::cmdSetPaConfig(lr2021_pa_sel_t pa_sel, uint8_t lf_mode,
                             uint8_t lf_duty, uint8_t lf_slices, uint8_t hf_duty) {
    const uint8_t cmd[5] = { (uint8_t)(OC_SET_PA_CFG >> 8),
                              (uint8_t)(OC_SET_PA_CFG),
                              (uint8_t)(((uint8_t)pa_sel << 7) | (lf_mode & 0x7F)),
                              (uint8_t)((lf_duty << 4) | (lf_slices & 0x0F)),
                              hf_duty };
    halWrite(cmd, 5);
}

// power_half_dbm = power_dBm * 2  (0.5 dBm resolution)
void LR2021::cmdSetTxParams(int8_t power_dbm, lr2021_ramp_time_t ramp) {
    const uint8_t cmd[4] = { (uint8_t)(OC_SET_TX_PARAMS >> 8),
                              (uint8_t)(OC_SET_TX_PARAMS),
                              (uint8_t)(power_dbm * 2),
                              (uint8_t)ramp };
    halWrite(cmd, 4);
}

void LR2021::cmdCalibrate(uint8_t mask) {
    const uint8_t cmd[3] = { (uint8_t)(OC_CALIBRATE >> 8),
                              (uint8_t)(OC_CALIBRATE),
                              mask };
    halWrite(cmd, 3);
    delay(5);  // calibration takes a few ms
    waitBusy(500);
}

void LR2021::cmdCalibrateFrontEnd(uint32_t freq_hz) {
    // LF path calibration value is ceil(freq_hz / 4 MHz).
    uint16_t freq_4mhz = (uint16_t)((freq_hz + 3999999UL) / 4000000UL);
    const uint8_t cmd[4] = { (uint8_t)(OC_CALIBRATE_FRONT_END >> 8),
                              (uint8_t)(OC_CALIBRATE_FRONT_END),
                              (uint8_t)(freq_4mhz >> 8),
                              (uint8_t)(freq_4mhz) };
    halWrite(cmd, 4);
}

uint16_t LR2021::cmdGetErrors() {
    const uint8_t cmd[2] = { (uint8_t)(OC_GET_ERRORS >> 8),
                              (uint8_t)(OC_GET_ERRORS) };
    uint8_t resp[2] = {0};
    halRead(cmd, 2, resp, 2);
    return (uint16_t)((uint16_t)resp[0] << 8) | resp[1];
}

void LR2021::cmdClearErrors() {
    const uint8_t cmd[2] = { (uint8_t)(OC_CLEAR_ERRORS >> 8),
                              (uint8_t)(OC_CLEAR_ERRORS) };
    halWrite(cmd, 2);
}

// Route irq_mask events to the specified DIO pin.
void LR2021::cmdSetDioIrq(uint8_t dio, uint32_t irq_mask) {
    const uint8_t cmd[7] = { (uint8_t)(OC_SET_DIO_IRQ_CFG >> 8),
                              (uint8_t)(OC_SET_DIO_IRQ_CFG),
                              dio,
                              (uint8_t)(irq_mask >> 24),
                              (uint8_t)(irq_mask >> 16),
                              (uint8_t)(irq_mask >> 8),
                              (uint8_t)(irq_mask) };
    halWrite(cmd, 7);
}

void LR2021::cmdSetDioFunction(uint8_t dio, uint8_t function, uint8_t drive) {
    const uint8_t cmd[4] = { (uint8_t)(OC_SET_DIO_FUNC >> 8),
                              (uint8_t)(OC_SET_DIO_FUNC),
                              dio,
                              (uint8_t)((function << 4) | (drive & 0x0F)) };
    halWrite(cmd, 4);
}

void LR2021::cmdClearIrq(uint32_t mask) {
    const uint8_t cmd[6] = { (uint8_t)(OC_CLEAR_IRQ >> 8),
                              (uint8_t)(OC_CLEAR_IRQ),
                              (uint8_t)(mask >> 24),
                              (uint8_t)(mask >> 16),
                              (uint8_t)(mask >> 8),
                              (uint8_t)(mask) };
    halWrite(cmd, 6);
}

uint32_t LR2021::cmdGetAndClearIrq() {
    const uint8_t cmd[2] = { (uint8_t)(OC_GET_CLR_IRQ >> 8),
                              (uint8_t)(OC_GET_CLR_IRQ) };
    uint8_t resp[4] = {0};
    halRead(cmd, 2, resp, 4);
    return ((uint32_t)resp[0] << 24) | ((uint32_t)resp[1] << 16)
         | ((uint32_t)resp[2] << 8)  | (uint32_t)resp[3];
}

// timeout_rtc in RTC steps (32768 Hz); 0 = no timeout for TX
void LR2021::cmdSetTx(uint32_t timeout_rtc) {
    const uint8_t cmd[5] = { (uint8_t)(OC_SET_TX >> 8),
                              (uint8_t)(OC_SET_TX),
                              (uint8_t)(timeout_rtc >> 16),
                              (uint8_t)(timeout_rtc >> 8),
                              (uint8_t)(timeout_rtc) };
    halWrite(cmd, 5);
}

// timeout_rtc = LR2021_RX_CONTINUOUS (0xFFFFFF) for continuous receive
void LR2021::cmdSetRx(uint32_t timeout_rtc) {
    const uint8_t cmd[5] = { (uint8_t)(OC_SET_RX >> 8),
                              (uint8_t)(OC_SET_RX),
                              (uint8_t)(timeout_rtc >> 16),
                              (uint8_t)(timeout_rtc >> 8),
                              (uint8_t)(timeout_rtc) };
    halWrite(cmd, 5);
}

uint16_t LR2021::cmdGetRxPacketLen() {
    const uint8_t cmd[2] = { (uint8_t)(OC_GET_RX_PKT_LEN >> 8),
                              (uint8_t)(OC_GET_RX_PKT_LEN) };
    uint8_t resp[2] = {0};
    halRead(cmd, 2, resp, 2);
    return (uint16_t)((uint16_t)resp[0] << 8) | resp[1];
}

uint16_t LR2021::cmdGetRxFifoLevel() {
    const uint8_t cmd[2] = { (uint8_t)(OC_FIFO_GET_RX_LEVEL >> 8),
                              (uint8_t)(OC_FIFO_GET_RX_LEVEL) };
    uint8_t resp[2] = {0};
    halRead(cmd, 2, resp, 2);
    return (uint16_t)((uint16_t)resp[0] << 8) | resp[1];
}

void LR2021::cmdClearRxFifo() {
    const uint8_t cmd[2] = { (uint8_t)(OC_FIFO_CLEAR_RX >> 8),
                              (uint8_t)(OC_FIFO_CLEAR_RX) };
    halWrite(cmd, 2);
}

void LR2021::cmdWriteFifo(const uint8_t* data, uint16_t len) {
    const uint8_t cmd[2] = { (uint8_t)(OC_FIFO_WRITE >> 8),
                              (uint8_t)(OC_FIFO_WRITE) };
    halWrite(cmd, 2, data, len);
}

void LR2021::cmdReadFifo(uint8_t* data, uint16_t len) {
    const uint8_t cmd[2] = { (uint8_t)(OC_FIFO_READ >> 8),
                              (uint8_t)(OC_FIFO_READ) };
    halDirectReadFifo(cmd, 2, data, len);
}

// ============================================================
//  LoRa commands
// ============================================================

// Byte2 = (SF<<4)|BW   Byte3 = (CR<<4)|PPM
void LR2021::cmdLoRaSetModParams(lr2021_lora_sf_t sf, lr2021_lora_bw_t bw,
                                  lr2021_lora_cr_t cr, uint8_t ppm) {
    const uint8_t cmd[4] = { (uint8_t)(OC_LORA_SET_MOD >> 8),
                              (uint8_t)(OC_LORA_SET_MOD),
                              (uint8_t)(((uint8_t)sf << 4) | ((uint8_t)bw & 0x0F)),
                              (uint8_t)(((uint8_t)cr << 4) | (ppm & 0x0F)) };
    halWrite(cmd, 4);
}

// Byte2-3: preamble (BE)  Byte4: payload_len  Byte5: (pkt_mode<<2)|(crc<<1)|iq
void LR2021::cmdLoRaSetPktParams(uint16_t preamble_sym, bool implicit,
                                  uint8_t payload_len, bool crc, bool iq_inv) {
    const uint8_t cmd[6] = { (uint8_t)(OC_LORA_SET_PKT >> 8),
                              (uint8_t)(OC_LORA_SET_PKT),
                              (uint8_t)(preamble_sym >> 8),
                              (uint8_t)(preamble_sym),
                              payload_len,
                              (uint8_t)(((implicit ? 1U : 0U) << 2)
                                      | ((crc     ? 1U : 0U) << 1)
                                      | ((iq_inv  ? 1U : 0U))) };
    halWrite(cmd, 6);
}

void LR2021::cmdLoRaSetSyncword(uint8_t syncword) {
    const uint8_t cmd[3] = { (uint8_t)(OC_LORA_SET_SYNCWORD >> 8),
                              (uint8_t)(OC_LORA_SET_SYNCWORD),
                              syncword };
    halWrite(cmd, 3);
}

// Response: [0] crc(b4)|cr(b3:0)  [1] len  [2] snr  [3] rssi_pkt  [4] rssi_sig  [5] flags
uint8_t LR2021::cmdLoRaGetPktStatus(int16_t& rssi, int8_t& snr, bool& crc_ok) {
    const uint8_t cmd[2] = { (uint8_t)(OC_LORA_GET_PKT_STAT >> 8),
                              (uint8_t)(OC_LORA_GET_PKT_STAT) };
    uint8_t resp[6] = {0};
    halRead(cmd, 2, resp, 6);
    crc_ok = ((resp[0] >> 4) & 0x01) == 0;  // bit4=1 means CRC error
    snr    = (int8_t)resp[2];
    rssi   = -(int16_t)resp[3];
    return resp[1];
}

// ============================================================
//  FLRC commands
// ============================================================

// Byte2 = br_bw   Byte3 = (CR<<4)|shape
void LR2021::cmdFLRCSetModParams(lr2021_flrc_br_bw_t br_bw,
                                  lr2021_flrc_cr_t cr, lr2021_flrc_shape_t shape) {
    const uint8_t cmd[4] = { (uint8_t)(OC_FLRC_SET_MOD >> 8),
                              (uint8_t)(OC_FLRC_SET_MOD),
                              (uint8_t)br_bw,
                              (uint8_t)(((uint8_t)cr << 4) | (uint8_t)shape) };
    halWrite(cmd, 4);
}

// Uses stored FLRC config constants; updates payload length before each TX/RX.
// Byte2: sync_word_len|preamble_len_enum<<2
// Byte3: crc_type|(header_type<<2)|(match_sw<<3)|(tx_sw<<6)
// Byte4-5: payload_len (BE, variable-length mode uses 0)
void LR2021::cmdFLRCSetPktParams(uint8_t payload_len) {
    uint8_t crc_type = _flrc_cfg.crc ? LR2021_FLRC_CRC_2B : 0x00U;
    const uint8_t cmd[6] = { (uint8_t)(OC_FLRC_SET_PKT >> 8),
                              (uint8_t)(OC_FLRC_SET_PKT),
                              (uint8_t)(LR2021_FLRC_SYNC_WORD_LEN_4B
                                      + (LR2021_FLRC_PREAMBLE_32B << 2)),
                              (uint8_t)(crc_type
                                      + (LR2021_FLRC_HEADER_VARIABLE << 2)
                                      + (LR2021_FLRC_MATCH_SYNCWORD_1 << 3)
                                      + (LR2021_FLRC_TX_SYNCWORD_1 << 6)),
                              0x00,          // payload_len high byte
                              payload_len }; // payload_len low byte
    halWrite(cmd, 6);
}

// Set syncword at index 1 (first of 3 available FLRC syncwords), 4 bytes.
void LR2021::cmdFLRCSetSyncword(const uint8_t syncword[4]) {
    const uint8_t cmd[3] = { (uint8_t)(OC_FLRC_SET_SYNCWORD >> 8),
                              (uint8_t)(OC_FLRC_SET_SYNCWORD),
                              0x01 };  // syncword index 1
    halWrite(cmd, 3, syncword, 4);
}

// Response: [0-1] pkt_len  [2] rssi_avg  [3] rssi_sync  [4] flags
uint16_t LR2021::cmdFLRCGetPktStatus(int16_t& rssi, bool& sync_ok) {
    const uint8_t cmd[2] = { (uint8_t)(OC_FLRC_GET_PKT_STAT >> 8),
                              (uint8_t)(OC_FLRC_GET_PKT_STAT) };
    uint8_t resp[5] = {0};
    halRead(cmd, 2, resp, 5);
    rssi    = -(int16_t)resp[2];
    uint16_t pkt_len = (uint16_t)((uint16_t)resp[0] << 8) | resp[1];
    sync_ok = ((resp[4] >> 4) & 0x0F) != 0;  // syncword_index != 0 → SW detected
    return pkt_len;
}

// ============================================================
//  Public API
// ============================================================

bool LR2021::begin(uint32_t spi_freq) {
    _spi_freq = spi_freq;

    pinMode(_pin_nss,   OUTPUT); digitalWrite(_pin_nss,   HIGH);
    pinMode(_pin_reset, OUTPUT); digitalWrite(_pin_reset, HIGH);
    pinMode(_pin_busy,  INPUT);
    pinMode(_pin_dio,   INPUT);

    _spi->begin();

    reset();

    // Full calibration (all blocks)
    cmdCalibrate(0x6F);

    cmdSetStandby(LR2021_STANDBY_XOSC);

    return true;
}

void LR2021::reset() {
    digitalWrite(_pin_reset, LOW);
    delay(10);
    digitalWrite(_pin_reset, HIGH);
    delay(10);
    waitBusy(500);
}

void LR2021::standby(lr2021_standby_mode_t mode) {
    cmdSetStandby(mode);
}

void LR2021::sleep() {
    // Sleep with no RAM retention and no clock; wake via NSS or reset
    const uint8_t cmd[7] = { (uint8_t)(OC_SET_SLEEP >> 8), (uint8_t)(OC_SET_SLEEP),
                              0x00, 0, 0, 0, 0 };
    halWrite(cmd, 7);
}

int16_t LR2021::getRSSI() {
    const uint8_t cmd[2] = { (uint8_t)(OC_GET_RSSI_INST >> 8),
                              (uint8_t)(OC_GET_RSSI_INST) };
    uint8_t resp[2] = {0};
    halRead(cmd, 2, resp, 2);
    return -(int16_t)resp[0];
}

uint32_t LR2021::getIRQStatus() {
    return cmdGetAndClearIrq();
}

void LR2021::clearIRQ(uint32_t mask) {
    cmdClearIrq(mask);
}

// ---- LoRa configuration ----

bool LR2021::configLoRa(const lr2021_lora_config_t& cfg) {
    _lora_mode = true;
    _lora_cfg  = cfg;

    cmdSetStandby(LR2021_STANDBY_XOSC);
    cmdSetPacketType(LR2021_PKT_TYPE_LORA);
    cmdSetRfFreq(cfg.freq_hz);
    cmdCalibrateFrontEnd(cfg.freq_hz);

    // LF PA for sub-GHz; duty_cycle=4, max slices=7
    cmdSetRxPath(0, 0);
    cmdSetPaConfig(LR2021_PA_SEL_LF, 0x00, 4, 7, 0);
    cmdSetTxParams(cfg.tx_power_dbm, LR2021_RAMP_16_US);

    cmdLoRaSetModParams(cfg.sf, cfg.bw, cfg.cr, 0);
    cmdLoRaSetPktParams(cfg.preamble_symbols, cfg.implicit_hdr,
                        cfg.implicit_hdr ? cfg.payload_len : 255,
                        cfg.crc, cfg.iq_inverted);
    cmdLoRaSetSyncword(cfg.syncword);

    // Route TX_DONE, RX_DONE, TIMEOUT, CRC_ERROR to DIO5
    cmdSetDioFunction(DIO5, DIO_FUNC_IRQ, DIO_DRIVE_NONE);
    cmdSetDioIrq(DIO5, LR2021_IRQ_TX_DONE | LR2021_IRQ_RX_DONE
                      | LR2021_IRQ_TIMEOUT | LR2021_IRQ_CRC_ERROR
                      | LR2021_IRQ_LEN_ERROR);
    cmdClearIrq(LR2021_IRQ_ALL);
    return true;
}

// ---- FLRC configuration ----

bool LR2021::configFLRC(const lr2021_flrc_config_t& cfg) {
    _lora_mode = false;
    _flrc_cfg  = cfg;

    cmdSetStandby(LR2021_STANDBY_XOSC);
    cmdSetPacketType(LR2021_PKT_TYPE_FLRC);
    cmdSetRfFreq(cfg.freq_hz);
    cmdCalibrateFrontEnd(cfg.freq_hz);

    cmdSetRxPath(0, 0);
    cmdSetPaConfig(LR2021_PA_SEL_LF, 0x00, 4, 7, 0);
    cmdSetTxParams(cfg.tx_power_dbm, LR2021_RAMP_16_US);

    cmdFLRCSetModParams(cfg.br_bw, cfg.cr, cfg.shape);
    cmdFLRCSetSyncword(cfg.syncword);
    cmdFLRCSetPktParams(255);  // RX maximum; payload_len updated per TX

    cmdSetDioFunction(DIO5, DIO_FUNC_IRQ, DIO_DRIVE_NONE);
    cmdSetDioIrq(DIO5, LR2021_IRQ_TX_DONE | LR2021_IRQ_RX_DONE
                      | LR2021_IRQ_TIMEOUT | LR2021_IRQ_CRC_ERROR
                      | LR2021_IRQ_LEN_ERROR | LR2021_IRQ_CMD_ERROR
                      | LR2021_IRQ_ERROR);
    cmdClearRxFifo();
    cmdClearErrors();
    cmdClearIrq(LR2021_IRQ_ALL);
    return true;
}

// ---- Blocking transmit ----

bool LR2021::transmit(const uint8_t* data, uint8_t len, uint32_t timeout_ms) {
    cmdSetStandby(LR2021_STANDBY_RC);
    cmdClearIrq(LR2021_IRQ_ALL);

    // Update payload length in packet params
    if (_lora_mode) {
        cmdLoRaSetPktParams(_lora_cfg.preamble_symbols, _lora_cfg.implicit_hdr,
                            len, _lora_cfg.crc, _lora_cfg.iq_inverted);
    } else {
        cmdFLRCSetPktParams(len);
    }

    cmdWriteFifo(data, len);

    uint32_t rtc = (timeout_ms > 0) ? LR2021_MS_TO_RTC(timeout_ms) : 0;
    cmdSetTx(rtc);

    uint32_t t0 = millis();
    while ((millis() - t0) < (timeout_ms + 200UL)) {
        if (digitalRead(_pin_dio) == HIGH) {
            uint32_t irq = cmdGetAndClearIrq();
            if (irq & LR2021_IRQ_TX_DONE)  return true;
            if (irq & LR2021_IRQ_TIMEOUT)   return false;
            if (irq & LR2021_IRQ_ERROR)     return false;
        }
        delayMicroseconds(100);
    }
    cmdClearIrq(LR2021_IRQ_ALL);
    cmdSetStandby(LR2021_STANDBY_RC);
    return false;
}

// ---- Blocking receive ----

int LR2021::receive(uint8_t* buf, uint8_t max_len,
                    lr2021_rx_status_t* status, uint32_t timeout_ms) {
    if (status) {
        status->rssi_dbm  = 0;
        status->snr       = 0;
        status->irq_flags = LR2021_IRQ_NONE;
        status->system_errors = 0;
        status->crc_ok    = false;
        status->flrc_sync_ok = false;
        status->length    = 0;
    }

    cmdSetStandby(LR2021_STANDBY_RC);
    cmdClearRxFifo();
    cmdClearErrors();
    cmdClearIrq(LR2021_IRQ_ALL);
    if (!_lora_mode) cmdFLRCSetPktParams(max_len);

    uint32_t rtc = (timeout_ms == 0) ? LR2021_RX_CONTINUOUS
                                     : LR2021_MS_TO_RTC(timeout_ms);
    cmdSetRx(rtc);

    // Bound the polling loop; for continuous mode cap at 60 s
    uint32_t t0   = millis();
    uint32_t wait = (timeout_ms == 0) ? 60000UL : (timeout_ms + 200UL);

    while ((millis() - t0) < wait) {
        if (digitalRead(_pin_dio) == HIGH) {
            uint32_t irq = cmdGetAndClearIrq();
            if (status) status->irq_flags = irq;

            if (irq & LR2021_IRQ_TIMEOUT) {
                if (status) status->rssi_dbm = getRSSI();
                cmdClearRxFifo();
                cmdSetStandby(LR2021_STANDBY_RC);
                return -1;
            }

            if (irq & (LR2021_IRQ_CMD_ERROR | LR2021_IRQ_ERROR)) {
                if (status) {
                    status->rssi_dbm = getRSSI();
                    status->system_errors = cmdGetErrors();
                }
                cmdClearErrors();
                cmdClearRxFifo();
                cmdSetStandby(LR2021_STANDBY_RC);
                return -1;
            }

            // A noisy channel can raise CRC/LEN errors repeatedly. Keep the
            // receiver armed until the caller's timeout instead of returning
            // immediately and flooding the application loop with failures.
            if (irq & (LR2021_IRQ_CRC_ERROR | LR2021_IRQ_LEN_ERROR)) {
                if (status) status->rssi_dbm = getRSSI();
                cmdSetStandby(LR2021_STANDBY_RC);
                cmdClearRxFifo();
                cmdClearIrq(LR2021_IRQ_ALL);
                cmdSetRx(rtc);
                continue;
            }

            if (irq & LR2021_IRQ_RX_DONE) {
                uint16_t pkt_len = cmdGetRxPacketLen();
                int16_t lora_rssi = 0;
                int8_t  lora_snr = 0;
                bool    lora_crc_ok = true;
                int16_t flrc_rssi = 0;
                bool    flrc_sync_ok = false;

                if (_lora_mode) {
                    uint8_t lora_pkt_len = cmdLoRaGetPktStatus(lora_rssi, lora_snr, lora_crc_ok);
                    if (pkt_len == 0) pkt_len = lora_pkt_len;
                } else {
                    uint16_t flrc_pkt_len = cmdFLRCGetPktStatus(flrc_rssi, flrc_sync_ok);
                    if (pkt_len == 0) pkt_len = flrc_pkt_len;
                }

                if ((pkt_len == 0) && (irq & LR2021_IRQ_FIFO_RX)) {
                    pkt_len = cmdGetRxFifoLevel();
                }
                if ((pkt_len == 0) || (!_lora_mode && !flrc_sync_ok)) {
                    if (status) {
                        status->length  = pkt_len;
                        status->crc_ok  = false;
                        status->flrc_sync_ok = flrc_sync_ok;
                        status->rssi_dbm = flrc_rssi;
                    }
                    cmdSetStandby(LR2021_STANDBY_RC);
                    cmdClearRxFifo();
                    cmdClearIrq(LR2021_IRQ_ALL);
                    cmdSetRx(rtc);
                    continue;
                }
                if (pkt_len > max_len) pkt_len = max_len;
                cmdReadFifo(buf, pkt_len);

                if (status) {
                    status->irq_flags = irq;
                    status->length    = pkt_len;
                    status->crc_ok    = !(irq & LR2021_IRQ_CRC_ERROR);

                    if (_lora_mode) {
                        status->rssi_dbm = lora_rssi;
                        status->snr      = lora_snr;
                        status->crc_ok   = lora_crc_ok;
                    } else {
                        status->rssi_dbm = flrc_rssi;
                        status->snr    = 0;
                        status->flrc_sync_ok = flrc_sync_ok;
                        status->crc_ok = status->crc_ok && flrc_sync_ok;
                    }
                }

                cmdSetStandby(LR2021_STANDBY_RC);
                return (int)pkt_len;
            }
        }
        delayMicroseconds(100);
    }

    cmdClearIrq(LR2021_IRQ_ALL);
    cmdClearRxFifo();
    cmdSetStandby(LR2021_STANDBY_RC);
    return -1;
}

// ---- Non-blocking async receive ----

void LR2021::startRx(uint32_t timeout_ms) {
    cmdClearRxFifo();
    cmdClearIrq(LR2021_IRQ_ALL);
    if (!_lora_mode) cmdFLRCSetPktParams(255);
    uint32_t rtc = (timeout_ms == 0) ? LR2021_RX_CONTINUOUS
                                     : LR2021_MS_TO_RTC(timeout_ms);
    cmdSetRx(rtc);
}

bool LR2021::dataReady() {
    return digitalRead(_pin_dio) == HIGH;
}

int LR2021::readPacket(uint8_t* buf, uint8_t max_len, lr2021_rx_status_t* status) {
    uint32_t irq = cmdGetAndClearIrq();
    if (!(irq & LR2021_IRQ_RX_DONE)) return -1;

    uint16_t pkt_len = cmdGetRxPacketLen();
    int16_t lora_rssi = 0;
    int8_t  lora_snr = 0;
    bool    lora_crc_ok = true;
    int16_t flrc_rssi = 0;
    bool    flrc_sync_ok = false;

    if (_lora_mode) {
        uint8_t lora_pkt_len = cmdLoRaGetPktStatus(lora_rssi, lora_snr, lora_crc_ok);
        if (pkt_len == 0) pkt_len = lora_pkt_len;
    } else {
        uint16_t flrc_pkt_len = cmdFLRCGetPktStatus(flrc_rssi, flrc_sync_ok);
        if (pkt_len == 0) pkt_len = flrc_pkt_len;
    }

    if ((pkt_len == 0) && (irq & LR2021_IRQ_FIFO_RX)) {
        pkt_len = cmdGetRxFifoLevel();
    }
    if (pkt_len > max_len) pkt_len = max_len;
    cmdReadFifo(buf, pkt_len);

    if (status) {
        status->irq_flags = irq;
        status->length    = pkt_len;
        status->crc_ok    = !(irq & LR2021_IRQ_CRC_ERROR);

        if (_lora_mode) {
            status->rssi_dbm = lora_rssi;
            status->snr      = lora_snr;
            status->crc_ok   = lora_crc_ok;
        } else {
            status->rssi_dbm = flrc_rssi;
            status->snr      = 0;
            status->flrc_sync_ok = flrc_sync_ok;
            status->crc_ok   = status->crc_ok && flrc_sync_ok;
        }
    }

    cmdSetStandby(LR2021_STANDBY_RC);
    return (int)pkt_len;
}

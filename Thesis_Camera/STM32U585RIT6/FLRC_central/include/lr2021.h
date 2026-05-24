#pragma once

#include <Arduino.h>
#include <SPI.h>

// ---- Default pins for Seeed XIAO RP2040 ----
// SPI0: SCK=D8(GPIO2), MISO=D9(GPIO4), MOSI=D10(GPIO3) — use board defaults
#define LR2021_DEFAULT_PIN_NSS    27   // D1
#define LR2021_DEFAULT_PIN_RESET  28   // D2
#define LR2021_DEFAULT_PIN_BUSY   29   // D3
#define LR2021_DEFAULT_PIN_DIO    26   // D0 → connect to LR2021 DIO5 pin

// ---- Timing ----
#define LR2021_RTC_FREQ_HZ        32768UL
#define LR2021_MS_TO_RTC(ms)      ((uint32_t)((uint64_t)(ms) * LR2021_RTC_FREQ_HZ / 1000ULL))
#define LR2021_RX_CONTINUOUS      0xFFFFFFUL

// ---- LoRa syncwords ----
#define LR2021_LORA_SYNCWORD_PUBLIC   0x34U
#define LR2021_LORA_SYNCWORD_PRIVATE  0x12U

// ---- IRQ bit flags (from lr20xx_system_types.h) ----
#define LR2021_IRQ_NONE                    0UL
#define LR2021_IRQ_FIFO_RX                 (1UL << 0)
#define LR2021_IRQ_FIFO_TX                 (1UL << 1)
#define LR2021_IRQ_PREAMBLE_DETECTED       (1UL << 5)
#define LR2021_IRQ_SYNC_WORD_HEADER_VALID  (1UL << 6)
#define LR2021_IRQ_LORA_HEADER_ERROR       (1UL << 9)
#define LR2021_IRQ_ERROR                   (1UL << 16)
#define LR2021_IRQ_CMD_ERROR               (1UL << 17)
#define LR2021_IRQ_RX_DONE                 (1UL << 18)
#define LR2021_IRQ_TX_DONE                 (1UL << 19)
#define LR2021_IRQ_TIMEOUT                 (1UL << 21)
#define LR2021_IRQ_CRC_ERROR               (1UL << 22)
#define LR2021_IRQ_LEN_ERROR               (1UL << 23)
#define LR2021_IRQ_ALL                     0xFFFFFFFFUL

// ---- Standby modes ----
typedef enum {
    LR2021_STANDBY_RC   = 0x00,
    LR2021_STANDBY_XOSC = 0x01,
} lr2021_standby_mode_t;

// ---- Packet types (from lr20xx_radio_common_types.h) ----
typedef enum {
    LR2021_PKT_TYPE_LORA = 0x00,
    LR2021_PKT_TYPE_FLRC = 0x05,
} lr2021_pkt_type_t;

// ---- LoRa spreading factor ----
typedef enum {
    LR2021_LORA_SF5  = 0x05,
    LR2021_LORA_SF6  = 0x06,
    LR2021_LORA_SF7  = 0x07,
    LR2021_LORA_SF8  = 0x08,
    LR2021_LORA_SF9  = 0x09,
    LR2021_LORA_SF10 = 0x0A,
    LR2021_LORA_SF11 = 0x0B,
    LR2021_LORA_SF12 = 0x0C,
} lr2021_lora_sf_t;

// ---- LoRa bandwidth ----
// BW field is lower nibble of mod_params byte 0: (SF<<4)|BW
typedef enum {
    LR2021_LORA_BW_031_KHZ  = 0x02,
    LR2021_LORA_BW_062_KHZ  = 0x03,
    LR2021_LORA_BW_125_KHZ  = 0x04,
    LR2021_LORA_BW_250_KHZ  = 0x05,
    LR2021_LORA_BW_500_KHZ  = 0x06,
    LR2021_LORA_BW_1000_KHZ = 0x07,
} lr2021_lora_bw_t;

// ---- LoRa coding rate ----
// CR field is upper nibble of mod_params byte 1: (CR<<4)|PPM
typedef enum {
    LR2021_LORA_CR_NO_CR = 0x00,
    LR2021_LORA_CR_4_5   = 0x01,
    LR2021_LORA_CR_4_6   = 0x02,
    LR2021_LORA_CR_4_7   = 0x03,
    LR2021_LORA_CR_4_8   = 0x04,
} lr2021_lora_cr_t;

// ---- FLRC bitrate / bandwidth pairs (Table 18-1, DS.LR2021 Rev1.1) ----
typedef enum {
    LR2021_FLRC_BR_2600_BW_2666 = 0x00,  // 2.600 Mb/s, 2.666 MHz
    LR2021_FLRC_BR_2080_BW_2222 = 0x01,  // 2.080 Mb/s, 2.222 MHz
    LR2021_FLRC_BR_1300_BW_1333 = 0x02,  // 1.300 Mb/s, 1.333 MHz
    LR2021_FLRC_BR_1040_BW_1333 = 0x03,  // 1.040 Mb/s, 1.333 MHz
    LR2021_FLRC_BR_0650_BW_0888 = 0x04,  // 0.650 Mb/s, 0.888 MHz
    LR2021_FLRC_BR_0520_BW_0769 = 0x05,  // 0.520 Mb/s, 0.769 MHz
    LR2021_FLRC_BR_0325_BW_0444 = 0x06,  // 0.325 Mb/s, 0.444 MHz
    LR2021_FLRC_BR_0260_BW_0444 = 0x07,  // 0.260 Mb/s, 0.444 MHz
} lr2021_flrc_br_bw_t;

// ---- FLRC coding rate ----
typedef enum {
    LR2021_FLRC_CR_1_2  = 0x00,
    LR2021_FLRC_CR_3_4  = 0x01,
    LR2021_FLRC_CR_NONE = 0x02,
    LR2021_FLRC_CR_2_3  = 0x03,
} lr2021_flrc_cr_t;

// ---- FLRC Gaussian pulse shaping ----
typedef enum {
    LR2021_FLRC_SHAPE_OFF  = 0x00,
    LR2021_FLRC_SHAPE_BT05 = 0x05,
    LR2021_FLRC_SHAPE_BT1  = 0x07,
} lr2021_flrc_shape_t;

// ---- PA selection ----
typedef enum {
    LR2021_PA_SEL_LF = 0x00,  // sub-GHz (868 / 915 MHz)
    LR2021_PA_SEL_HF = 0x01,  // 2.4 GHz
} lr2021_pa_sel_t;

// ---- TX ramp time ----
typedef enum {
    LR2021_RAMP_2_US   = 0x00,
    LR2021_RAMP_4_US   = 0x01,
    LR2021_RAMP_8_US   = 0x02,
    LR2021_RAMP_16_US  = 0x03,
    LR2021_RAMP_32_US  = 0x04,
    LR2021_RAMP_48_US  = 0x05,
    LR2021_RAMP_64_US  = 0x06,
    LR2021_RAMP_128_US = 0x0A,
    LR2021_RAMP_304_US = 0x12,
} lr2021_ramp_time_t;

// ---- Configuration structs ----
struct lr2021_lora_config_t {
    uint32_t         freq_hz;           // e.g. 915000000
    lr2021_lora_sf_t sf;                // spreading factor
    lr2021_lora_bw_t bw;               // bandwidth
    lr2021_lora_cr_t cr;               // coding rate
    uint16_t         preamble_symbols; // typically 8–16
    uint8_t          syncword;         // 0x34=public LoRaWAN, 0x12=private
    bool             crc;              // enable CRC
    bool             iq_inverted;      // invert IQ (for downlink in LoRaWAN)
    bool             implicit_hdr;     // fixed payload length
    int8_t           tx_power_dbm;     // output power in dBm (e.g. 14)
};

struct lr2021_flrc_config_t {
    uint32_t            freq_hz;
    lr2021_flrc_br_bw_t br_bw;         // bitrate + bandwidth pair
    lr2021_flrc_cr_t    cr;            // coding rate
    lr2021_flrc_shape_t shape;         // Gaussian pulse shaping
    uint8_t             syncword[4];   // 4-byte syncword (syncword index 1)
    bool                crc;           // enable CRC
    int8_t              tx_power_dbm;
};

struct lr2021_rx_status_t {
    int16_t  rssi_dbm;
    int8_t   snr;          // LoRa only; 0 for FLRC
    uint32_t irq_flags;
    bool     crc_ok;
    uint16_t length;
};

// ================================================================
//  LR2021 driver class
// ================================================================
class LR2021 {
public:
    LR2021(int pin_nss   = LR2021_DEFAULT_PIN_NSS,
           int pin_reset = LR2021_DEFAULT_PIN_RESET,
           int pin_busy  = LR2021_DEFAULT_PIN_BUSY,
           int pin_dio   = LR2021_DEFAULT_PIN_DIO,
           SPIClass* spi = &SPI);

    // Initialise GPIO, SPI, reset the chip, run calibration
    bool begin(uint32_t spi_freq = 8000000UL);

    // Hardware reset (chip goes to standby-RC after reset)
    void reset();

    // Configure LoRa / FLRC mode (call once; subsequent TX/RX use stored config)
    bool configLoRa(const lr2021_lora_config_t& cfg);
    bool configFLRC(const lr2021_flrc_config_t& cfg);

    // Blocking transmit — returns true on TX_DONE, false on timeout / error
    bool transmit(const uint8_t* data, uint8_t len, uint32_t timeout_ms = 5000);

    // Blocking receive — returns byte count on success, -1 on timeout / CRC error
    // timeout_ms = 0 → continuous receive (blocks until a packet arrives)
    int receive(uint8_t* buf, uint8_t max_len,
                lr2021_rx_status_t* status = nullptr,
                uint32_t timeout_ms = 0);

    // Non-blocking async receive
    void startRx(uint32_t timeout_ms = 0);
    bool dataReady();   // true when DIO pin is asserted
    int  readPacket(uint8_t* buf, uint8_t max_len,
                    lr2021_rx_status_t* status = nullptr);

    // Utility
    void     standby(lr2021_standby_mode_t mode = LR2021_STANDBY_RC);
    void     sleep();
    int16_t  getRSSI();
    uint32_t getIRQStatus();
    void     clearIRQ(uint32_t mask = LR2021_IRQ_ALL);
    bool     isBusy() { return digitalRead(_pin_busy) == HIGH; }

private:
    int       _pin_nss, _pin_reset, _pin_busy, _pin_dio;
    SPIClass* _spi;
    uint32_t  _spi_freq;
    bool      _lora_mode = true;

    lr2021_lora_config_t _lora_cfg = {};
    lr2021_flrc_config_t _flrc_cfg = {};

    // ---- SPI HAL (matches lr20xx_hal_* conventions) ----
    // Write:  NSS↓, send cmd[], send data[], NSS↑
    void halWrite(const uint8_t* cmd, uint8_t cmdLen,
                  const uint8_t* data = nullptr, uint16_t dataLen = 0);
    // Read:   NSS↓, send cmd[], receive data[], NSS↑
    void halRead(const uint8_t* cmd, uint8_t cmdLen,
                 uint8_t* data, uint16_t dataLen);
    // DirectRead: NSS↓, receive data[] (no command — for GetStatus)
    void halDirectRead(uint8_t* data, uint8_t len);
    // DirectReadFifo: NSS↓, send cmd[], receive data[] (FIFO variant)
    void halDirectReadFifo(const uint8_t* cmd, uint8_t cmdLen,
                           uint8_t* data, uint16_t dataLen);

    void waitBusy(uint32_t timeout_ms = 200);

    // ---- Radio commands (inline wrappers around halWrite/halRead) ----
    void cmdSetStandby(lr2021_standby_mode_t mode);
    void cmdSetPacketType(lr2021_pkt_type_t type);
    void cmdSetRfFreq(uint32_t freq_hz);
    void cmdSetPaConfig(lr2021_pa_sel_t pa_sel, uint8_t lf_mode,
                        uint8_t lf_duty, uint8_t lf_slices, uint8_t hf_duty);
    void cmdSetTxParams(int8_t power_dbm, lr2021_ramp_time_t ramp);
    void cmdCalibrate(uint8_t mask);
    void cmdSetDioIrq(uint8_t dio, uint32_t irq_mask);
    void cmdClearIrq(uint32_t mask);
    uint32_t cmdGetAndClearIrq();
    void cmdSetTx(uint32_t timeout_rtc);
    void cmdSetRx(uint32_t timeout_rtc);
    uint16_t cmdGetRxPacketLen();
    void cmdWriteFifo(const uint8_t* data, uint16_t len);
    void cmdReadFifo(uint8_t* data, uint16_t len);

    // LoRa
    void cmdLoRaSetModParams(lr2021_lora_sf_t sf, lr2021_lora_bw_t bw,
                             lr2021_lora_cr_t cr, uint8_t ppm = 0);
    void cmdLoRaSetPktParams(uint16_t preamble_sym, bool implicit,
                             uint8_t payload_len, bool crc, bool iq_inv);
    void cmdLoRaSetSyncword(uint8_t syncword);
    void cmdLoRaGetPktStatus(int16_t& rssi, int8_t& snr, bool& crc_ok);

    // FLRC
    void cmdFLRCSetModParams(lr2021_flrc_br_bw_t br_bw,
                             lr2021_flrc_cr_t cr, lr2021_flrc_shape_t shape);
    void cmdFLRCSetPktParams(uint8_t payload_len);
    void cmdFLRCSetSyncword(const uint8_t syncword[4]);
    void cmdFLRCGetPktStatus(int16_t& rssi, bool& sync_ok);
};

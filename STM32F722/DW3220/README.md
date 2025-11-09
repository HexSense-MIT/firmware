# DW3220_PDoA_ref - UWB Ranging Driver

## Overview

This is a **UWB (Ultra-Wideband) ranging driver** for the DW3220/DW3000 chip running on a PIC16F1508 microcontroller. It implements **PDoA (Phase Difference of Arrival)** positioning using single-sided or double-sided ranging protocols.

## System Architecture

The code supports three board configurations:
- **ROAMER** (Board 0): Mobile device that responds to ranging requests
- **BASE** (Board 1): Primary anchor that initiates ranging and calculates PDoA
- **BASE2** (Board 2): Secondary anchor (experimental, for phase delay measurement)

## Hardware Configuration

### GPIO Pin Mapping
- **LED**: RA2 (Status indicator)
- **SYNC**: RA5 (Synchronization signal)
- **CS**: RA4 (SPI Chip Select)
- **MISO**: RC5 (SPI Data In)
- **MOSI**: RC4 (SPI Data Out)
- **CLK**: RC3 (SPI Clock)
- **RESET**: RC6 (DW3220 Reset)
- **WAKEUP**: RC7 (DW3220 Wakeup)

### Communication
- **SPI**: Bit-banged software SPI implementation
- **UART**: 115200 baud, TX only (debug and data output)
- **System Clock**: 16 MHz internal oscillator
- **Tick Timer**: ~977 Hz (Timer0)

## Radio Configuration

### Default Settings
- **Channel**: 9 (8 GHz band)
- **Preamble Length**: 128 symbols
- **Preamble Code**: 9
- **PAC**: 8
- **Data Rate**: 6.8 Mbps
- **PHR Mode**: Standard
- **PHR Rate**: 850 kbps
- **STS Length**: 128 symbols (for PDoA)

### PDoA Configuration
When `ENABLE_PDOA` is defined:
- **STS Mode**: Fixed code, positioned after data packet
- **STS Length**: 128 symbols
- **STS Threshold**: 12 (STS_MNTH)
- **Phase Offset**: 3600 (adjustable for readability)

## Detailed Flowchart

```
┌─────────────────────────────────────────────────────────────────┐
│                        SYSTEM STARTUP                            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ main() - Lines 1180-1221                                         │
│ ┌─────────────────────────────────────────────────────────────┐ │
│ │ 1. Configure 16MHz internal oscillator (OSCCON = 0x78)     │ │
│ │ 2. Setup UART @ 115200 baud (BAUD_CODE=35)                 │ │
│ │ 3. Configure Timer0 for tick generation (~977 Hz)          │ │
│ │ 4. Set all I/O to digital mode (ANSELA/B/C = 0)            │ │
│ └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    init_radio() - Lines 877-1178                 │
│─────────────────────────────────────────────────────────────────│
│ STEP 1: Hardware Reset (880-884)                                 │
│   • RESET_LAT = 0 (pull low)                                    │
│   • delay(1)                                                     │
│   • RESET_TRIS = 1 (release, pulled high externally)            │
│   • delay(2)                                                     │
│─────────────────────────────────────────────────────────────────│
│ STEP 2: Configure SPI GPIO (886-896)                             │
│   • CS_LAT = 1 (chip select high/inactive)                     │
│   • CLK_LAT = 0 (clock idle low)                                │
│   • MOSI_LAT = 0 (data out low)                                 │
│   • WAKEUP_LAT = 0 (wakeup low)                                 │
│   • All set as outputs (TRIS = 0)                               │
│─────────────────────────────────────────────────────────────────│
│ STEP 3: Read OTP Calibration Values (907-921)                   │
│   • readOTP(0x04) → ldo_low                                     │
│   • readOTP(0x05) → ldo_high                                    │
│   • readOTP(0x0A) → bias_tune (bits 20:16)                      │
│   • If valid (!= 0):                                            │
│       - write(0x11, 0x1F, bias_tune)  // PMSC bias              │
│       - write(0x0B, 0x08, 0x0100)     // OTP enable             │
│─────────────────────────────────────────────────────────────────│
│ STEP 4: Crystal Trim (923-929)                                   │
│   • readOTP(0x1E) → xtrim_value                                 │
│   • If xtrim_value == 0: xtrim_value = 0x2E (default)           │
│   • write(FS_CTRL_REG, XTAL_TRIM, xtrim_value)                  │
│     Register: 0x09:0x14                                          │
│─────────────────────────────────────────────────────────────────│
│ STEP 5: System Configuration (932-944)                           │
│   • Build usr_cfg from:                                          │
│     - STDRD_SYS_CONFIG (0x188)                                  │
│     - PHR_MODE (0x0 = standard)                                 │
│     - PHR_RATE (0x0 = 850kB)                                    │
│   • IF ENABLE_PDOA && BOARD==BASE:                              │
│     - usr_cfg |= (3 << 16)  // Enable PDOA mode (SYS_CFG:18:17) │
│     - usr_cfg |= (1 << 12)  // CP_SPC: STS after data           │
│     - usr_cfg |= (1 << 15)  // CP_SDC: fixed STS code           │
│   • write(GEN_CFG_AES_LOW, SYS_CFG, usr_cfg)                    │
│     Register: 0x00:0x10                                          │
│─────────────────────────────────────────────────────────────────│
│ STEP 6: OTP & DTUNE Configuration (946-956)                      │
│   • otp_write = 0x1400                                           │
│   • If PREAMBLE_LENGTH >= 256: otp_write |= 0x04                │
│   • write(OTP_IF_REG, 0x08, otp_write)                          │
│     Register: 0x0B:0x08                                          │
│   • write_len(DRX_REG, 0x00, 0x00, 1)  // Reset DTUNE0          │
│   • write(DRX_REG, 0x0, PAC)  // PAC = 0 (PAC8)                 │
│     Register: 0x06:0x00                                          │
│─────────────────────────────────────────────────────────────────│
│ STEP 7: STS Configuration for PDOA (958-973)                     │
│   • IF ENABLE_PDOA:                                              │
│     - Read STS_CFG (0x02:0x00)                                  │
│     - Set STS length = 128 symbols: (sts_cfg & 0xFF00) | 15     │
│       Formula: (128/8 - 1) = 15                                  │
│     - write_len(STS_CFG_REG, STS_CFG, sts_cfg, 2)               │
│     - Read STS_CONF_0 (0x0E:0x12)                               │
│     - Set STS_MNTH = 12: (sts_cfg & 0xFF80FFFF) | (12 << 16)    │
│     - write(CIA_REG3, STS_CONF_0, sts_cfg)                      │
│     - Read CIA_ADJUST (0x0E:0x1A)                               │
│     - Adjust phase offset: (res & 0xC000) | 3600                │
│     - write(CIA_REG3, CIA_ADJUST, adjusted_value)               │
│─────────────────────────────────────────────────────────────────│
│ STEP 8: TX Frame Control (976-1002)                              │
│   • write_len(GEN_CFG_AES_LOW, TX_FCTRL+5, 0x00, 1)             │
│     Register: 0x00:0x29 = 0x00                                   │
│   • write(DRX_REG, 0x0C, 0xAF5F584C)                            │
│     Register: 0x06:0x0C = 0xAF5F584C                             │
│   • Read CHAN_CTRL (0x01:0x14)                                  │
│   • chan_ctrl_val &= ~0x1FFF  // Clear RF config bits           │
│   • chan_ctrl_val |= CHANNEL (0x1 = CH9)                        │
│   • chan_ctrl_val |= (PREAMBLE_CODE << 8)  // Code 9            │
│   • chan_ctrl_val |= (PREAMBLE_CODE << 3)                       │
│   • IF ENABLE_PDOA: chan_ctrl_val |= (3 << 1)  // SFD_TYPE=3    │
│   • write(GEN_CFG_AES_HIGH, CHAN_CTRL, chan_ctrl_val)           │
│   • Read TX_FCTRL (0x00:0x24)                                   │
│   • tx_fctrl_val |= (PREAMBLE_128 << 12)  // 128 preamble       │
│   • tx_fctrl_val |= (DATARATE_6_8MB << 10)  // 6.8 Mbps         │
│   • write(GEN_CFG_AES_LOW, TX_FCTRL, tx_fctrl_val)              │
│   • write(DRX_REG, 0x02, 0x81)                                  │
│     Register: 0x06:0x02 = 0x81                                   │
│─────────────────────────────────────────────────────────────────│
│ STEP 9: RF Configuration (1005-1026)                             │
│   • IF CHANNEL_9:                                                │
│     - rf_tx_ctrl_2 = 0x1C010034                                 │
│     - pll_conf = 0x0F3C                                          │
│   • ELSE (CHANNEL_5):                                            │
│     - rf_tx_ctrl_2 = 0x1C071134                                 │
│     - pll_conf = 0x1F3C                                          │
│   • write(RF_CONF, RF_TX_CTRL_2, rf_tx_ctrl_2)                  │
│     Register: 0x07:0x1C                                          │
│   • write(FS_CTRL_REG, PLL_CFG, pll_conf)                       │
│     Register: 0x09:0x00                                          │
│   • IF CHANNEL_9:                                                │
│     - write(RF_CONF, RX_CTRL_HI, 0x08B5A833)                    │
│       Register: 0x07:0x10                                        │
│   • write(RF_CONF, LDO_RLOAD, 0x14)                             │
│     Register: 0x07:0x51 = 0x14                                   │
│   • write(RF_CONF, RF_TX_CTRL_1, 0x0E)                          │
│     Register: 0x07:0x1A = 0x0E                                   │
│   • write(FS_CTRL_REG, PLL_CAL, 0x81)                           │
│     Register: 0x09:0x08 = 0x81 (extend lock delay)              │
│─────────────────────────────────────────────────────────────────│
│ STEP 10: Clock & PLL Lock (1027-1046)                            │
│   • write(GEN_CFG_AES_LOW, SYS_STATUS, 0x02)                    │
│     Register: 0x00:0x44 = 0x02                                   │
│   • write(PMSC_REG, 0x04, 0x300200)  // Auto clock mode         │
│     Register: 0x11:0x04 = 0x300200                               │
│   • write(PMSC_REG, 0x08, 0x0138)                               │
│     Register: 0x11:0x08 = 0x0138                                 │
│   • Wait for PLL lock (poll DEV_ID bit 1):                      │
│     FOR i = 0 to 99:                                             │
│       IF read(GEN_CFG_AES_LOW, DEV_ID) & 0x2:                   │
│         success = 1, break                                       │
│     Register polled: 0x00:0x00                                   │
│─────────────────────────────────────────────────────────────────│
│ STEP 11: OTP Finalization (1048-1053)                            │
│   • Read otp_val from OTP_IF_REG:0x08 (0x0B:0x08)               │
│   • otp_val |= 0x40                                              │
│   • IF CHANNEL_9: otp_val |= 0x2000                             │
│   • write(OTP_IF_REG, 0x08, otp_val)                            │
│   • write(RX_TUNE_REG, 0x19, 0xF0)                              │
│     Register: 0x03:0x19 = 0xF0                                   │
│─────────────────────────────────────────────────────────────────│
│ STEP 12: RX Calibration (1055-1104)                              │
│   • Read LDO_CTRL (0x07:0x48)                                   │
│   • tmp_ldo = 0x105 | 0x100 | 0x4 | 0x1 = 0x105                 │
│   • write(RF_CONF, LDO_CTRL, 0x105)                             │
│   • write(EXT_SYNC, RX_CAL_RESQ, 0x020000)                      │
│     Register: 0x04:0x1C = 0x020000                               │
│   • read(EXT_SYNC, RX_CAL)  // Dummy read, 0x04:0x0C            │
│   • delay(20)                                                    │
│   • write(EXT_SYNC, RX_CAL, 0x11)  // Start calibration         │
│     Register: 0x04:0x0C = 0x11                                   │
│   • Poll RX_CAL_STS (0x04:0x20) for completion (max 100 tries)  │
│   • write(EXT_SYNC, RX_CAL, 0x00)  // Stop                      │
│   • write(EXT_SYNC, RX_CAL_STS, 0x01)  // Clear status          │
│   • Read RX_CAL_RESI (0x04:0x14) - check if 0x1FFFFFFF (fail)   │
│   • Read RX_CAL_RESQ (0x04:0x1C) - check if 0x1FFFFFFF (fail)   │
│   • write(RF_CONF, LDO_CTRL, ldo_ctrl_val)  // Restore          │
│─────────────────────────────────────────────────────────────────│
│ STEP 13: DGC (Digital Gain Control) Configuration (1105-1137)    │
│   • write(0x0E, 0x02, 0x01)  // Enable CIA diagnostics          │
│   • write(RX_TUNE_REG, 0x1C, 0x10000240)  // DGC_CFG0           │
│   • write(RX_TUNE_REG, 0x20, 0x1B6DA489)  // DGC_CFG1           │
│   • IF CHANNEL_5:                                                │
│     - write(RX_TUNE_REG, 0x38-0x50, CH5 LUT values)             │
│       7 registers: DGC_LUT_0 through DGC_LUT_6                   │
│   • ELSE (CHANNEL_9):                                            │
│     - write(RX_TUNE_REG, 0x38-0x50, CH9 LUT values)             │
│       Different LUT values for 8 GHz band                        │
│   • write(RX_TUNE_REG, 0x18, 0xE5E5)  // THR_64                 │
│     Register: 0x03:0x18 = 0xE5E5                                 │
│─────────────────────────────────────────────────────────────────│
│ STEP 14: Final Configuration (1143-1175)                         │
│   • write(DRX_REG, 0x0, 0x81101C)                               │
│     Register: 0x06:0x00 = 0x81101C                               │
│   • write(RF_CONF, SAR_TEST, 0x4)  // Enable temp sensor        │
│     Register: 0x07:0x34 = 0x4                                    │
│   • write(RF_CONF, LDO_CTRL, 0x14)                              │
│   • write(RF_CONF, RF_TX_CTRL_1, 0x0E)                          │
│   • write(PMSC_REG, 0x04, 0xB40200)                             │
│     Register: 0x11:0x04 = 0xB40200                               │
│   • write(PMSC_REG, 0x08, 0x80030738)                           │
│     Register: 0x11:0x08 = 0x80030738                             │
│   • write(GEN_CFG_AES_LOW, SYS_ENABLE, 0xFFFFFFFF)              │
│     Register: 0x00:0x3C = 0xFFFFFFFF                             │
│   • write(GEN_CFG_AES_LOW, SYS_ENABLE+4, 0xFFFF)                │
│     Register: 0x00:0x40 = 0xFFFF                                 │
│   • write_len(AON_REG, 0x00, 0x000900, 3)                       │
│     Register: 0x0A:0x00 = 0x000900 (auto RX cal, on-wake IDLE)  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       Configure TX Mode                          │
│   CONFIGURE_AS_TX macro (529-531):                               │
│   • write(RF_CONF, RF_TX_CTRL_2, 0x34)  // OG_DELAY             │
│     Register: 0x07:0x1C = 0x34                                   │
│   • write(GEN_CFG_AES_HIGH, TX_POWER, 0xFCFCFCFC)               │
│     Register: 0x01:0x0C = 0xFCFCFCFC                             │
│   clear_system_status() - write(0x00:0x44, 0x3F7FFFFF)          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────┴─────────────────────┐
        │                                             │
        ▼                                             ▼
┌──────────────────────┐                  ┌──────────────────────┐
│   ROAMER BOARD       │                  │    BASE BOARD        │
│   (Lines 1236-1318)  │                  │   (Lines 1322-1481)  │
└──────────────────────┘                  └──────────────────────┘
        │                                             │
        ▼                                             ▼
┌──────────────────────────────────────┐   ┌──────────────────────────────────┐
│  Single-Sided Ranging (SSR)          │   │  Single-Sided Ranging (SSR)      │
│  ROAMER Loop (ENABLE_DS not defined) │   │  BASE Loop                        │
└──────────────────────────────────────┘   └──────────────────────────────────┘
        │                                             │
        ▼                                             ▼
┌──────────────────────────────────────┐   ┌──────────────────────────────────┐
│ 1. STANDARD_RX (1239)                │   │ 1. Set TX frame = PING_PACKET    │
│    writeFastCommand(0x02)            │   │    write(TX_BUFFER, 0, 0)        │
│    Register: Fast command 0x02       │   │    Register: 0x14:0x00 = 0       │
│                                       │   │    setFrameLength(1)              │
│ 2. Wait for packet (1241)            │   │    write TX_FCTRL with len=3     │
│    while(!receivedFrameSucc())       │   │                                   │
│    Polls SYS_STATUS (0x00:0x44)      │   │ 2. tx_instant_rx() (1404)        │
│    Looking for bit 13 (RXFCG)        │   │    writeFastCommand(0x0C)        │
│                                       │   │    Fast command: TX then RX      │
│ 3. clear_system_status() (1242)      │   │                                   │
│    write(0x00:0x44, 0x3F7FFFFF)      │   │ 3. Wait for response (1406-1415)  │
│                                       │   │    timeout counter, max 10 ticks  │
│ 4. If rx_status == 1 (valid pkt):    │   │    receivedFrameSucc() polling    │
│                                       │   │                                   │
│    delayedTX() - Lines 744-811       │   │ 4. If rx_status == 1 (1420):     │
│    ┌─────────────────────────────┐   │   │    ┌──────────────────────────┐  │
│    │ a. readRXTimestamp() (748)  │   │   │    │ a. Read reply_delay from  │  │
│    │    result = IP_TS register  │   │   │    │    RX_BUFFER_0_REG:0x00  │  │
│    │    Read 0x0C:0x00 (low 32)  │   │   │    │    Register: 0x12:0x00   │  │
│    │    Read 0x0C:0x04 (hi 8)    │   │   │    │                           │  │
│    │    → rx_lo, rx_hi (40-bit)  │   │   │    │ b. getClockOffset() (1427)│  │
│    │                              │   │   │    │    Read DRX_CAR_INT      │  │
│    │ b. Calculate next TX time   │   │   │    │    Register: 0x06:0x29   │  │
│    │    add40(rx, TRANSMIT_DELAY)│   │   │    │    Sign extend bit 20    │  │
│    │    TRANSMIT_DELAY=0x165a0bc0│   │   │    │                           │  │
│    │    = 6ms in 15.65ps units   │   │   │    │ c. readTXTimestamp()     │  │
│    │    → next_tx (40-bit)       │   │   │    │    Read TX_TIME register │  │
│    │                              │   │   │    │    Register: 0x00:0x74   │  │
│    │ c. Shift to 32-bit (759-760)│   │   │    │    → tx_lo, tx_hi        │  │
│    │    exact_tx = next_tx >> 8  │   │   │    │                           │  │
│    │    (only 31 bits used)      │   │   │    │ d. readRXTimestamp()     │  │
│    │                              │   │   │    │    Read IP_TS register   │  │
│    │ d. Quantize to 512 (764-765)│   │   │    │    → rx_lo, rx_hi        │  │
│    │    calc_tx = next_tx &      │   │   │    │                           │  │
│    │              DX_TIME_MASK   │   │   │    │ e. Calculate round trip  │  │
│    │    (mask = 0xFFFFFE00)      │   │   │    │    sub40(rx, tx)         │  │
│    │                              │   │   │    │    → result_lo           │  │
│    │ e. Calculate reply_delay    │   │   │    │                           │  │
│    │    sub40(calc_tx, rx)       │   │   │    │ f. IF ENABLE_PDOA (1438): │  │
│    │    → reply_delay (32-bit)   │   │   │    │    Read PDOA register    │  │
│    │                              │   │   │    │    Reg 0x0C:0x1E (14bit) │  │
│    │ f. Set payload (796-798)    │   │   │    │    Read STS_STS quality  │  │
│    │    write(TX_BUFFER, 0,      │   │   │    │    Reg 0x02:0x08 (8bit)  │  │
│    │          reply_delay)       │   │   │    │    Check STS_TOAST bits  │  │
│    │    Register: 0x14:0x00      │   │   │    │    Reg 0x0C:0x0C+4, bit23│  │
│    │    setFrameLength(4)        │   │   │    │    Combine into 32-bit:  │  │
│    │                              │   │   │    │    pdoa = (angle & 0x3FFF)│  │
│    │ g. Write DX_TIME (802)      │   │   │    │         | (quality << 16) │  │
│    │    WRITE_TX_DELAY(exact_tx) │   │   │    │         | (error_flag<<15)│  │
│    │    write_len(0x00:0x2C,     │   │   │    │                           │  │
│    │              exact_tx, 4)   │   │   │    │ g. If valid (result_lo >= │  │
│    │                              │   │   │    │    TRANSMIT_DELAY) (1457):│  │
│    │ h. DELAYED_TX (803)         │   │   │    │    Send binary packet:    │  │
│    │    writeFastCommand(0x3)    │   │   │    │    0xFF 0xD2 (header)    │  │
│    │    Toggles LED (806)        │   │   │    │    result_lo (round trip)│  │
│    │                              │   │   │    │    reply_delay           │  │
│    │ i. Wait TX complete (809)   │   │   │    │    clock_offset          │  │
│    │    while(!sentFrameSucc())  │   │   │    │    pdoa (if enabled)     │  │
│    │    Polls SYS_STATUS bit 7   │   │   │    │    Each 32-bit, LSB first│  │
│    │    Register: 0x00:0x44      │   │   │    │                           │  │
│    │                              │   │   │    │ h. clear_system_status() │  │
│    │ j. clear_system_status()    │   │   │    │                           │  │
│    └─────────────────────────────┘   │   │    └──────────────────────────┘  │
│                                       │   │                                   │
│ 5. Loop back to step 1                │   │ 5. If timeout: CANCEL_RX (1474)  │
│                                       │   │    writeFastCommand(0)           │
│                                       │   │                                   │
│                                       │   │ 6. clear_system_status() (1477)  │
│                                       │   │                                   │
│                                       │   │ 7. Toggle LED (1480)              │
│                                       │   │                                   │
│                                       │   │ 8. Loop back to step 1            │
└──────────────────────────────────────┘   └──────────────────────────────────┘
```

## Key Register Operations

### Critical Registers for PDoA

#### 1. SYS_CFG (0x00:0x10) - Line 944
System configuration register that enables PDoA mode:
- **Bits 18:17 = 3**: Enable PDoA mode (BASE only)
- **Bit 12 = 1**: CP_SPC (STS positioned after data packet)
- **Bit 15 = 1**: CP_SDC (use fixed STS code)

#### 2. STS_CFG (0x02:0x00) - Line 962
STS (Scrambled Timestamp Sequence) configuration:
- **Value = 15**: Sets STS length to 128 symbols
- **Formula**: `(length_in_symbols / 8) - 1 = (128 / 8) - 1 = 15`

#### 3. STS_CONF_0 (0x0E:0x12) - Line 968
STS configuration and threshold:
- **Bits 22:16 = 12**: STS_MNTH (threshold for STS quality)
- Must be set according to STS length for proper detection

#### 4. CIA_ADJUST (0x0E:0x1A) - Line 972
Channel impulse analysis adjustment:
- **Bits 13:0 = 3600**: Phase offset adjustment
- Improves PDoA angle readability and accuracy

#### 5. PDOA (0x0C:0x1E) - Line 1439
Phase Difference of Arrival result register:
- **Bits 13:0**: Phase difference angle
- **Range**: 0-16383 representing 0° to 360°
- **Resolution**: ~0.022° per LSB
- Read after successful RX on BASE board

#### 6. STS_STS (0x02:0x08) - Line 1440
STS quality indicator:
- **Value**: Percentage of good STS symbols (0-255)
- **Validity**: Must be > 60% of STS length (i.e., > 77 for 128 symbols)
- Used to validate PDoA measurement quality

#### 7. Timestamp Registers

**IP_TS (0x0C:0x00)** - RX Timestamp
- **Size**: 40 bits (5 bytes)
- **Low 32 bits**: Register 0x0C:0x00
- **High 8 bits**: Register 0x0C:0x04 (bits 7:0)
- **Units**: ~15.65 picoseconds
- Captured at packet reception

**TX_TIME (0x00:0x74)** - TX Timestamp
- **Size**: 40 bits (5 bytes)
- **Low 32 bits**: Register 0x00:0x74
- **High 8 bits**: Register 0x00:0x78 (bits 7:0)
- **Units**: ~15.65 picoseconds
- Captured at packet transmission

**DX_TIME (0x00:0x2C)** - Delayed TX Time
- **Size**: 32 bits (only 31 bits used)
- **Write value**: Upper 32 bits of desired 40-bit TX timestamp
- **Quantization**: Lower 9 bits ignored (512 unit boundaries = ~8ns)

#### 8. CHAN_CTRL (0x01:0x14) - Lines 980-995
Channel control register:
- **Bits 1:0**: RF_CHAN (0x0 = CH5, 0x1 = CH9)
- **Bits 7:3**: TX_PCODE (preamble code for TX)
- **Bits 12:8**: RX_PCODE (preamble code for RX)
- **Bits 2:1**: SFD_TYPE (set to 3 for PDoA with STS)

#### 9. TX_FCTRL (0x00:0x24) - Lines 997-1002
Transmit frame control:
- **Bits 9:0**: Frame length in bytes (including 2-byte FCS)
- **Bits 11:10**: Data rate (0x1 = 6.8Mbps, 0x0 = 850kbps)
- **Bits 21:12**: Preamble length code

#### 10. SYS_STATUS (0x00:0x44) - Status Register
System status register (read/clear):
- **Bit 7 (0x80)**: TXFRS - TX frame sent
- **Bit 13 (0x2000)**: RXFCG - RX frame with good CRC
- **Bits 14,16-20,26 (0x4279000)**: Various RX error flags
- **Clear**: Write 1 to bit to clear (write 0x3F7FFFFF to clear all)

#### 11. DRX_CAR_INT (0x06:0x29) - Clock Offset
Carrier integrator register:
- **Bits 20:0**: Raw clock offset value (21-bit signed)
- **Sign extension**: If bit 20 is set, extend with 0xFFE00000
- **Units**: PPM offset between transmitter and receiver clocks

#### 12. RF Configuration Registers

**RF_TX_CTRL_2 (0x07:0x1C)** - Lines 1019
- **CH9**: 0x1C010034
- **CH5**: 0x1C071134
- Configures TX power amplifier for specific channel

**PLL_CFG (0x09:0x00)** - Line 1020
- **CH9**: 0x0F3C
- **CH5**: 0x1F3C
- PLL configuration for channel frequency

**RX_CTRL_HI (0x07:0x10)** - Line 1022
- **CH9**: 0x08B5A833
- RX analog configuration (undocumented, CH9 only)

**TX_POWER (0x01:0x0C)** - Line 531
- **Value**: 0xFCFCFCFC
- Sets TX power levels for different data rates

#### 13. Calibration Registers

**RX_CAL (0x04:0x0C)** - Line 1068
- **0x11**: Start PGF (Pulse Generator Filter) calibration
- **0x00**: Stop calibration

**RX_CAL_STS (0x04:0x20)** - Line 1072
- **Bit 0**: Calibration complete status
- Poll until set, then write 0x01 to clear

**RX_CAL_RESI/RESQ (0x04:0x14, 0x04:0x1C)** - Lines 1091-1097
- Calibration results for I and Q channels
- **0x1FFFFFFF**: Indicates calibration failure

## Timing Parameters

### Time Units
- **Base unit**: ~15.65 picoseconds
- **Calculation**: 1 / (499.2 MHz / 128) = 15.65 ps
- **40-bit timestamp range**: ~4.4 milliseconds before wraparound
- **32-bit DX_TIME range**: ~17.2 milliseconds before wraparound

### Critical Timing Values

**TRANSMIT_DELAY** (Line 202)
```c
#define TRANSMIT_DELAY 0x165a0bc0  // 6ms in 15.65ps units
```
- **Value**: 377,226,176 units
- **Time**: ~5.9 ms
- Delay between RX and delayed TX in ROAMER

**DX_TIME_MASK** (Line 205)
```c
#define DX_TIME_MASK 0xFFFFFE00
```
- **Quantization**: 512 units (9-bit boundary)
- **Time quantum**: ~8 nanoseconds
- DW3220 can only schedule TX at these boundaries

### Timeout Values

**RX Timeout** (Lines 714-715)
- **Stage 1**: 10 ticks (~10.2 ms)
- **Stage 2**: 15 ticks (~15.4 ms)
- **1 tick**: ~1.024 ms (at 977 Hz)

## Operating Modes

### Single-Sided Ranging (SSR)
Default mode when `ENABLE_DS` is not defined.

**Protocol Flow:**
```
BASE                    ROAMER
  |                        |
  |---> PING (instant) --->|
  |                        | Timestamp: rx_ts
  |                        | Calculate: tx_ts = rx_ts + 6ms
  |                        |
  |<--- PONG (delayed) ----|
  | Timestamp: tx_ts, rx_ts|
  |                        |
  | Calculate distance     |
```

**Distance Calculation:**
```
round_trip = rx_ts - tx_ts
flight_time = round_trip - reply_delay
distance = flight_time * c / 2

where:
  c = speed of light = 299,792,458 m/s
  time units = 15.65 ps
```

### Double-Sided Ranging (DSR)
Enabled with `#define ENABLE_DS`. More complex, accounts for clock drift.

**Protocol Flow:**
```
BASE                    ROAMER
  |                        |
  |---> Packet 1 --------->|
  |                        |
  |<---- Packet 2 ---------|
  |                        |
  |---> Packet 3 --------->|
  |                        |
  |<---- Packet 4 ---------|
  |                        |
  | Calculate distance     |
```

**Distance Calculation (DSR):**
```
roundA = time between TX1 and RX2
replyA = time between RX2 and TX3
roundB = time between TX2 and RX1 (from ROAMER)
replyB = time between RX1 and TX2 (from ROAMER)

flight_time = (roundA * roundB - replyA * replyB) / (roundA + roundB + replyA + replyB)
distance = flight_time * c
```

## Binary Protocol

### BASE Output Format (SSR Mode)

When BASE successfully receives a response, it outputs binary data:

```
Offset  Size  Description
------  ----  -----------
0       1     0xFF (sync byte 1)
1       1     0xD2 (sync byte 2, SSR packet identifier)
2       4     round_trip (32-bit LSB first)
6       4     reply_delay (32-bit LSB first)
10      4     clock_offset (32-bit LSB first)
14      4     pdoa (32-bit LSB first, if ENABLE_PDOA)
```

**Total packet size:**
- Without PDoA: 14 bytes
- With PDoA: 18 bytes

### PDoA Data Format (32-bit)

```
Bit     Description
----    -----------
31:24   Reserved
23:16   STS quality (0-255)
15      Error flag (1 = STS timeout error)
14      Reserved
13:0    Phase angle (0-16383 = 0° to 360°)
```

**Angle calculation:**
```c
angle_degrees = (pdoa & 0x3FFF) * 360.0 / 16384.0
quality_percent = ((pdoa >> 16) & 0xFF) * 100.0 / 255.0
has_error = (pdoa & 0x8000) != 0
```

### BASE2 Output Format (Experimental)

```
Offset  Size  Description
------  ----  -----------
0       1     0xFF (sync byte 1)
1       1     0xD3 (sync byte 2, BASE2 packet identifier)
2       4     round_trip (32-bit LSB first)
6       4     reply_delay (32-bit LSB first)
10      4     clock_offset (32-bit LSB first)
```

## SPI Communication

### SPI Protocol (Bit-banged)
- **Mode**: Mode 0 (CPOL=0, CPHA=0)
- **Clock**: Software-generated, ~1 MHz max
- **Bit order**: MSB first
- **Byte order**: LSB first (little-endian for multi-byte values)

### Register Access Format

**Header Byte 1:**
```
Bit 7:    Write (1) / Read (0)
Bit 6:    Extended addressing (1 = 2-byte header)
Bits 5:1: Base register address (0x00 - 0x1F)
Bit 0:    Always 0
```

**Header Byte 2 (if extended):**
```
Bits 7:2: Sub-address offset (0x00 - 0x3F, shifted left by 2)
Bits 1:0: Always 0
```

### Fast Commands
Single-byte commands (Line 521-526):
```
Bit 7:    Always 1
Bit 6:    Always 0
Bits 5:1: Command code (0x00 - 0x1F)
Bit 0:    Always 1
```

**Common Fast Commands:**
- `0x81` (cmd 0x00): Cancel/idle
- `0x83` (cmd 0x01): Start TX
- `0x85` (cmd 0x02): Start RX
- `0x87` (cmd 0x03): Start delayed TX
- `0x9F` (cmd 0x0F): Delayed TX then RX
- `0x99` (cmd 0x0C): TX then instant RX

## Debug Output

When `DEBUG` is defined, diagnostic messages are sent via UART:

### Initialization Messages
- `"ROAMER\n"` or `"BASE\n"` - Board identification
- `"PLL is now locked.\n"` - PLL successfully locked
- `"PGF calibration complete.\n"` - RX calibration success
- Various error messages if calibration fails

### Debug Functions
- `print_number(n)` - Print decimal number
- `print_hex2(n)` - Print 8-bit hex value
- `print_hex8(n)` - Print 32-bit hex value
- `print_text(s)` - Print string
- `print_bin32(n)` - Print 32-bit binary (LSB first)

## Function Reference

### Initialization Functions

**`init_radio()` (Line 877)**
- Performs complete DW3220 initialization
- Configures all registers for UWB operation
- Returns when radio is ready for operation

**`readOTP(addr)` (Line 509)**
- Reads One-Time Programmable memory
- Used for factory calibration values
- Returns 32-bit value from OTP address

### SPI Functions

**`spi_transfer(data)` (Line 178)**
- Sends and receives one byte via SPI
- Bit-banged implementation
- Returns received byte

**`send_bytes(data, len, rec_len)` (Line 353)**
- Low-level SPI transaction
- Sends `len` bytes, receives `rec_len` bytes
- Returns received data as 32-bit value (up to 4 bytes)

**`read(base, sub)` (Line 485)**
- Reads from DW3220 register
- Returns 32-bit value (auto-determines length)

**`read_len(base, sub, len)` (Line 489)**
- Reads specific number of bytes
- Returns up to 32 bits

**`write(base, sub, data)` (Line 474)**
- Writes to DW3220 register
- Auto-determines data length from value

**`write_len(base, sub, data, dataLen)` (Line 462)**
- Writes specific number of bytes
- Useful for writing exact lengths

**`writeFastCommand(cmd)` (Line 521)**
- Sends fast command (single byte)
- Common commands: TX, RX, delayed TX, etc.

### Timing Functions

**`readRXTimestamp()` (Line 645)**
- Reads 40-bit RX timestamp from IP_TS register
- Stores result in `result_hi` (8-bit) and `result_lo` (32-bit)

**`readTXTimestamp()` (Line 655)**
- Reads 40-bit TX timestamp from TX_TIME register
- Stores result in `result_hi` and `result_lo`

**`add40(hi1, lo1, hi2, lo2)` (Line 631)**
- Adds two 40-bit values
- Result in `result_hi` and `result_lo`
- Handles carry between low and high parts

**`sub40(hi1, lo1, hi2, lo2)` (Line 638)**
- Subtracts two 40-bit values
- Result in `result_hi` and `result_lo`
- Handles borrow between low and high parts

**`getClockOffset()` (Line 661)**
- Reads carrier frequency offset
- Sign-extends 21-bit value to 32 bits
- Returns signed offset in PPM-related units

### Frame Functions

**`setFrameLength(frameLen)` (Line 683)**
- Sets TX frame length in bytes
- Automatically adds 2-byte FCS
- Updates TX_FCTRL register

**`sendFrame(stage)` (Line 698)**
- Sends one-byte frame with stage number
- Calls `tx_instant_rx()` to transmit then receive
- Used in double-sided ranging

**`receivedFrameSucc()` (Line 590)**
- Checks if frame received successfully
- Returns: 1 = success, 2 = error, 0 = no frame
- Reads SYS_STATUS register

**`sentFrameSucc()` (Line 614)**
- Checks if frame transmitted successfully
- Returns: 1 = success, 0 = not sent
- Reads SYS_STATUS register

**`waitReceive(stage)` (Line 710)**
- Waits for frame reception with timeout
- Different timeouts for different stages
- Cancels RX on timeout
- Returns RX status

**`delayedTX()` (Line 744)**
- Implements delayed transmission (ROAMER)
- Calculates TX time based on RX timestamp + 6ms
- Sends reply with calculated delay value
- Core of single-sided ranging response

**`clear_system_status()` (Line 533)**
- Clears all status bits
- Write 0x3F7FFFFF to SYS_STATUS register

## Macros

**`CONFIGURE_AS_TX`** (Line 529)
- Configures radio for transmission
- Sets TX power and output delay

**`STANDARD_TX`** (Line 539)
- Starts immediate transmission
- Fast command 0x01

**`STANDARD_RX`** (Line 542)
- Starts immediate reception
- Fast command 0x02

**`DELAYED_TX`** (Line 550)
- Starts delayed transmission
- TX time from DX_TIME register
- Fast command 0x03

**`DELAYED_TX_THEN_RX`** (Line 554)
- Delayed TX followed by RX
- Fast command 0x0F

**`CANCEL_RX`** (Line 557)
- Cancels ongoing RX
- Required before next TX after timeout
- Fast command 0x00

**`SET_TX_FRAME(data)`** (Line 676)
- Writes data to TX buffer
- Single 32-bit value

**`WRITE_TX_DELAY(value)`** (Line 560)
- Sets delayed TX timestamp
- Must write all 32 bits

## Configuration Options

### Board Selection (Line 56-59)
```c
#define ROAMER 0
#define BASE 1
#define BASE2 2
#define BOARD BASE  // Change this to select board type
```

### Feature Flags (Lines 63-65)
```c
//#define ENABLE_PHASE  // Phase delay measurement (experimental)
#define ENABLE_PDOA     // Enable PDoA angle measurement
//#define ENABLE_DS       // Double-sided ranging (vs single-sided)
```

### Debug Options (Line 61, 95, 879)
```c
#define DEBUG           // Enable debug UART output
#define DEBUG_INIT      // Enable initialization debug messages
```

## Error Handling

### Common Issues

**PLL Lock Failure**
- Symptom: `DEV_ID` bit 1 never sets
- Check: Crystal oscillator, XTAL_TRIM value
- Recovery: Power cycle, verify hardware

**PGF Calibration Failure**
- Symptom: RX_CAL_STS doesn't set, or RESI/RESQ = 0x1FFFFFFF
- Check: LDO configuration, RF settings
- Recovery: Re-run calibration sequence

**RX Timeout**
- Symptom: `receivedFrameSucc()` returns 0
- Action: `CANCEL_RX` macro called automatically
- Required before next transmission

**Invalid PDoA**
- Check: STS quality < 60% threshold
- Check: STS_TOAST bit set (timing error)
- Action: Discard measurement, use next packet

### Status Checking

Always check status after operations:
```c
// After RX
rx_status = receivedFrameSucc();
if(rx_status == 1) {
    // Valid packet, process data
} else if(rx_status == 2) {
    // RX error, discard
} else {
    // Timeout, cancel RX
    CANCEL_RX
}
clear_system_status();
```

## Performance Characteristics

### Ranging Update Rate
- **Single-sided**: ~100 Hz (limited by 6ms ROAMER delay + TX/RX time)
- **Double-sided**: ~25 Hz (4 packet exchanges)

### Ranging Accuracy
- **Without PDoA**: ~10cm typical (time-of-flight only)
- **With PDoA**: Angular accuracy ~1-2° typical
- **Clock offset compensation**: Improves accuracy with frequency drift

### Power Consumption
- **TX**: High power (~100 mA typical at full power)
- **RX**: Medium power (~60 mA typical)
- **IDLE**: Low power (not used in this code, always active)

## References

### Key Documentation
- DW3000 User Manual (referenced throughout code)
- Arduino DW3000 library: https://github.com/Fhilb/DW3000_Arduino/
- Decadriver reference: https://github.com/foldedtoad/dwm3000/
- Configuration notes: https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54

### Important User Manual Sections
- Chapter 3.2: Timestamp system and units
- Chapter 9: Fast commands
- Page 27: SPI header format
- Page 77-80: SYS_CFG register (PDoA enable)
- Page 87: DX_TIME register (delayed TX)
- Page 93: SYS_STATUS register (status bits)
- Page 122-125: STS configuration
- Page 149-155: RF configuration registers
- Page 162: PLL configuration
- Page 179-183: Timestamp and PDoA registers
- Page 199-204: CIA configuration

## License

GNU General Public License v2.0 or later

Copyright (C) 2025 Adam Williams

## Notes

- This is reference implementation code, optimized for simplicity over performance
- Space/ROM is constrained on PIC16F1508, many optimizations for code size
- No floating point math (not available on PIC16)
- All 40-bit timestamp arithmetic done with separate high/low 32-bit values
- Temperature sensor code commented out but available (SAR ADC)
- Antenna delay calibration disabled (not significantly beneficial)
- Soft reset functionality commented out (not required for stable operation)

## Future Improvements

- [ ] Average multiple PDoA measurements for better accuracy
- [ ] Implement adaptive timeout based on distance
- [ ] Add temperature compensation for timestamp drift
- [ ] Support for more than 2 boards in PDoA mode
- [ ] Implement secure timestamp sequence (STS) with encryption
- [ ] Power saving modes (sleep/wake cycles)
- [ ] Bidirectional ranging (both BASE and ROAMER can initiate)

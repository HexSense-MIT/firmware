# DW3000 UWB Wireless Communication - Step-by-Step Code Explanation

This document provides a comprehensive, step-by-step explanation of the DW3000 Ultra-Wideband (UWB) wireless communication implementation on STM32F722. The main file is `main.c`, which demonstrates both TX (transmitter) and RX (receiver) operations.

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Configuration](#hardware-configuration)
- [Initialization Flow](#initialization-flow)
- [Register Operations](#register-operations)
- [TX (Transmit) Flow](#tx-transmit-flow)
- [RX (Receive) Flow](#rx-receive-flow)
- [Complete Flowchart](#complete-flowchart)

---

## Overview

The DW3000 is a Qorvo (formerly DecaWave) Ultra-Wideband (UWB) transceiver chip that provides precision wireless communication and ranging capabilities. This implementation uses:

- **MCU**: STM32F722
- **UWB Chip**: DW3000 (Device ID: 0xDECA0302)
- **Communication**: SPI interface
- **Operating Mode**: Non-STS (Scrambled Timestamp Sequence) mode
- **Channel**: Channel 5 (can be configured to Channel 9)
- **Data Rate**: 6.8 Mbps
- **Preamble Length**: 128 symbols

---

## System Architecture

### File Structure

```
Core/
├── Inc/
│   ├── main.h                      # Main header
│   ├── DW3000_FZ.h                 # DW3000 driver header
│   ├── DW3000_send_test_FZ.h       # TX functions header
│   ├── DW3000_recv_test_FZ.h       # RX functions header
│   ├── dw3000_deca_regs.h          # Register definitions
│   ├── dw3000_deca_vals.h          # Register values
│   └── HS_system.h                 # System utilities
└── Src/
    ├── main.c                      # Main application
    ├── DW3000_FZ.c                 # DW3000 driver implementation
    ├── DW3000_send_test_FZ.c       # TX functions
    └── DW3000_recv_test_FZ.c       # RX functions
```

### Communication Configuration

Defined in `main.c:35-49`:

```c
static dwt_config_t config = {
  5,                   // Channel number (Channel 5)
  DWT_PLEN_128,        // Preamble length: 128 symbols
  DWT_PAC8,            // Preamble acquisition chunk: PAC 8
  9,                   // TX preamble code: Code 9 (64 MHz PRF)
  9,                   // RX preamble code: Code 9 (64 MHz PRF)
  1,                   // SFD type: Non-standard 8 symbol
  DWT_BR_6M8,          // Data rate: 6.8 Mbps
  DWT_PHRMODE_STD,     // PHY header mode: Standard
  DWT_PHRRATE_STD,     // PHY header rate: Standard
  (129 + 8 - 8),       // SFD timeout: 129 symbols
  DWT_STS_MODE_OFF,    // STS mode: OFF (no scrambled timestamp)
  DWT_STS_LEN_64,      // STS length: 64 (not used when STS is OFF)
  DWT_PDOA_M0          // PDOA mode: OFF
};
```

---

## Hardware Configuration

### GPIO Pins

Defined in `main.c:426-511`:

| Pin Name | Function | Port | Description |
|----------|----------|------|-------------|
| UWB_PWR_EN | Output | PA | DW3000 power enable (3.3V LDO) |
| UWB_RST | Output | PA | DW3000 hardware reset |
| UWB_CS | Output | PC | SPI chip select (active low) |
| UWB_IRQ | Input | PB | Interrupt request (rising edge) |
| PIN_LED1 | Output | PB | Status LED 1 |
| PIN_LED2 | Output | PC | Status LED 2 |

### SPI Configuration

**SPI1** is used for DW3000 communication:

- **Mode**: Master
- **Data Size**: 8-bit
- **Clock Polarity**: Low (CPOL=0)
- **Clock Phase**: 1st Edge (CPHA=0)
- **NSS**: Software controlled
- **Initial Baud Rate**: Prescaler 32 → ~3 MHz (low speed for initialization)
- **Operating Baud Rate**: Prescaler 2 → ~24 MHz (high speed for data transfer)

**Location**: `main.c:346-378`

### Interrupt Configuration

**EXTI2** is configured for DW3000 IRQ:
- **Line**: EXTI2 (UWB_IRQ_Pin)
- **Mode**: Rising edge trigger
- **Priority**: 0 (highest)

**Location**: `main.c:502-503`

---

## Initialization Flow

### Step-by-Step Initialization Process

The initialization happens in `main.c:102-208`:

#### 1. **MCU Initialization** (`main.c:112-130`)

```c
HAL_Init();                // Initialize HAL library
SystemClock_Config();      // Configure system clock to 96 MHz
MX_GPIO_Init();            // Initialize GPIO pins
MX_SPI1_Init();            // Initialize SPI1 (low speed)
MX_SPI2_Init();            // Initialize SPI2
MX_FATFS_Init();           // Initialize FAT filesystem
MX_USB_DEVICE_Init();      // Initialize USB device
```

#### 2. **DW3000 Power On** (`main.c:135-138`)

```c
DW3000poweron();           // Set UWB_PWR_EN pin HIGH → Enable 3.3V LDO
HAL_Delay(10);             // Wait 10ms for power stabilization
DW3000hardReset();         // Toggle UWB_RST pin (LOW→HIGH)
HAL_Delay(10);             // Wait 10ms for reset completion
```

**Implementation** (`DW3000_FZ.c:181-206`):
- `DW3000poweron()`: Sets `UWB_PWR_EN` to HIGH
- `DW3000hardReset()`:
  - Sets `UWB_RST` to LOW
  - Waits 10ms
  - Sets `UWB_RST` to HIGH (open-drain, never drive high continuously)

#### 3. **Set SPI Low Speed** (`main.c:140`)

```c
set_SPI2lowspeed(&hspi1);  // Set SPI to 3 MHz (prescaler = 16)
```

**Register**: SPI1 BaudRatePrescaler
**Value**: `SPI_BAUDRATEPRESCALER_16`
**Reason**: DW3000 requires ≤7 MHz SPI speed during initialization and reset

#### 4. **Wait for SPI Ready** (`main.c:144-146`)

```c
while(!(dwt_read32bitoffsetreg(SYS_STATUS_ID, 0) & SYS_STATUS_SPIRDY_BIT_MASK)) {
    HAL_Delay(10);
}
```

**Register**: `SYS_STATUS` (0x00000044)
**Bit**: `SPIRDY` (bit 23) = `0x800000`
**Operation**: Read and check bit 23
**Purpose**: Wait until DW3000 SPI interface is ready

#### 5. **Verify Device ID** (`main.c:149-157`)

```c
uint32_t dev_id = dwt_read32bitoffsetreg(DEV_ID_ID, 0);

if (dev_id == (uint32_t)DWT_DW3000_DEV_ID) {
    printf("DW3000 Device ID: 0x%08lX\r\n", dev_id);
    blink_led(PIN_LED1_GPIO_Port, PIN_LED1_Pin, 50);
} else {
    printf("Wrong Device ID: 0x%08lX\r\n", dev_id);
    while (1);  // Halt on error
}
```

**Register**: `DEV_ID` (0x00000000)
**Expected Value**: `0xDECA0302` (DW3000 non-PDOA version)
**Length**: 4 bytes
**Purpose**: Verify DW3000 chip is present and responding

#### 6. **Check IDLE_RC State** (`main.c:161-166`)

```c
if(DW3000check_IDLE_RC()) {
    // Device is in IDLE_RC state (good)
} else {
    while (1);  // Halt if not in IDLE_RC
}
```

**Register**: `SYS_STATUS` (0x00000044)
**Bit**: `RCINIT` (bit 24) = `0x1000000`
**Operation**: Check if bit 24 is set
**Purpose**: Verify device is in IDLE_RC (RC oscillator running) state

**Implementation** (`DW3000_FZ.c:279-282`):
```c
uint8_t DW3000check_IDLE_RC(void) {
    uint32_t reg = dwt_read32bitoffsetreg(SYS_STATUS_ID, 0);
    return ((reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}
```

#### 7. **Initialize DW3000** (`main.c:169-172`)

```c
if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    printf("dwt_initialise error\r\n");
    while (100) {;}
}
```

**Function**: `dwt_initialise(DWT_DW_INIT)`
**Parameter**: `DWT_DW_INIT = 0x0`
**Operations**:
- Checks device ID
- Reads OTP (One-Time Programmable) memory for calibration values
- Applies LDO and BIAS tuning from OTP
- Clears AON (Always-On) configuration
- Performs soft reset if needed

#### 8. **Enable LED Blinking** (`main.c:174`)

```c
dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
```

**Register**: `LED_CTRL` (0x110024)
**Register**: `GPIO_MODE` (0x110010)
**Operations**:
- Enable GPIO pins 2 and 3 as LED outputs
- Configure initial blink pattern
- GPIO2 → RXLED, GPIO3 → TXLED

#### 9. **Configure DW3000** (`main.c:178-181`)

```c
if (dwt_configure(&config))  {
    printf("CONFIG FAILED\r\n");
    while (100) {;}
}
```

**Function**: `dwt_configure(&config)`
**Key Register Operations**:

a. **Channel Configuration** - Register: `CHAN_CTRL` (0x11000C)
   - RX_PCODE = 9 (bits 8-12)
   - TX_PCODE = 9 (bits 0-4)
   - SFD_TYPE = 1 (bits 17-18)
   - RF_CHAN = 5 (bits 22-23)

b. **TX Frame Control** - Register: `TX_FCTRL` (0x000024)
   - TXPSR = preamble length (bits 12-15)
   - TXBR = data rate (bit 10)

c. **SYS_CFG** - Register: `SYS_CFG` (0x000010)
   - PHR_MODE (bit 4)
   - PHR_6M8 (bit 5)

d. **Run PLL Calibration**
   - Locks the PLL (Phase-Locked Loop)
   - Required for stable operation

e. **Run PGF Calibration** (Pulse Generator Frequency)
   - Calibrates receiver path
   - Critical for good RX performance

#### 10. **Configure TX or RX Mode** (`main.c:183-190`)

**TX Node Configuration**:
```c
if (current_node == tx_node) {
    DW3000_irq_for_tx_done();  // Enable TX done interrupt
    dwt_configuretxrf(0x34, 0xfdfdfdfd, 0x00);  // Configure TX power
}
```

**Register Operations for TX**:
- **SYS_ENABLE_LO** (0x00003C) - Enable TX frame sent interrupt
  - Set bit 7 (TXFRS_ENABLE) = 1
  - Value written: `0x00000080`

- **TX Power Configuration**:
  - PG_DELAY = 0x34
  - TX_POWER = 0xfdfdfdfd (all TX power levels set to 0xFD)
  - PG_COUNT = 0x00

**RX Node Configuration**:
```c
if (current_node == rx_node) {
    DW3000_irq_for_rx_done();  // Enable RX done interrupt
}
```

**Register Operations for RX**:
- **SYS_ENABLE_LO** (0x00003C) - Enable RX frame received interrupt
  - Set bit 14 (RXFCG_ENABLE) = 1
  - Value written: `0x00004000`

#### 11. **Enter IDLE_PLL State** (`main.c:192-197`)

```c
if(DW3000check_IDLE_RC()) {
    DW3000enter_IDLE_PLL();  // Transition to IDLE_PLL
    HAL_Delay(10);
} else {
    while (1);
}
```

**Register**: `SEQ_CTRL` (0x110008)
**Bit**: `AINIT2IDLE` (bit 8) = `0x100`
**Operation**: Set bit 8 to automatically transition to IDLE_PLL
**Purpose**: Enable PLL and prepare for TX/RX

**Implementation** (`DW3000_FZ.c:269-272`):
```c
void DW3000enter_IDLE_PLL(void) {
    set_bits(SEQ_CTRL_ID, SEQ_CTRL_AINIT2IDLE_BIT_MASK, 4);
    Delay_us(10);
}
```

#### 12. **Verify IDLE_PLL State** (`main.c:201-206`)

```c
if(DW3000check_IDLE_PLL() && DW3000check_IDLE()) {
    HAL_GPIO_WritePin(PIN_LED1_GPIO_Port, PIN_LED1_Pin, GPIO_PIN_SET);
} else {
    HAL_GPIO_WritePin(PIN_LED1_GPIO_Port, PIN_LED1_Pin, GPIO_PIN_RESET);
    while (1);
}
```

**PLL Lock Check** - Register: `SYS_STATUS` (0x000044)
- **Bit**: `CP_LOCK` (bit 1) = `0x2`
- **Operation**: Check if PLL is locked

**IDLE State Check** - Register: `SYS_STATE_LO` (0x000048)
- **Bits**: bits 16-23 should equal `DW_SYS_STATE_IDLE`
- **Operation**: Verify state machine is in IDLE

#### 13. **Set SPI High Speed** (`main.c:207`)

```c
set_SPI2highspeed(&hspi1);  // Set SPI to 24 MHz (prescaler = 2)
```

**Register**: SPI1 BaudRatePrescaler
**Value**: `SPI_BAUDRATEPRESCALER_2`
**Purpose**: Increase SPI speed for faster data transfer (now that initialization is complete)

---

## Register Operations

### SPI Communication Protocol

The DW3000 uses a custom SPI protocol with headers:

#### 1. **Short Address (1 byte header)**
For registers 0x00-0x1F:
```
Bit 7: R/W (0=Read, 1=Write)
Bits 6-1: Register address (5 bits)
Bit 0: Must be 0
```

#### 2. **Full Address (2 byte header)**
For registers with base + sub-index:
```
Byte 0:
  Bit 7: R/W (0=Read, 1=Write)
  Bit 6: Sub-index flag (1=sub-index follows)
  Bits 5-1: Base address (5 bits)
  Bit 0: Reserved

Byte 1:
  Bit 7: Extended sub-index (if sub > 127)
  Bits 6-0: Sub-index (7 bits)
```

#### 3. **Fast Commands (1 byte)**
For immediate actions:
```
Bit 7: Must be 1
Bits 6-1: Command code (5 bits)
Bit 0: Must be 1
```

**Example Commands**:
- `CMD_TX = 0x1` → Start transmission
- `CMD_RX = 0x2` → Start reception
- `CMD_CLR_IRQS = 0x12` → Clear all interrupts

### Key Registers

#### SYS_STATUS (0x000044) - System Status Register

**Length**: 8 bytes (64 bits)
**Access**: Read/Write (W1C - Write 1 to Clear)

Important bits:
- Bit 0 (IRQS): Interrupt request status
- Bit 1 (CP_LOCK): PLL lock status
- Bit 7 (TXFRS): TX frame sent
- Bit 13 (RXFR): RX frame ready
- Bit 14 (RXFCG): RX frame CRC good
- Bit 15 (RXFCE): RX frame CRC error
- Bit 23 (SPIRDY): SPI ready
- Bit 24 (RCINIT): RC oscillator initialized

#### SYS_ENABLE_LO (0x00003C) - System Enable Register (Lower)

**Length**: 4 bytes
**Access**: Read/Write

Important bits:
- Bit 7 (TXFRS_ENABLE): Enable TX frame sent interrupt
- Bit 13 (RXFR_ENABLE): Enable RX frame ready interrupt
- Bit 14 (RXFCG_ENABLE): Enable RX CRC good interrupt

#### DEV_ID (0x000000) - Device ID Register

**Length**: 4 bytes
**Access**: Read Only
**Value**: 0xDECA0302 (DW3000), 0xDECA0312 (DW3000 with PDOA)

#### TX_BUFFER (0x140000) - TX Data Buffer

**Length**: 1024 bytes
**Access**: Write
**Purpose**: Store data to be transmitted

#### RX_BUFFER (0x120000) - RX Data Buffer

**Length**: 1024 bytes
**Access**: Read
**Purpose**: Read received data

#### TX_FCTRL (0x000024) - TX Frame Control

**Length**: 4 bytes
**Access**: Read/Write

Important fields:
- Bits 0-9 (TXFLEN): TX frame length (including 2-byte CRC)
- Bit 10 (TXBR): TX bit rate
- Bit 11 (TR): Ranging enable
- Bits 12-15 (TXPSR): TX preamble symbol repetitions
- Bits 16-25 (TXB_OFFSET): TX buffer offset

#### RX_FINFO (0x00006C) - RX Frame Information

**Length**: 4 bytes
**Access**: Read Only

Important fields:
- Bits 0-6 (RXFLEN): RX frame length (standard mode, max 127 bytes)
- Bits 0-9 (RXFLEN): RX frame length (extended mode, max 1023 bytes)

#### SEQ_CTRL (0x110008) - Sequencer Control

**Length**: 4 bytes
**Access**: Read/Write

Important bits:
- Bit 8 (AINIT2IDLE): Auto transition from INIT to IDLE_PLL
- Bit 22 (FORCE2IDLE): Force to IDLE state
- Bit 23 (FORCE2INIT): Force to INIT state

---

## TX (Transmit) Flow

### Complete TX Process (`main.c:217-244`)

#### Step 1: Clear Interrupts (`main.c:219`)

```c
DW3000_clear_IRQ();
```

**Operations**:
1. Read `SYS_STATUS` register (0x000044) - 4 bytes
2. Write the same value back to clear flags (W1C - Write 1 to Clear)
3. Reset `DW3000_IRQ_flag` to false

**Purpose**: Clear any previous interrupt flags before starting new TX

#### Step 2: Write TX Data (`main.c:220`)

```c
DW3000_writetxdata_FZ(data2send, 10);
```

**Register**: `TX_BUFFER` (0x140000)
**Data**: `{0x00, 0x11, 0x23, 0x45, 0xAA, 0x01, 0xFE, 0x0A, 0xEE, 0x0B}`
**Length**: 10 bytes
**Operation**: Write data to TX buffer starting at offset 0

**Implementation** (`DW3000_send_test_FZ.c:12-14`):
```c
void DW3000_writetxdata_FZ(uint8_t *data, uint16_t len) {
    DW3000writereg(TX_BUFFER_ID, data, len);
}
```

**SPI Transaction**:
1. Pull CS LOW
2. Send 2-byte header: `[base_addr | RW_bit | sub_index_flag, sub_index]`
3. Send 10 bytes of data
4. Pull CS HIGH

#### Step 3: Configure TX Frame Control (`main.c:221`)

```c
dwt_writetxfctrl(12, 0, 0);
```

**Register**: `TX_FCTRL` (0x000024)
**Parameters**:
- `txFrameLength = 12` (10 bytes data + 2 bytes CRC)
- `txBufferOffset = 0`
- `ranging = 0` (not a ranging frame)

**Register Value Written**:
- Bits 0-9 (TXFLEN) = 12
- Bit 11 (TR) = 0
- Bits 12-15 (TXPSR) = value from config (preamble length)
- Bit 10 (TXBR) = value from config (data rate)

#### Step 4: Start Transmission (`main.c:222`)

```c
DW3000_txcmd_FZ(0);
```

**Fast Command**: `CMD_TX = 0x1`
**SPI Packet**: 1 byte = `0x83` (0b10000011)
- Bit 7 = 1 (fast command)
- Bits 6-1 = 0x01 (CMD_TX)
- Bit 0 = 1 (fast command flag)

**Implementation** (`DW3000_send_test_FZ.c:21-23`):
```c
void DW3000_txcmd_FZ(uint32_t delay) {
    DW3000_writefastCMD_FZ(CMD_TX);
}
```

**What Happens**:
1. DW3000 state machine transitions from IDLE → TX
2. Preamble is transmitted
3. SFD (Start Frame Delimiter) is transmitted
4. PHR (Physical Header) is transmitted
5. Data payload is transmitted
6. CRC is automatically appended and transmitted
7. State machine returns to IDLE
8. TX done interrupt is triggered

#### Step 5: Wait for TX Done (`main.c:225`)

```c
while (!DW3000_IRQ_flag) {;}
```

**Mechanism**:
- External interrupt on UWB_IRQ pin (EXTI2)
- ISR sets `DW3000_IRQ_flag = true`
- Main loop waits until flag is set

**Interrupt Handler** (in `stm32f7xx_it.c`):
```c
void EXTI2_IRQHandler(void) {
    if (HAL_GPIO_ReadPin(UWB_IRQ_GPIO_Port, UWB_IRQ_Pin)) {
        DW3000_IRQ_flag = true;
    }
    HAL_GPIO_EXTI_IRQHandler(UWB_IRQ_Pin);
}
```

#### Step 6: Check TX Status (`main.c:228-243`)

```c
DW3000_IRQ_flag = false;
uint32_t current_status = DW3000readreg(SYS_STATUS_ID, 4);

if (current_status & SYS_STATUS_TXFRS_BIT_MASK) {
    // TX successful
    printf("TX done, SYS_STATUS: 0x%08lX\r\n", current_status);
    DW3000_clear_IRQ();
    // Blink LED2
    HAL_GPIO_WritePin(GPIOC, PIN_LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOC, PIN_LED2_Pin, GPIO_PIN_RESET);
    HAL_Delay(950);
} else {
    // TX failed
    printf("TX failed, SYS_STATUS: 0x%08lX\r\n", current_status);
    DW3000_clear_IRQ();
    HAL_Delay(1000);
}
```

**Register**: `SYS_STATUS` (0x000044)
**Check Bit**: `TXFRS` (bit 7) = `0x80`
**Purpose**: Verify transmission was successful

**Timing**: 1 second period (50ms LED on, 950ms LED off)

---

## RX (Receive) Flow

### Complete RX Process (`main.c:246-280`)

#### Step 1: Clear RX Buffer (`main.c:248`)

```c
memset(rx_buffer, 0, sizeof(rx_buffer));
```

**Buffer Size**: `FRAME_LEN_MAX` bytes
**Purpose**: Clear previous received data

#### Step 2: Clear Interrupts (`main.c:249`)

```c
DW3000_clear_IRQ();
```

**Same as TX**: Clear all interrupt flags in `SYS_STATUS` register

#### Step 3: Start Receiver (`main.c:250`)

```c
DW3000_start_receiver_FZ();
```

**Fast Command**: `CMD_RX = 0x2`
**SPI Packet**: 1 byte = `0x85` (0b10000101)

**Implementation** (`DW3000_recv_test_FZ.c:14-16`):
```c
void DW3000_start_receiver_FZ(void) {
    DW3000_writefastCMD_FZ(CMD_RX);
}
```

**What Happens**:
1. DW3000 state machine transitions from IDLE → RX
2. Receiver is enabled and listening
3. Waits for valid preamble
4. When preamble detected, searches for SFD
5. After SFD, receives PHR
6. Receives data payload
7. Validates CRC
8. If CRC good, triggers RX done interrupt
9. State machine returns to IDLE

#### Step 4: Wait for RX Done (`main.c:254`)

```c
while (!DW3000_IRQ_flag) {;}
```

**Same mechanism as TX**: Wait for external interrupt to set flag

#### Step 5: Check RX Status (`main.c:256-279`)

```c
DW3000_IRQ_flag = false;
uint32_t current_status = DW3000readreg(SYS_STATUS_ID, 4);

if (current_status & SYS_STATUS_RXFR_BIT_MASK) {
    // RX successful
    DW3000_clear_IRQ();
    HAL_GPIO_WritePin(GPIOC, PIN_LED2_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOC, PIN_LED2_Pin, GPIO_PIN_RESET);

    // Read frame length
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;

    // Read received data (without CRC)
    dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0);

    // Clear RXFCG flag
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    // Print received data
    printf("RX OK, length %d bytes: ", frame_len);
    for (size_t i = 0; i < frame_len; i++) {
        printf("0x%02X ", rx_buffer[i]);
    }
    printf("\r\n");
} else {
    // RX failed or timeout
    DW3000_clear_IRQ();
    printf("RX failed, SYS_STATUS: 0x%08lX\r\n", current_status);
}
```

**Key Operations**:

a. **Read Frame Length**:
   - **Register**: `RX_FINFO` (0x00006C)
   - **Bits**: 0-6 (or 0-9 in extended mode)
   - **Mask**: `RX_FINFO_RXFLEN_BIT_MASK`
   - **Result**: Total frame length including CRC

b. **Read RX Data**:
   - **Register**: `RX_BUFFER` (0x120000)
   - **Length**: `frame_len - 2` (exclude 2-byte CRC)
   - **Offset**: 0
   - **Function**: `dwt_readrxdata()`

c. **Clear RXFCG Flag**:
   - **Register**: `SYS_STATUS` (0x000044)
   - **Bit**: RXFCG (bit 14) = `0x4000`
   - **Operation**: Write 1 to clear

---

## Complete Flowchart

```
┌─────────────────────────────────────────────────────────────────┐
│                     SYSTEM INITIALIZATION                       │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │   HAL_Init()      │
                    │   Clock Config    │
                    │   GPIO Init       │
                    │   SPI Init        │
                    └─────────┬─────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    DW3000 POWER & RESET                         │
├─────────────────────────────────────────────────────────────────┤
│  1. DW3000poweron()           [Set UWB_PWR_EN = HIGH]          │
│  2. Wait 10ms                                                   │
│  3. DW3000hardReset()         [Toggle UWB_RST: LOW→HIGH]       │
│  4. Wait 10ms                                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SPI LOW SPEED MODE                           │
├─────────────────────────────────────────────────────────────────┤
│  set_SPI2lowspeed()           [Prescaler = 16 → 3 MHz]        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│               WAIT FOR SPI READY                                │
├─────────────────────────────────────────────────────────────────┤
│  Read: SYS_STATUS (0x000044)                                   │
│  Check: SPIRDY bit (bit 23)                                     │
│  Poll until: SPIRDY = 1                                         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                VERIFY DEVICE ID                                 │
├─────────────────────────────────────────────────────────────────┤
│  Read: DEV_ID (0x000000) - 4 bytes                             │
│  Expected: 0xDECA0302                                           │
│  If match: Continue                                             │
│  If not: HALT                                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│              CHECK IDLE_RC STATE                                │
├─────────────────────────────────────────────────────────────────┤
│  Read: SYS_STATUS (0x000044)                                   │
│  Check: RCINIT bit (bit 24)                                     │
│  If set: Continue                                               │
│  If not: HALT                                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│            INITIALIZE DW3000 (dwt_initialise)                   │
├─────────────────────────────────────────────────────────────────┤
│  1. Verify device ID                                            │
│  2. Read OTP calibration data                                   │
│  3. Apply LDO and BIAS tuning                                   │
│  4. Clear AON configuration                                     │
│  5. Soft reset if needed                                        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                  ENABLE LED BLINK                               │
├─────────────────────────────────────────────────────────────────┤
│  Write: LED_CTRL (0x110024)                                     │
│  Write: GPIO_MODE (0x110010)                                    │
│  Configure: GPIO2=RXLED, GPIO3=TXLED                            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│          CONFIGURE DW3000 (dwt_configure)                       │
├─────────────────────────────────────────────────────────────────┤
│  Write: CHAN_CTRL (0x11000C)                                    │
│    - Channel 5                                                  │
│    - RX_PCODE = 9, TX_PCODE = 9                                │
│    - SFD_TYPE = 1                                               │
│                                                                 │
│  Write: TX_FCTRL (0x000024)                                     │
│    - Preamble length = 128                                      │
│    - Data rate = 6.8M                                           │
│                                                                 │
│  Write: SYS_CFG (0x000010)                                      │
│    - PHR_MODE = Standard                                        │
│    - PHR_RATE = Standard                                        │
│                                                                 │
│  Run PLL Calibration                                            │
│  Run PGF Calibration (RX calibration)                           │
└─────────────────────────────────────────────────────────────────┘
                              │
                 ┌────────────┴────────────┐
                 │                         │
                 ▼                         ▼
    ┌─────────────────────┐   ┌─────────────────────┐
    │   TX NODE CONFIG    │   │   RX NODE CONFIG    │
    ├─────────────────────┤   ├─────────────────────┤
    │ Enable TX IRQ:      │   │ Enable RX IRQ:      │
    │ SYS_ENABLE_LO       │   │ SYS_ENABLE_LO       │
    │ Set: TXFRS_ENABLE   │   │ Set: RXFCG_ENABLE   │
    │ (bit 7)             │   │ (bit 14)            │
    │                     │   │                     │
    │ Configure TX RF:    │   └─────────┬───────────┘
    │ PG_DELAY = 0x34     │             │
    │ TX_POWER = 0xFDFD.. │             │
    └──────────┬──────────┘             │
               │                        │
               └────────────┬───────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              ENTER IDLE_PLL STATE                               │
├─────────────────────────────────────────────────────────────────┤
│  Write: SEQ_CTRL (0x110008)                                     │
│  Set: AINIT2IDLE bit (bit 8)                                    │
│  Effect: Auto transition to IDLE_PLL                            │
│  Wait: 10ms                                                     │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│           VERIFY PLL LOCK & IDLE STATE                          │
├─────────────────────────────────────────────────────────────────┤
│  Read: SYS_STATUS (0x000044)                                   │
│  Check: CP_LOCK bit (bit 1) = 1                                │
│                                                                 │
│  Read: SYS_STATE_LO (0x000048)                                  │
│  Check: State = IDLE                                            │
│                                                                 │
│  If both OK: Turn on LED1, Continue                             │
│  If not: HALT                                                   │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                SPI HIGH SPEED MODE                              │
├─────────────────────────────────────────────────────────────────┤
│  set_SPI2highspeed()      [Prescaler = 2 → 24 MHz]            │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    MAIN LOOP                                    │
└─────────────────────────────────────────────────────────────────┘
                            │
                ┌───────────┴───────────┐
                │                       │
                ▼                       ▼
   ┌──────────────────────┐  ┌──────────────────────┐
   │     TX MODE          │  │     RX MODE          │
   └──────────────────────┘  └──────────────────────┘
                │                       │
                ▼                       ▼

════════════════════════════════════════════════════════════════════
                           TX FLOW
════════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────┐
│  STEP 1: CLEAR INTERRUPTS                                       │
├─────────────────────────────────────────────────────────────────┤
│  Read: SYS_STATUS (0x000044) - 4 bytes                         │
│  Write: Same value back (W1C - Write 1 to Clear)               │
│  Reset: DW3000_IRQ_flag = false                                 │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 2: WRITE TX DATA                                          │
├─────────────────────────────────────────────────────────────────┤
│  Write to: TX_BUFFER (0x140000)                                 │
│  Data: {0x00, 0x11, 0x23, 0x45, 0xAA, 0x01, 0xFE, 0x0A, ...}   │
│  Length: 10 bytes                                               │
│  Offset: 0                                                      │
│                                                                 │
│  SPI Transaction:                                               │
│    CS LOW                                                       │
│    Send: 2-byte header [0xD4, 0x00]                           │
│    Send: 10 bytes of data                                       │
│    CS HIGH                                                      │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 3: CONFIGURE TX FRAME CONTROL                             │
├─────────────────────────────────────────────────────────────────┤
│  Write to: TX_FCTRL (0x000024)                                  │
│  Frame Length: 12 (10 data + 2 CRC)                            │
│  Buffer Offset: 0                                               │
│  Ranging: 0 (disabled)                                          │
│                                                                 │
│  Register value:                                                │
│    TXFLEN (bits 0-9) = 12                                       │
│    TXBR (bit 10) = 1 (6.8M)                                    │
│    TR (bit 11) = 0 (no ranging)                                │
│    TXPSR (bits 12-15) = preamble config                         │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 4: START TRANSMISSION                                     │
├─────────────────────────────────────────────────────────────────┤
│  Send Fast Command: CMD_TX (0x1)                                │
│  SPI Packet: 1 byte = 0x83                                      │
│                                                                 │
│  DW3000 State Machine:                                          │
│    IDLE → TX_PREAMBLE → TX_SFD → TX_PHR → TX_DATA → IDLE      │
│                                                                 │
│  Transmission Sequence:                                         │
│    1. Preamble (128 symbols)                                    │
│    2. SFD (8 symbols, non-standard)                            │
│    3. PHR (Physical Header)                                     │
│    4. Data payload (10 bytes)                                   │
│    5. CRC (2 bytes, auto-calculated)                           │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 5: WAIT FOR TX DONE                                       │
├─────────────────────────────────────────────────────────────────┤
│  Wait: while (!DW3000_IRQ_flag)                                 │
│                                                                 │
│  Hardware Interrupt:                                            │
│    UWB_IRQ pin rises (EXTI2)                                    │
│    ISR executes                                                 │
│    Sets DW3000_IRQ_flag = true                                  │
│    Loop exits                                                   │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 6: CHECK TX STATUS                                        │
├─────────────────────────────────────────────────────────────────┤
│  Reset: DW3000_IRQ_flag = false                                 │
│  Read: SYS_STATUS (0x000044) - 4 bytes                         │
│  Check: TXFRS bit (bit 7) = 0x80                               │
│                                                                 │
│  If TXFRS = 1:                                                  │
│    ✓ TX SUCCESS                                                 │
│    - Clear interrupts                                           │
│    - Blink LED2 (50ms on, 950ms off)                          │
│    - Print: "TX done"                                           │
│                                                                 │
│  If TXFRS = 0:                                                  │
│    ✗ TX FAILED                                                  │
│    - Clear interrupts                                           │
│    - Print: "TX failed"                                         │
│    - Wait 1 second                                              │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
                   [Loop back to STEP 1]


════════════════════════════════════════════════════════════════════
                           RX FLOW
════════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────┐
│  STEP 1: CLEAR RX BUFFER                                        │
├─────────────────────────────────────────────────────────────────┤
│  memset(rx_buffer, 0, FRAME_LEN_MAX)                            │
│  Purpose: Clear previous received data                          │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 2: CLEAR INTERRUPTS                                       │
├─────────────────────────────────────────────────────────────────┤
│  Read: SYS_STATUS (0x000044) - 4 bytes                         │
│  Write: Same value back (W1C)                                   │
│  Reset: DW3000_IRQ_flag = false                                 │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 3: START RECEIVER                                         │
├─────────────────────────────────────────────────────────────────┤
│  Send Fast Command: CMD_RX (0x2)                                │
│  SPI Packet: 1 byte = 0x85                                      │
│                                                                 │
│  DW3000 State Machine:                                          │
│    IDLE → RX_WAIT → RX_PREAMBLE → RX_SFD → RX_PHR →           │
│    RX_DATA → IDLE                                               │
│                                                                 │
│  Reception Sequence:                                            │
│    1. Wait for preamble detection                               │
│    2. Search for SFD (Start Frame Delimiter)                   │
│    3. Receive PHR (Physical Header)                            │
│    4. Receive data payload                                      │
│    5. Validate CRC                                              │
│    6. Trigger interrupt if CRC good                            │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 4: WAIT FOR RX DONE                                       │
├─────────────────────────────────────────────────────────────────┤
│  Wait: while (!DW3000_IRQ_flag)                                 │
│                                                                 │
│  Hardware Interrupt:                                            │
│    UWB_IRQ pin rises (EXTI2)                                    │
│    ISR executes                                                 │
│    Sets DW3000_IRQ_flag = true                                  │
│    Loop exits                                                   │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 5: CHECK RX STATUS                                        │
├─────────────────────────────────────────────────────────────────┤
│  Reset: DW3000_IRQ_flag = false                                 │
│  Read: SYS_STATUS (0x000044) - 4 bytes                         │
│  Check: RXFR bit (bit 13) = 0x2000                             │
│                                                                 │
│  If RXFR = 1:                                                   │
│    ✓ RX SUCCESS                                                 │
│    - Clear interrupts                                           │
│    - Blink LED2 (50ms)                                          │
│    - Read frame length from RX_FINFO                           │
│    - Read data from RX_BUFFER                                   │
│    - Clear RXFCG flag                                           │
│    - Print received data                                        │
│                                                                 │
│  If RXFR = 0:                                                   │
│    ✗ RX FAILED or TIMEOUT                                       │
│    - Clear interrupts                                           │
│    - Print: "RX failed"                                         │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 5a: READ FRAME LENGTH                                     │
├─────────────────────────────────────────────────────────────────┤
│  Read: RX_FINFO (0x00006C) - 4 bytes                           │
│  Mask: RXFLEN bits (0-6 or 0-9)                                │
│  Result: frame_len (includes CRC)                               │
│                                                                 │
│  Example: frame_len = 12 (10 data + 2 CRC)                     │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 5b: READ RX DATA                                          │
├─────────────────────────────────────────────────────────────────┤
│  Read from: RX_BUFFER (0x120000)                                │
│  Length: frame_len - 2 (exclude CRC)                           │
│  Offset: 0                                                      │
│  Destination: rx_buffer[]                                       │
│                                                                 │
│  SPI Transaction:                                               │
│    CS LOW                                                       │
│    Send: 2-byte header [0x52, 0x00]                           │
│    Receive: 10 bytes of data                                    │
│    CS HIGH                                                      │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│  STEP 5c: CLEAR RXFCG FLAG                                      │
├─────────────────────────────────────────────────────────────────┤
│  Write to: SYS_STATUS (0x000044)                                │
│  Value: SYS_STATUS_RXFCG_BIT_MASK (0x4000)                     │
│  Effect: Clear RX frame CRC good flag                           │
└─────────────────────────────────────────────────────────────────┘
                            │
                            ▼
                   [Loop back to STEP 1]
```

---

## Register Summary Table

| Register Name | Address | Length | Key Bits/Fields | Purpose |
|--------------|---------|--------|-----------------|---------|
| DEV_ID | 0x000000 | 4 | Full register | Device identification |
| SYS_CFG | 0x000010 | 4 | PHR_MODE, PHR_6M8, DIS_FCE | System configuration |
| TX_FCTRL | 0x000024 | 4 | TXFLEN, TXBR, TXPSR, TR | TX frame control |
| SYS_ENABLE_LO | 0x00003C | 4 | TXFRS_ENABLE, RXFCG_ENABLE | Interrupt enables |
| SYS_STATUS | 0x000044 | 8 | TXFRS, RXFR, RXFCG, CP_LOCK, SPIRDY, RCINIT | Status flags |
| SYS_STATE_LO | 0x000048 | 4 | bits 16-23 | State machine state |
| RX_FINFO | 0x00006C | 4 | RXFLEN | RX frame info |
| CHAN_CTRL | 0x11000C | 4 | RF_CHAN, TX_PCODE, RX_PCODE, SFD_TYPE | Channel configuration |
| SEQ_CTRL | 0x110008 | 4 | AINIT2IDLE, FORCE2IDLE | Sequencer control |
| LED_CTRL | 0x110024 | 4 | BLINK_EN | LED control |
| GPIO_MODE | 0x110010 | 4 | MSGP2_MODE, MSGP3_MODE | GPIO pin modes |
| TX_BUFFER | 0x140000 | 1024 | Full buffer | TX data buffer |
| RX_BUFFER | 0x120000 | 1024 | Full buffer | RX data buffer |

---

## Fast Commands

| Command | Code | Byte Value | Description |
|---------|------|------------|-------------|
| CMD_TX | 0x1 | 0x83 | Start transmission |
| CMD_RX | 0x2 | 0x85 | Start reception |
| CMD_TXRXOFF | 0x0 | 0x81 | Turn off TX/RX |
| CMD_CLR_IRQS | 0x12 | 0xA5 | Clear all interrupts |
| CMD_TX_W4R | 0xC | 0x99 | TX then wait for RX |

---

## Configuration Values Explained

### Channel 5 Configuration
- **Frequency**: ~6.5 GHz center frequency
- **Bandwidth**: 500 MHz
- **PRF**: 64 MHz (determined by preamble code 9)
- **Preamble Code**: 9 (optimized for 64 MHz PRF)

### Data Rate: 6.8 Mbps
- Fastest data rate available on DW3000
- Better for high-throughput applications
- Slightly reduced range compared to 850 kbps

### Preamble Length: 128 symbols
- Moderate length (trade-off between range and latency)
- Longer preambles → better range, more latency
- Shorter preambles → less range, less latency

### PAC Size: 8
- Matched to preamble length (128 symbols → PAC 8)
- Affects receiver sensitivity and acquisition time

---

## Timing Analysis

### TX Timing (approximate)
1. **Preamble**: 128 symbols × ~1 μs/symbol = ~128 μs
2. **SFD**: 8 symbols × ~1 μs/symbol = ~8 μs
3. **PHR**: ~8 μs (at 6.8M PHR rate)
4. **Data**: 10 bytes × 8 bits × (1/6.8M) = ~11.8 μs
5. **CRC**: 2 bytes × 8 bits × (1/6.8M) = ~2.4 μs

**Total TX Time**: ~158 μs per frame

### Main Loop Timing
- **TX Mode**: 1 second period (1 Hz transmission rate)
- **RX Mode**: Continuous listening (starts immediately after each RX)

---

## Error Handling

### Initialization Errors
- **Wrong Device ID**: System halts (infinite loop)
- **Not in IDLE_RC**: System halts
- **dwt_initialise fails**: System halts
- **dwt_configure fails**: System halts
- **PLL not locked**: System halts

### Runtime Errors
- **TX Failed**: Prints error, waits 1 second, retries
- **RX Failed**: Prints error, restarts receiver immediately

---

## Power Consumption Considerations

1. **IDLE_PLL State**: PLL is locked and running (higher power than IDLE_RC)
2. **TX**: High power during transmission
3. **RX**: Continuous high power (receiver always on)
4. **LED Blinking**: Additional current draw

For low-power applications, consider:
- Using IDLE_RC between operations
- Implementing sleep modes
- Reducing TX power
- Using frame timeouts on RX

---

## Next Steps / Improvements

1. **Add ranging functionality**: Measure distance between devices
2. **Implement timestamp reading**: For precise timing applications
3. **Add encryption**: Secure the wireless link
4. **Implement MAC layer**: Add addressing, ACKs, retries
5. **Power optimization**: Add sleep/wake cycles
6. **Error correction**: Add FEC (Forward Error Correction)
7. **Multi-device support**: Device addressing and network topology

---

## References

- DW3000 User Manual (Qorvo)
- DW3000 API Guide
- STM32F7 HAL Reference
- IEEE 802.15.4 Standard (UWB PHY)

---

## File Locations

- **Main file**: `Src/main.c`
- **DW3000 driver**: `Src/DW3000_FZ.c`, `Inc/DW3000_FZ.h`
- **TX functions**: `Src/DW3000_send_test_FZ.c`, `Inc/DW3000_send_test_FZ.h`
- **RX functions**: `Src/DW3000_recv_test_FZ.c`, `Inc/DW3000_recv_test_FZ.h`
- **Register definitions**: `Inc/dw3000_deca_regs.h`
- **Register values**: `Inc/dw3000_deca_vals.h`

---

**Document Version**: 1.0
**Last Updated**: 2025-11-09
**Author**: Generated from source code analysis

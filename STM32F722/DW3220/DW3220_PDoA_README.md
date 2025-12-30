# DW3220 Phase Difference of Arrival (PDoA) Implementation

## Overview

This document explains how the `DW3220_PDoA_ref.c` file implements Phase Difference of Arrival (PDoA) measurement using the Qorvo/Decawave DW3220 Ultra-Wideband (UWB) transceiver to obtain orientation measurements.

## What is PDoA?

**Phase Difference of Arrival (PDoA)** is a technique that measures the phase difference between signals received on two antennas to determine the angle of arrival (AoA) of the signal. This allows the system to determine the orientation/direction of a transmitter relative to the receiver.

The DW3220 has two receive paths that can measure the phase difference between signals arriving at two antennas, which is then used to calculate the angle of arrival.

## System Architecture

The code implements a **two-board ranging system**:

- **ROAMER (Board 0)**: Responds to ping packets with delayed transmissions
- **BASE (Board 1)**: Initiates ranging, receives responses, and calculates PDoA
- **BASE2 (Board 2)**: Optional third board for phase delay testing (currently disabled)

## PDoA Configuration Steps

### 1. Radio Initialization (`init_radio()` - Lines 877-1178)

The initialization sequence configures the DW3220 for PDoA operation:

#### Step 1: Hardware Reset (Lines 880-884)
```c
RESET_LAT = 0;      // Pull RESET low
RESET_TRIS = 0;     // Configure as output
delay(1);           // Wait 1ms
RESET_TRIS = 1;     // Release reset (high-Z)
delay(2);           // Wait 2ms for chip to initialize
```
**Operation**: Hard reset of the DW3220 chip

---

#### Step 2: OTP Memory Configuration (Lines 907-921)
```c
uint32_t ldo_low = readOTP(0x04);      // Read from OTP address 0x04
uint32_t ldo_high = readOTP(0x05);     // Read from OTP address 0x05
uint32_t bias_tune = readOTP(0x0A);    // Read from OTP address 0x0A
```

**Registers Used**:
- `OTP_IF_REG (0x0B)` base register
  - Sub-address `0x04`: OTP address to read
  - Sub-address `0x08`: OTP control (value: 0x02 to trigger read)
  - Sub-address `0x10`: OTP data output

**Operations**:
- **READ**: OTP addresses 0x04, 0x05, 0x0A (calibration values)
- **WRITE**: Bias tune value to register `0x11:0x1F`
- **WRITE**: `0x0100` to `0x0B:0x08` if OTP values are valid

---

#### Step 3: Crystal Trim Configuration (Lines 923-929)
```c
uint32_t xtrim_value = readOTP(0x1E);  // Read crystal trim from OTP
if(xtrim_value == 0)
    xtrim_value = 0x2e;                 // Default if OTP is blank
write(FS_CTRL_REG, XTAL_TRIM, xtrim_value);
```

**Registers Used**:
- `FS_CTRL_REG (0x09):XTAL_TRIM (0x14)`

**Operations**:
- **READ**: OTP address 0x1E
- **WRITE**: Crystal trim value to `0x09:0x14`

---

#### Step 4: **CRITICAL PDoA System Configuration** (Lines 931-944)

This is where PDoA mode is enabled:

```c
uint32_t usr_cfg = ((uint32_t)STDRD_SYS_CONFIG & 0xFFF) |
    (PHR_MODE << 3) |
    (PHR_RATE << 4);

#ifdef ENABLE_PDOA
#if (BOARD == BASE)
    usr_cfg |= (((uint32_t)3) << 16);  // Enable PDOA mode (bits 17:16 = 0b11)
#endif
    // Set STS packet position (CP_SPC) and fixed STS code (CP_SDC)
    usr_cfg |= (((uint32_t)1) << 12) | (((uint32_t)1) << 15);
#endif

write(GEN_CFG_AES_LOW, SYS_CFG, usr_cfg);
```

**Register**: `GEN_CFG_AES_LOW (0x00):SYS_CFG (0x10)`

**Operations**:
- **WRITE** to `0x00:0x10` with configuration value

**Key Bits Set**:
- **Bit 17:16 = 0b11 (value 3)**: `PDOA_MODE` - Enables PDoA Mode 3 (dual antenna receive)
- **Bit 12 = 1**: `CP_SPC` - STS packet position configuration
- **Bit 15 = 1**: `CP_SDC` - Use fixed/static STS code
- **Bits 3-4**: PHR mode and rate settings
- **Bits 0-11**: Standard system configuration (0x188)

**Why this matters**: This configuration tells the DW3220 to:
1. Use both receive paths for phase difference measurement
2. Enable STS (Scrambled Timestamp Sequence) for secure ranging
3. Configure STS for PDoA operation

---

#### Step 5: **STS Length Configuration for PDoA** (Lines 957-968)

PDoA requires specific STS (Scrambled Timestamp Sequence) configuration:

```c
#ifdef ENABLE_PDOA
    // Change STS length to 128 symbols (required for PDOA)
    uint32_t sts_cfg = read(STS_CFG_REG, STS_CFG);        // Read current config
    sts_cfg = (sts_cfg & 0xff00) | (128 / 8 - 1);         // Set to 128/8-1 = 15
    write_len(STS_CFG_REG, STS_CFG, sts_cfg, 2);          // Write back
```

**Register**: `STS_CFG_REG (0x02):STS_CFG (0x00)`

**Operations**:
- **READ**: Current STS configuration from `0x02:0x00`
- **WRITE**: Modified value (length = 2 bytes) to `0x02:0x00`

**Value**: `(128 / 8 - 1) = 15` in lower byte
- STS length is specified in 8-symbol blocks
- 128 symbols = 16 blocks, encoded as 15 (16-1)

```c
    // Set STS_MNTH (STS threshold) for new STS length
    sts_cfg = read(CIA_REG3, STS_CONF_0);                 // Read STS config
    sts_cfg = (sts_cfg & 0xff80ffff) | (12 << 16);        // Set threshold to 12
    write(CIA_REG3, STS_CONF_0, sts_cfg);                 // Write back
```

**Register**: `CIA_REG3 (0x0E):STS_CONF_0 (0x12)`

**Operations**:
- **READ**: Current STS configuration from `0x0E:0x12`
- **WRITE**: Modified value to `0x0E:0x12`

**Value**: Bit 16-22 = 12 (STS match threshold)

```c
    // Adjust phase offset for better readability
    res = read(CIA_REG3, CIA_ADJUST);                     // Read current adjustment
    write(CIA_REG3, CIA_ADJUST, (res & 0xc000) | 3600);   // Set phase offset
#endif
```

**Register**: `CIA_REG3 (0x0E):CIA_ADJUST (0x1A)`

**Operations**:
- **READ**: Current CIA adjustment from `0x0E:0x1A`
- **WRITE**: Phase offset value 3600 (preserving bits 15:14)

**Purpose**: Fine-tunes the phase offset calculation to improve PDoA accuracy and readability

---

#### Step 6: Channel and Preamble Configuration (Lines 980-995)

```c
uint32_t chan_ctrl_val = read(GEN_CFG_AES_HIGH, CHAN_CTRL);
chan_ctrl_val &= (~0x1FFF);                    // Clear channel config bits
chan_ctrl_val |= CHANNEL;                       // RF_CHAN = 9 (CHANNEL_9 = 0x1)
chan_ctrl_val |= ((uint32_t)PREAMBLE_CODE) << 8;  // RX_PCODE = 9
chan_ctrl_val |= ((uint32_t)PREAMBLE_CODE) << 3;  // TX_PCODE = 9

#ifdef ENABLE_PDOA
    chan_ctrl_val |= 3 << 1;                   // SFD_TYPE = 3 (required for STS)
#endif

write(GEN_CFG_AES_HIGH, CHAN_CTRL, chan_ctrl_val);
```

**Register**: `GEN_CFG_AES_HIGH (0x01):CHAN_CTRL (0x14)`

**Operations**:
- **READ**: Current channel control from `0x01:0x14`
- **WRITE**: Modified configuration to `0x01:0x14`

**Configuration**:
- **Bits 1:2 = 3**: `SFD_TYPE` - STS-compatible SFD
- **Bits 3-7 = 9**: TX Preamble Code
- **Bits 8-12 = 9**: RX Preamble Code
- **Bits 0 = 1**: Channel 9 (CHANNEL_9)

---

#### Step 7: Additional Radio Configuration (Lines 997-1167)

Configures RF parameters, PLL, calibration, and receive tuning for optimal performance.

Key registers written:
- `TX_FCTRL (0x00:0x24)`: Preamble length, data rate
- `RF_TX_CTRL_2 (0x07:0x1C)`: Channel 9 specific value `0x1C010034`
- `PLL_CFG (0x09:0x00)`: Channel 9 PLL config `0x0F3C`
- `RX_CTRL_HI (0x07:0x10)`: Channel 9 RX config `0x08B5A833`
- RX calibration sequence (registers `0x04:0x0C` through `0x04:0x20`)
- DGC (Digital Gain Control) lookup tables for Channel 9

---

## PDoA Measurement Process

### BASE Board Operation (Lines 1322-1481)

When PDoA is enabled, the BASE board performs single-sided ranging with PDoA measurement:

#### Step 1: Transmit Ping (Lines 1402-1404)
```c
SET_TX_FRAME(PING_PACKET)    // Write packet type to TX buffer
setFrameLength(1);            // Set frame length to 1 byte
tx_instant_rx();              // Transmit and immediately switch to RX
```

**Operations**:
- **WRITE**: `PING_PACKET (0)` to `TX_BUFFER (0x14):0x00`
- **WRITE**: Frame length to `TX_FCTRL (0x00:0x24)`
- **WRITE**: Fast command `0x0C` (TX then RX)

---

#### Step 2: Wait for Response (Lines 1406-1415)

Waits up to 10ms for a response from the ROAMER.

---

#### Step 3: **Read PDoA Value** (Lines 1437-1444)

**This is the key PDoA measurement step:**

```c
#ifdef ENABLE_PDOA
    // Read PDOA value (14-bit phase difference)
    uint32_t pdoa = (read(CIA_REG1, PDOA) & 0x3fff) |
                    (read(STS_CFG_REG, STS_STS) << 16);

    // Check if STS timeout occurred (quality indicator)
    if((read(CIA_REG1, STS_TS + 4) >> 23) ||
       (read(CIA_REG1, STS1_TS + 4) >> 23))
        pdoa |= 0x8000;  // Set error flag
#endif
```

**Registers Read**:

1. **`CIA_REG1 (0x0C):PDOA (0x1E)`** - PDoA value register
   - **READ**: 4 bytes from `0x0C:0x1E`
   - **Value**: Bits 0-13 contain the **14-bit phase difference measurement**
   - **Units**: Phase difference in units that map to angle of arrival
   - Masked with `0x3fff` to extract 14 bits

2. **`STS_CFG_REG (0x02):STS_STS (0x08)`** - STS quality/status
   - **READ**: 4 bytes from `0x02:0x08`
   - **Value**: STS quality indicator
   - Shifted to bits 16-31 of output
   - **Purpose**: Indicates quality of STS correlation (must be >60% of STS length)

3. **`CIA_REG1 (0x0C):STS_TS (0x08) + 4`** - First STS timestamp upper byte
   - **READ**: Check bit 23 of upper timestamp
   - **Purpose**: STS_TOAST flag - indicates STS timeout on first antenna

4. **`CIA_REG1 (0x0C):STS1_TS (0x10) + 4`** - Second STS timestamp upper byte
   - **READ**: Check bit 23 of upper timestamp
   - **Purpose**: STS_TOAST flag - indicates STS timeout on second antenna

**PDoA Data Format** (32-bit value transmitted to host):
- **Bits 0-13**: Phase difference measurement (14 bits)
- **Bit 15**: Error flag (set if either STS timeout occurred)
- **Bits 16-31**: STS quality value

---

#### Step 4: Transmit Results (Lines 1459-1466)

```c
send_uart(0xff);              // Sync byte
send_uart(0xd2);              // Packet type identifier
print_bin32(result_lo);       // Round trip time (40-bit low 32 bits)
print_bin32(reply_delay);     // ROAMER's reply delay
print_bin32(clock_offset);    // Clock offset compensation
#ifdef ENABLE_PDOA
    print_bin32(pdoa);        // *** PDoA measurement ***
#endif
```

**Output Format** (when PDoA enabled):
- Sync: `0xFF 0xD2`
- 4 bytes: Round trip time
- 4 bytes: Reply delay from ROAMER
- 4 bytes: Clock offset
- **4 bytes: PDoA measurement** (phase difference + quality + error flags)

---

### ROAMER Board Operation (Lines 1236-1318)

The ROAMER responds to ping packets with delayed transmission:

#### Step 1: Receive Ping (Lines 1239-1241)
```c
STANDARD_RX                           // Fast command: Start RX
while(!(rx_status = receivedFrameSucc())) ;  // Wait for packet
clear_system_status();                // Clear status flags
```

---

#### Step 2: Delayed Transmission Response (`delayedTX()` - Lines 743-811)

```c
void delayedTX()
{
    readRXTimestamp();                    // Get RX timestamp (40-bit)
    uint32_t rx_lo = result_lo;
    uint8_t rx_hi = result_hi;

    // Calculate next TX time: rx_ts + TRANSMIT_DELAY
    add40(rx_hi, rx_lo, 0, TRANSMIT_DELAY);  // Add 6ms delay
    uint32_t next_tx_lo = result_lo;
    uint8_t next_tx_hi = result_hi;

    // Convert 40-bit to 32-bit register value (shift right 8 bits)
    uint32_t exact_tx_timestamp = next_tx_lo >> 8;
    exact_tx_timestamp |= ((uint32_t)next_tx_hi) << 24;

    // Quantize to DX_TIME_MASK granularity
    uint32_t calc_tx_lo = next_tx_lo & DX_TIME_MASK;  // 0xFFFFFE00
    uint8_t calc_tx_hi = next_tx_hi;

    // Calculate reply delay (for ranging calculation)
    sub40(calc_tx_hi, calc_tx_lo, rx_hi, rx_lo);
    uint32_t reply_delay = result_lo;

    // Set payload with reply delay
    write(TX_BUFFER, 0x00, reply_delay);
    setFrameLength(4);

    // Write transmit time
    WRITE_TX_DELAY(exact_tx_timestamp);  // Write to DX_TIME (0x00:0x2C)
    DELAYED_TX                            // Fast command: Delayed TX

    LED_LAT = !LED_LAT;                  // Toggle LED

    while(!(tx_status = sentFrameSucc())) ;  // Wait for TX complete
    clear_system_status();
}
```

**Registers Used**:

1. **RX Timestamp** - `CIA_REG1 (0x0C):IP_TS (0x00)`
   - **READ**: 5 bytes (40-bit timestamp)
   - **Operations**: Read from `0x0C:0x00` (4 bytes) and `0x0C:0x04` (1 byte)

2. **TX Delay Register** - `GEN_CFG_AES_LOW (0x00):DX_TIME (0x2C)`
   - **WRITE**: 4 bytes (32-bit delayed TX time)
   - **Value**: Calculated exact_tx_timestamp
   - **Units**: Time in units of ~15.65 picoseconds

3. **TX Buffer** - `TX_BUFFER (0x14):0x00`
   - **WRITE**: Reply delay value (4 bytes)

4. **TX Frame Control** - `TX_FCTRL (0x00:0x24)`
   - **WRITE**: Frame length configuration

5. **Fast Command**: `0x03` (Delayed TX)
   - Triggers transmission at the programmed DX_TIME

---

## How PDoA Provides Orientation

### Physical Principle

The DW3220 has **two receive paths** connected to two antennas separated by a known distance. When a signal arrives at an angle θ:

1. The signal reaches the two antennas at slightly different times
2. This time difference causes a **phase shift** between the received signals
3. The chip measures this phase difference using the STS portion of the packet
4. The 14-bit PDoA value represents this phase difference

### Angle Calculation

The angle of arrival can be calculated from the phase difference:

```
θ = arcsin(λ × φ / (2π × d))

Where:
  θ = Angle of arrival
  λ = Wavelength of RF signal (≈ 3.75 cm for Channel 9 at ~8 GHz)
  φ = Phase difference in radians (PDoA value scaled)
  d = Antenna separation distance
```

### PDoA Value Interpretation

The 14-bit PDoA register value (0-16383) maps to phase difference:
- **0 to 8191**: Phase difference from -180° to +180°
- **8192**: Corresponds to 0° phase difference (broadside arrival)
- Wraps around at ±180°

The actual mapping depends on:
- RF channel (affects wavelength)
- Antenna spacing
- Calibration offset (configured at line 972: CIA_ADJUST = 3600)

---

## Register Summary Table

| Register | Base:Sub | Operation | Purpose | Value |
|----------|----------|-----------|---------|-------|
| **PDoA Configuration** |
| SYS_CFG | 0x00:0x10 | WRITE | Enable PDoA Mode | Bits 17:16 = 0b11 |
| SYS_CFG | 0x00:0x10 | WRITE | STS packet position | Bit 12 = 1 |
| SYS_CFG | 0x00:0x10 | WRITE | Fixed STS code | Bit 15 = 1 |
| STS_CFG | 0x02:0x00 | READ/WRITE | STS length | 15 (=128 symbols) |
| STS_CONF_0 | 0x0E:0x12 | READ/WRITE | STS threshold | Bits 16-22 = 12 |
| CIA_ADJUST | 0x0E:0x1A | READ/WRITE | Phase offset | 3600 |
| CHAN_CTRL | 0x01:0x14 | READ/WRITE | SFD type for STS | Bits 1:2 = 3 |
| **PDoA Measurement** |
| PDOA | 0x0C:0x1E | READ | Phase difference | 14-bit value |
| STS_STS | 0x02:0x08 | READ | STS quality | Quality % |
| STS_TS+4 | 0x0C:0x0C | READ | STS timeout flag 1 | Bit 23 |
| STS1_TS+4 | 0x0C:0x14 | READ | STS timeout flag 2 | Bit 23 |
| **Timing Registers** |
| IP_TS | 0x0C:0x00 | READ | RX timestamp | 40-bit value |
| TX_TIME | 0x00:0x74 | READ | TX timestamp | 40-bit value |
| DX_TIME | 0x00:0x2C | WRITE | Delayed TX time | 32-bit value |
| **Support Registers** |
| TX_BUFFER | 0x14:0x00 | WRITE | Transmit data | Payload |
| RX_BUFFER_0 | 0x12:0x00 | READ | Received data | Payload |
| SYS_STATUS | 0x00:0x44 | READ | System status | Status flags |

---

## Configuration Constants

From lines 254-261:

```c
#define CHANNEL         CHANNEL_9           // 0x1 (≈8 GHz, λ≈3.75cm)
#define PREAMBLE_LENGTH PREAMBLE_128       // 5 (128 symbols)
#define PREAMBLE_CODE   9                  // Preamble code
#define PAC             PAC8               // 0x00
#define DATARATE        DATARATE_6_8MB     // 0x1 (6.8 Mbps)
#define PHR_MODE        PHR_MODE_STANDARD  // 0x0
#define PHR_RATE        PHR_RATE_850KB     // 0x0
```

**PDoA-Specific**:
- **STS Length**: 128 symbols (line 961) - Required for PDoA
- **PDoA Mode**: 3 (line 937) - Dual antenna mode
- **SFD Type**: 3 (line 992) - STS-compatible SFD

---

## Data Flow Diagram

```
BASE Board                          ROAMER Board
──────────                          ────────────

1. TX: PING packet
   └─> [0x14:0x00] = 0x00
   └─> Fast cmd 0x0C (TX→RX)
                                    2. RX: Receive PING
                                       └─> Read [0x12:0x00]
                                       └─> Read RX timestamp [0x0C:0x00]

                                    3. Calculate delayed TX time
                                       └─> Write DX_TIME [0x00:0x2C]
                                       └─> Write reply_delay [0x14:0x00]
                                       └─> Fast cmd 0x03 (Delayed TX)

4. RX: Receive PONG              ←─ 4. TX: PONG with reply_delay
   └─> Read [0x12:0x00]

5. **READ PDOA**:
   ├─> Read PDOA [0x0C:0x1E]        ← Phase difference (14-bit)
   ├─> Read STS_STS [0x02:0x08]     ← Quality
   ├─> Read STS timeout flags       ← Error checking
   └─> Combine into 32-bit result

6. Calculate range & orientation
   └─> Output via UART:
       [0xFF][0xD2][round_trip][reply_delay][clock_offset][pdoa]
```

---

## Compilation Options

From lines 61-66:

```c
#define DEBUG              // Enable debug output
//#define ENABLE_PHASE     // 2 board phase delay (disabled)
#define ENABLE_PDOA        // *** ENABLE PDoA MEASUREMENT ***
//#define ENABLE_DS        // Double-sided ranging (disabled)
```

**To enable PDoA**: Uncomment `#define ENABLE_PDOA` (currently enabled)

When `ENABLE_PDOA` is defined:
- BASE board enables PDoA mode in SYS_CFG
- STS length is configured to 128 symbols
- PDoA register is read after each successful reception
- PDoA value is transmitted in output packet

---

## Key Timing Parameters

```c
#define TRANSMIT_DELAY 0x165a0bc0   // 6ms (line 202)
#define DX_TIME_MASK   0xfffffe00   // 9-bit quantization (line 205)
```

- **TRANSMIT_DELAY**: ROAMER waits 6ms after receiving before transmitting response
- **DX_TIME_MASK**: Masks lower 9 bits for delayed transmission quantization
- Time unit: ~15.65 picoseconds (1/(499.2 MHz × 128))

---

## Communication Protocol

### Packet Format (Single-Sided Ranging)

**PING** (BASE → ROAMER):
- Byte 0: `0x00` (PING_PACKET)

**PONG** (ROAMER → BASE):
- Bytes 0-3: `reply_delay` (32-bit)

### Output Format (BASE → UART)

```
[0xFF][0xD2][round_trip][reply_delay][clock_offset][pdoa]
   │     │        │            │             │         │
   │     │        │            │             │         └─ 4 bytes: PDoA measurement
   │     │        │            │             └─────────── 4 bytes: Clock offset
   │     │        │            └───────────────────────── 4 bytes: Reply delay
   │     │        └────────────────────────────────────── 4 bytes: Round trip time
   │     └─────────────────────────────────────────────── Packet type
   └───────────────────────────────────────────────────── Sync byte
```

Total: 18 bytes per measurement when PDoA is enabled
Total: 14 bytes per measurement when PDoA is disabled

---

## How to Use PDoA Data

### 1. Extract Phase Difference
```c
uint16_t phase_raw = pdoa & 0x3fff;          // Extract 14-bit phase
bool error = (pdoa & 0x8000) != 0;           // Check error flag
uint16_t quality = (pdoa >> 16) & 0xFFFF;    // Extract STS quality
```

### 2. Convert to Angle (Approximate)
```c
// Assuming antenna spacing d and wavelength λ
float phase_rad = (phase_raw - 8192) * M_PI / 8192.0;  // Convert to radians
float angle_rad = asin(wavelength * phase_rad / (2 * M_PI * antenna_spacing));
float angle_deg = angle_rad * 180.0 / M_PI;
```

### 3. Quality Check
```c
uint16_t sts_length = 128;  // Configured STS length
uint16_t quality_threshold = (sts_length * 60) / 100;  // 60% threshold

if (quality >= quality_threshold && !error) {
    // Good measurement
} else {
    // Poor quality or timeout - discard
}
```

---

## Important Notes

1. **PDoA only works on BASE board**: ROAMER doesn't enable PDoA mode (line 936 check)

2. **STS is critical**: The Scrambled Timestamp Sequence (STS) is what allows secure and accurate phase measurement

3. **Quality matters**: Always check STS_STS quality value (should be >60% of STS length)

4. **Calibration required**: The CIA_ADJUST value (3600) is a calibration offset that may need adjustment for your specific hardware

5. **Channel dependency**: Phase-to-angle conversion depends on RF wavelength (different for Channel 5 vs Channel 9)

6. **Antenna spacing**: The actual angle calculation requires knowing the physical distance between the two receive antennas on the DW3220 module

7. **Ambiguity**: Phase wraps around every 360°, creating potential ambiguity in angle measurement

---

## References

- **DW3000 User Manual**: Chapter 6 (PDoA), Chapter 9 (Fast Commands), Chapter 11 (Registers)
- **Register Definitions**: `dw3000_deca_regs.h` and `dw3000_deca_vals.h`
- **Source Code**: Based on Adam Williams' implementation for PIC16F1508
- **Original Library**: https://github.com/Fhilb/DW3000_Arduino

---

## Troubleshooting

### No PDoA readings (all zeros)
- Check that `ENABLE_PDOA` is defined
- Verify BOARD is set to BASE
- Confirm SYS_CFG register bit 16:17 = 0b11

### High error rate (bit 15 set frequently)
- Check STS quality value (bits 16-31)
- Verify STS length is 128 symbols
- Check signal strength/distance
- Review antenna configuration

### Inconsistent angle measurements
- Calibrate CIA_ADJUST offset (register 0x0E:0x1A)
- Verify antenna spacing matches calculation
- Check for multipath interference
- Ensure good STS quality (>60%)

---

## Summary

The DW3220_PDoA_ref.c implements Phase Difference of Arrival by:

1. **Configuring the DW3220** to use dual-antenna PDoA mode (SYS_CFG bits 16-17)
2. **Setting up STS** with 128 symbols and proper thresholds
3. **Measuring phase difference** between two receive antennas using the PDOA register
4. **Reading quality indicators** (STS_STS) and timeout flags
5. **Transmitting the PDoA value** along with ranging data

The 14-bit phase difference can be converted to angle of arrival using trigonometry and knowledge of the RF wavelength and antenna spacing, providing orientation information about the transmitter relative to the receiver.

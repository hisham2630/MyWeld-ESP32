# ⚡ MyWeld ESP32 Firmware

> Supercapacitor spot welder controller firmware for the **Guition JC3248W535** development board (ESP32-S3, 480×320 TFT, AXS15231B touch).

[![Platform](https://img.shields.io/badge/Platform-ESP32--S3-blue)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Framework](https://img.shields.io/badge/Framework-ESP--IDF%205.x-red)](https://docs.espressif.com/projects/esp-idf/)
[![Build](https://img.shields.io/badge/Build-PlatformIO-orange)](https://platformio.org/)
[![LVGL](https://img.shields.io/badge/UI-LVGL%209.x-green)](https://lvgl.io/)
[![Version](https://img.shields.io/badge/Version-1.0.0-brightgreen)](#)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Architecture](#architecture)
- [Features](#features)
- [Weld Pulse Sequence](#weld-pulse-sequence)
- [BLE Protocol](#ble-protocol)
- [Pin Assignments](#pin-assignments)
- [Getting Started](#getting-started)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Safety Design](#safety-design)
- [Acknowledgments](#acknowledgments)

---

## Overview

**MyWeld** is a DIY supercapacitor spot welder controlled by an ESP32-S3 microcontroller, adapted from the [**MyWeld V2.0 PRO**](https://www.youtube.com/@akakasyan) by **Aka Kasyan**. The firmware provides:

- A full-color touchscreen UI (LVGL 9.x, 480×320) for real-time control and monitoring
- A dual-mode welding engine — **Manual** (button press) and **Auto** (contact detection)
- A **P1 / T / P2 dual-pulse** sequence for professional weld quality
- **BLE companion app** connectivity (Android) via a custom binary protocol
- Up to **10 named presets** stored in NVS (flash)
- **I2S audio** feedback (startup melody, beeps, error tones)
- Real-time **supercapacitor voltage monitoring** with voltage-based weld blocking

---

## Hardware

| Component | Part |
|---|---|
| **MCU Board** | Guition JC3248W535 (ESP32-S3, 16 MB Flash, PSRAM) |
| **Display** | 3.5″ 480×320 TFT, QSPI, AXS15231B controller |
| **Touch** | Capacitive touch via I2C (addr `0x3B`) |
| **Audio** | Built-in I2S amplifier → speaker (P6 header) |
| **Supercap Bank** | 2S2P configuration, 3000 F / 3.0 V per cell, max 5.7 V |
| **Output** | MOSFET bank fire signal (`GPIO 46`) |
| **Charger** | Supercap charger enable (`GPIO 16`, active-LOW) |
| **Weld Button** | External trigger (`GPIO 14`, active-LOW, pull-up) |

---

## Architecture

The firmware runs **5 FreeRTOS tasks** pinned to two cores:

```
Core 0                          Core 1
──────────────────────          ────────────────────────────
ui_task      (priority 5)       welding_task  (priority 10)  ← highest
ble_task     (priority 1)       adc_task      (priority  3)
                                audio_task    (priority  2)
```

**Core 0** handles all UI rendering (LVGL) and BLE communication.  
**Core 1** handles time-critical welding pulses, ADC voltage sampling, and audio output.

### Initialization Order (Safety-Critical)

```
1. GPIO safe defaults  ← MOSFET = LOW immediately on boot
2. NVS init           ← Load saved settings / presets
3. I2S audio init
4. Display + LVGL init
5. UI build
6. Welding state machine init
7. BLE serial init
8. Launch FreeRTOS tasks
```

---

## Features

### ✅ Dual Welding Modes

| Mode | Trigger | Description |
|------|---------|-------------|
| **MAN** (Manual) | Physical button (`GPIO 14`) | Press and hold to fire |
| **AUTO** | Electrode contact detection (`GPIO 7`) | Auto-fires after configurable delay `S` |

### ✅ P1/T/P2 Dual-Pulse Sequence

```
[P1 pulse] ──── [T pause] ──── [P2 pulse]
     │                               │
  0.0–50 ms                      0.0–50 ms
       (P2 = 0 → single pulse mode)
```

### ✅ Real-Time Status Display

- Supercapacitor voltage bar (0–5.7 V with color coding)
- Charge percentage
- Weld state indicator (IDLE / ARMED / FIRING / BLOCKED / ERROR)
- Session and lifetime weld counters
- Active preset name

### ✅ Preset Management

- 10 named presets (up to 20 chars each)
- Stores: P1, T, P2, S, mode (AUTO/MAN)
- Instantly loadable from UI or Android app

### ✅ BLE Remote Control

- Device advertises as `"MyWeld"` (configurable display name)
- PIN authentication (default: `1234`)
- Real-time status notifications every ~500 ms
- Read/write all weld parameters remotely
- Load/save presets remotely

### ✅ Audio Feedback

| Tone | Event |
|------|-------|
| Startup melody (C–E–G) | Boot complete |
| Double beep (D5–A5) | BLE connected |
| Single beep (880 Hz) | Parameter changed |
| High beep (1200 Hz) | Pulse fired |
| Error melody | Fault / blocked |
| Contact tone (1000 Hz) | AUTO contact detected |

### ✅ Safety Features

- MOSFET output driven **LOW on boot** before any other init
- Low-voltage block: refuses to weld below **3.0 V**
- Low-voltage warning: at **4.0 V**
- Charger disabled during pulse (prevents charging during discharge)
- ADC-based protection rail monitoring (gate-drive health check)
- NVS write debouncing (prevents flash wear)

---

## Weld Pulse Sequence

```
        ┌─ charger OFF ─────────────────────────┐
        │  settle 500µs                          settle 500µs
        │          ┌────────┐        ┌────────┐  │
OUTPUT  │          │  P1    │  T gap │  P2    │  │
────────┘          └────────┘        └────────┘  └── charger ON
        │←500µs→│←  P1  →│←T→│←  P2  →│←500µs→│
```

- **P1**: Pre-pulse (cleans surface, typically 3–8 ms)  
- **T**: Pause between pulses (typically 5–10 ms)  
- **P2**: Main pulse (fuses metal, typically 5–15 ms). Set to 0 for single-pulse mode.

---

## BLE Protocol

The firmware uses a custom **binary protocol V2** over BLE GATT.

### GATT Service

| Attribute | UUID |
|-----------|------|
| Service | `00001234-0000-1000-8000-00805F9B34FB` |
| Params Char (READ) | `00001235-0000-1000-8000-00805F9B34FB` |
| Status Char (NOTIFY) | `00001236-0000-1000-8000-00805F9B34FB` |
| Command Char (WRITE) | `00001237-0000-1000-8000-00805F9B34FB` |

### Packet Format

```
[SYNC=0xAA] [TYPE] [LEN] [PAYLOAD...] [CRC]
                                        └── XOR of TYPE + LEN + all PAYLOAD bytes
```

All multi-byte values are **little-endian**.

### Message Types

| Type | Direction | Description |
|------|-----------|-------------|
| `0x01` STATUS | ESP→App | 32-byte periodic status (via NOTIFY, ~500 ms) |
| `0x02` PARAMS_READ | App→ESP | Request current parameters |
| `0x03` PARAMS_RESPONSE | ESP→App | Current parameters (28 bytes) |
| `0x04` PARAMS_WRITE | App→ESP | Update parameters |
| `0x05` CMD | App→ESP | Execute command (sub-typed) |
| `0x06` ACK | ESP→App | Success response |
| `0x07` NAK | ESP→App | Error response |
| `0x08` VERSION | App→ESP | Request firmware version |
| `0x09` VERSION_RESPONSE | ESP→App | Firmware version info |
| `0x0D` AUTH_REQUEST | App→ESP | Authenticate with PIN |
| `0x0E` AUTH_RESPONSE | ESP→App | Authentication result |

### Commands (`0x05` CMD sub-types)

| Sub-type | Command |
|----------|---------|
| `0x01` | Load preset by index |
| `0x02` | Save current params as preset |
| `0x03` | Factory reset |
| `0x04` | Reset weld counter |
| `0x06` | Authenticate (PIN) |
| `0x07` | Change PIN |

---

## Pin Assignments

| Function | GPIO | Notes |
|----------|------|-------|
| MOSFET fire output | 46 | Active HIGH, pull-down |
| Charger enable | 16 | Active LOW |
| Weld button | 14 | Active LOW, pull-up |
| Supercap voltage ADC | 5 | ADC1_CH4, 10k+15k divider |
| Protection rail ADC | 6 | ADC1_CH5, 47k+15k divider |
| Contact detect ADC | 7 | ADC1_CH6 |
| LCD QSPI CLK | 47 | Internal, no user wiring |
| LCD QSPI CS | 45 | Internal |
| LCD TE | 38 | Internal |
| LCD Backlight | 1 | PWM |
| LCD D0–D3 | 21, 48, 40, 39 | Internal |
| Touch SCL | 8 | I2C |
| Touch SDA | 4 | I2C |
| I2S LRCLK (WS) | 2 | Audio |
| I2S BCLK | 42 | Audio |
| I2S DOUT | 41 | Audio |

---

## Getting Started

### Prerequisites

- [PlatformIO IDE](https://platformio.org/) (VS Code extension or CLI)
- ESP32-S3 board (Guition JC3248W535 or compatible)
- USB cable (USB-C)

### Build & Flash

```bash
# Clone the repo
git clone https://github.com/hisham2630/MyWeld-ESP32.git
cd MyWeld-ESP32

# Flash to board (release build)
pio run -e jc3248w535 --target upload

# Monitor serial output
pio device monitor -e jc3248w535
```

### Debug Build

```bash
# Build with full symbols + halt-on-panic
pio run -e debug --target upload && pio device monitor -e debug
```

The debug environment uses `-Og` optimisation and `esp32_exception_decoder` for readable backtraces.

---

## Project Structure

```
MyWeld-ESP32/
├── platformio.ini          # Build configuration (release + debug envs)
├── partitions.csv          # Custom 16MB partition table
├── sdkconfig.defaults      # ESP-IDF SDK defaults
├── sdkconfig.jc3248w535    # Board-specific sdkconfig
└── src/
    ├── main.c              # Entry point, task launcher
    ├── config.h            # All pin defs, thresholds, timing constants
    ├── display.c / .h      # QSPI display init, LVGL driver, backlight
    ├── ui.c / .h           # LVGL UI — screens, widgets, update callbacks
    ├── welding.c / .h      # Welding state machine + ADC task
    ├── ble_serial.c / .h   # NimBLE GATT server + protocol handler
    ├── ble_protocol.h      # Binary protocol definitions (shared with Android)
    ├── settings.c / .h     # NVS settings load/save, presets, PIN
    ├── audio.c / .h        # I2S tone generator + sound effects
    ├── esp_lcd_axs15231b.c # AXS15231B QSPI display driver
    ├── esp_lcd_touch.c     # Capacitive touch I2C driver
    └── lv_conf.h           # LVGL configuration
```

---

## Configuration

All hardware and behaviour constants are in [`src/config.h`](src/config.h):

| Constant | Default | Description |
|----------|---------|-------------|
| `SUPERCAP_MAX_V` | 5.7 V | Max charge voltage |
| `LOW_VOLTAGE_WARN` | 4.0 V | Low voltage UI warning |
| `LOW_VOLTAGE_BLOCK` | 3.0 V | Minimum voltage to weld |
| `PULSE_MAX_MS` | 50 ms | Maximum pulse duration |
| `S_VALUE_DEFAULT` | 0.5 s | Default AUTO delay |
| `MAX_PRESETS` | 10 | Number of preset slots |
| `BLE_DEVICE_NAME` | `"MyWeld"` | Default BLE advertised name |
| `PIN_DEFAULT` | `"1234"` | Factory PIN |
| `NVS_SAVE_DEBOUNCE_MS` | 2000 ms | Flash write debounce |
| `ADC_SAMPLE_INTERVAL` | 500 ms | Voltage sampling interval |

---

## Safety Design

> ⚠️ Supercapacitor spot welders store very high energy. The firmware includes multiple safety layers, but improper hardware wiring can still be dangerous. Always verify your hardware before operating.

1. **Boot-time GPIO safety** — `PIN_OUTPUT` is driven LOW **before any `app_main` logic runs** to prevent accidental MOSFET activation during boot.
2. **Voltage blocking** — Welding is hard-blocked when supercap voltage drops below `LOW_VOLTAGE_BLOCK` (3.0 V), preventing weak/incomplete welds.
3. **Protection rail monitoring** — The 13.5 V gate-drive rail is sampled continuously; a fault blocks welding.
4. **Charger interlock** — The charger is disabled during the pulse window to prevent current fighting.
5. **NVS debouncing** — Parameter saves are debounced (`NVS_SAVE_DEBOUNCE_MS`) to reduce flash wear.
6. **BLE authentication** — All parameter writes and commands require PIN authentication.

---

## Companion App

The Android companion app for remote control is available at:  
👉 **[github.com/hisham2630/MyWeld-Android](https://github.com/hisham2630/MyWeld-Android)**

---

## Acknowledgments

This project is an ESP32-S3 adaptation of the **MyWeld V2.0 PRO** spot welder, originally designed and built by **[Aka Kasyan](https://www.youtube.com/@akakasyan)**.

A huge thank you to Aka Kasyan for:
- 🔩 The original **MyWeld V2.0 PRO** hardware design and PCB layout
- 💻 The proven **welding logic** (dual-pulse P1/T/P2 sequence, protection systems)
- 📺 The excellent **YouTube tutorials** that made this project possible
- 🌍 Sharing his knowledge and inspiring the maker/DIY community

The original project was built around an **Arduino Nano** with a 20×4 LCD, rotary encoder, and 12× IRL40SC228 MOSFETs. This adaptation ports the welding logic to an **ESP32-S3** (Guition JC3248W535) with a full-color TFT touchscreen, BLE companion app, I2S audio, and 16× IXTP170N075T2 MOSFETs — while preserving the core pulse generation and safety principles from Aka Kasyan's original design.

---

## License

This project is open source. See [LICENSE](LICENSE) for details.

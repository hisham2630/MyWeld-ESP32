# MyWeld ESP32-S3 JC3248W535 Adaptation

## Goal
Adapt the MyWeld V2.0 PRO Arduino Nano spot welder code to run on the **JC3248W535** (ESP32-S3 + 480×320 TFT capacitive touch) with a full dashboard UI using **LovyanGFX + LVGL**, preserving all original welding logic while adding enhanced features.

---

## Decisions Summary

| Decision | Choice |
|----------|--------|
| **Framework** | **PlatformIO + ESP-IDF 5.x** — proven to work with JC3248W535/AXS15231B |
| **LVGL** | **v9.x** — latest API, active development |
| **UI Style** | **Full Dashboard** — all parameters + voltage bar + status on one rich screen |
| **Theme** | **Dark mode + Light mode** — switchable in settings |
| **MAN Weld Trigger** | **Physical button only** (IO14) — safety-first for 2000A system |
| **Charger Control** | **Dedicated CHARGER_EN** (IO16) — independent timing control |
| **Display Library** | **LovyanGFX + LVGL 9.x** — fast rendering + rich widget toolkit |
| **Encoder** | **REMOVED** — all parameter adjustment via touchscreen |
| **Audio** | **I2S speaker** via built-in amp — pleasant tones, not buzzer |
| **Pulse Resolution** | **0.5ms steps** (0, 0.5, 1.0, 1.5, ... 50ms) |
| **Bluetooth** | **BLE serial** — phone monitoring & parameter control |
| **Language** | English only |
| **Extra Features** | ALL: live voltage graph, weld counter, preset profiles (10 slots), color voltage bar, charging animation, settings page, ADC calibration, test mode |

---

## Hardware Changes from Original V2.0 PRO

| Original | Replacement |
|----------|-------------|
| Arduino Nano | JC3248W535 (ESP32-S3-WROOM-1, N16R8) |
| 20×4 I2C LCD | Built-in 480×320 TFT (AXS15231B driver, QSPI) |
| Rotary Encoder + Push Button | Capacitive touchscreen |
| EEPROM | NVS (Preferences library) |
| 12× IRL40SC228 MOSFETs | 16× IXTP170N075T2 MOSFETs |
| TC4422CPA gate driver | TC4428 (both channels paralleled via 10Ω resistors) |
| TPS61088 boost | 10A CC/CV buck-boost module (supercap charger, KEY pin for EN) |
| 7805 regulator | 3A buck-boost → 5V (logic), 1A buck → 13.5V (gate drive) |
| SMCJ13A TVS | 1.SKE12CA 12V bidirectional TVS |
| Gate resistors 4R7 | 10Ω per MOSFET |
| Supercap-powered logic | External 13V battery |
| — | 30SQ060 Schottky diode on charger output |
| — | 2N2222 transistor for KEY pin isolation |

---

## JC3248W535 Board Pin Assignments (VERIFIED ✅)

> Source: Official JC3248W535 GPIO割当表 (2025/02/01)

### Display (QSPI — internal, no user wiring)
| Function | GPIO |
|----------|------|
| CLK | 47 |
| CS | 45 |
| DC (RS) | NC |
| RST | NC |
| TE | 38 |
| BL (Backlight PWM) | 1 |
| DATA0 | 21 |
| DATA1 | 48 |
| DATA2 | 40 |
| DATA3 | 39 |

### Touch (I2C — internal, from schematic)
| Function | GPIO |
|----------|------|
| I2C SCL (TP_SCL) | 4 |
| I2C SDA (TP_SDA) | 8 |
| RST | NC (not connected) |
| INT | NC (not connected) |
| I2C Address | 0x3B |

> ⚠️ **Note:** The GPIO allocation table labels these as SDA=4/SCL=8, but the **schematic** clearly shows IO4→TP_SCL and IO8→TP_SDA. The schematic is the source of truth. In ESP32-S3 code, we configure with `Wire.begin(/*SDA=*/8, /*SCL=*/4)`.


### I2S Audio (built-in amplifier → P6 speaker connector)
| Function | GPIO |
|----------|------|
| CTRL (amp power) | VBAT (always on) |
| LRCLK (Word Select) | **2** |
| BCLK (Bit Clock) | **42** |
| DATA (I2S Data Out) | **41** |

> 🔊 **The board has a built-in I2S audio amplifier!** Connect a small speaker to **P6** (-SP, -VAT). This replaces the ugly buzzer with proper audio tones/melodies.

> ⚠️ **GPIO2, GPIO41, GPIO42 are NOT available as user IO** — they are wired to the I2S audio system.

### SD Card (TF slot — internal)
| Function | GPIO |
|----------|------|
| CLK | 12 |
| MOSI (CMD) | 11 |
| MISO (DAT0) | 13 |
| CS (CD/DAT3) | 10 |

> ⚠️ **GPIO 10, 11, 12, 13 are used by SD card** — NOT available for general IO even though they appear similar to user GPIOs.

### Board Connectors Reference
| Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 | Pin 7 | Pin 8 |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|
| **P1** (UART/Power) | +5VIN | TXD | RXD | GND | — | — | — | — |
| **P2** (8P User IO) | IO5 | IO6 | IO7 | IO15 | IO16 | IO46 | IO9 | IO14 |
| **P3** (4P IO) | GND | 3.3V | IO17 | IO18 | — | — | — | — |
| **P4** (4P IO) | GND | 3.3V | IO17 | IO18 | — | — | — | — |
| **P5** (Battery) | -VAT | +VAT | — | — | — | — | — | — |
| **P6** (Speaker) | -SP | -VAT | — | — | — | — | — | — |

> P3 and P4 are **duplicates** — both expose IO17 and IO18.

### Switches
| Switch | Action |
|--------|--------|
| **SW1** single press | ON |
| **SW1** double press | OFF |
| **RST** | Reset |
| **BOOT** | Boot mode |

### User GPIO Assignment for MyWeld (P2 header)
| Function | GPIO | Header Pin | Direction | ADC? | Notes |
|----------|------|------------|-----------|------|-------|
| **OUTPUT_PIN** (MOSFET fire) | **IO46** | P2 pin 6 | Digital OUT | No | Pure digital, no ADC conflict |
| **CHARGER_EN** | **IO16** | P2 pin 5 | Digital OUT | — | KEY pin control via 2N2222 |
| **START_PIN** (Weld button) | **IO14** | P2 pin 8 | INPUT_PULLUP | — | External physical weld trigger |
| **VOLTAGE_PIN** (Supercap V) | **IO5** | P2 pin 1 | ADC Input | ADC1_CH4 ✓ | Via 10k/15k divider |
| **PROTECTION_PIN** (13.5V) | **IO6** | P2 pin 2 | ADC Input | ADC1_CH5 ✓ | Via 47k/15k divider |
| **CONTACT_PIN** (Auto mode) | **IO7** | P2 pin 3 | ADC Input | ADC1_CH6 ✓ | 10kΩ pull-up + 100kΩ isolation |
| **SPEAKER** (I2S audio) | **IO2/42/41** | Built-in | I2S Output | — | Built-in amp → P6 speaker |
| *Spare* | **IO15** | P2 pin 4 | — | ADC2_CH4 | Available for future use |
| *Spare* | **IO9** | P2 pin 7 | — | ADC1_CH8 | Available for future use |
| *Spare* | **IO17** | P3/P4 | — | ADC2_CH6 | Available for future use |
| *Spare* | **IO18** | P3/P4 | — | ADC2_CH7 | Available for future use |

> ✅ All 3 ADC inputs use **ADC1** channels (GPIO5, 6, 7) — most reliable, no WiFi conflict.

> ✅ **4 spare GPIOs** remain (IO9, IO15, IO17, IO18) for future expansion.

### ADC Voltage Divider Calculations (3.3V ESP32-S3 ADC)

| Measurement | Range | Divider | ADC Voltage | Multiplier |
|-------------|-------|---------|-------------|------------|
| Supercap (0–5.7V) | 10k/15k | 0–2.28V | Safe ✓ | `V = ADC_raw × (3.3/4095) × (25/15)` |
| Protection (0–13.5V) | 47k/15k | 0–3.26V | Safe ✓ | `V = ADC_raw × (3.3/4095) × (62/15)` |
| Contact detect | 10kΩ pull-up | 0–3.3V | Safe ✓ | Threshold ~1.5V |

---

## Power Architecture

```
13V External Battery
    │
    ├──→ [1A Buck Module] ──→ 13.5V ──→ TC4428 VDD (gate drive, ALWAYS ON)
    │
    ├──→ [3A Buck Module] ──→  5.0V ──→ ESP32-S3 via P1 +5VIN (ALWAYS ON)
    │
    └──→ [10A CC/CV Buck-Boost] ──→ 5.5V/5A ──→ [30SQ060 Schottky] ──→ Supercaps (2S2P, 3.0V×2=6.0V, limit 5.7V)
                │                                                             │
               KEY pin ←── 2N2222 collector                                   │
                             │ base ← [1kΩ] ← IO16 (CHARGER_EN)              ↓
                            │ emitter ← GND                            MOSFET Bank (16× IXTP170N075T2)
                                                                              │
                                                                        Welding Output
```

### Disconnect Logic
- `CHARGER_EN = LOW` (idle) → transistor OFF → KEY floating → module ON → charging ✓
- `CHARGER_EN = HIGH` (weld) → transistor ON → KEY pulled to GND → module OFF → clean pulse ✓

---

## Software Architecture (PlatformIO + ESP-IDF 5.x)

```
MyWeld-ESP32/
├── platformio.ini                # Build config: ESP32-S3, ESP-IDF 5.x, LVGL 9.x
├── sdkconfig.defaults            # ESP-IDF defaults (PSRAM, flash, I2S, etc.)
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                    # Entry point: app_main()
│   ├── config.h                  # Pin definitions, constants, thresholds
│   ├── display.h / display.c     # LovyanGFX init + LVGL flush/input bridge
│   ├── ui.h / ui.c               # LVGL UI: screens, widgets, callbacks
│   ├── ui_theme.h / ui_theme.c   # Dark/Light mode theme definitions
│   ├── welding.h / welding.c     # Core welding logic (pulse, protection, contact)
│   ├── audio.h / audio.c         # I2S audio: tones, melodies, feedback
│   ├── settings.h / settings.c   # NVS read/write, preset profiles
│   ├── ble_serial.h / ble_serial.c # Bluetooth LE serial (phone control)
│   └── lv_conf.h                 # LVGL 9.x configuration
├── components/                   # ESP-IDF custom components if needed
│   └── lovyangfx/                # LovyanGFX as component
└── boards/
    └── jc3248w535.json            # Custom board definition
```

> **Why ESP-IDF instead of Arduino?** The AXS15231B display on JC3248W535 requires ESP-IDF 5.x I2C/SPI drivers that Arduino-ESP32 doesn't fully support yet. NorthernMan54's proven build uses ESP-IDF 5.3. We use C (not C++) for ESP-IDF consistency, but LovyanGFX is C++ — we wrap it.

### Core Modules

#### 1. `config.h` — Pin Definitions & Constants
```c
// GPIO Pins (JC3248W535 — VERIFIED from official pinout table + schematic)
#define OUTPUT_PIN      46   // MOSFET bank fire signal (P2 pin 6)
#define CHARGER_EN      16   // Supercap charger KEY control (P2 pin 5)
#define START_PIN       14   // Physical weld button, MAN mode (P2 pin 8)
#define VOLTAGE_PIN     5    // Supercap voltage ADC — ADC1_CH4 (P2 pin 1)
#define PROTECTION_PIN  6    // 13.5V rail protection ADC — ADC1_CH5 (P2 pin 2)
#define CONTACT_PIN     7    // Electrode contact detection ADC — ADC1_CH6 (P2 pin 3)
#define BACKLIGHT_PIN   1    // Display backlight PWM (built-in)

// I2S Audio (built-in amplifier → speaker on P6)
#define I2S_LRCLK       2    // I2S Word Select (WS)
#define I2S_BCLK        42   // I2S Bit Clock
#define I2S_DATA        41   // I2S Data Out

// Touch I2C (from schematic: IO4=SCL, IO8=SDA)
#define TOUCH_SCL       4
#define TOUCH_SDA       8

// ADC
#define ADC_RESOLUTION  12   // ESP32-S3: 12-bit (0–4095)
#define ADC_VREF        3.3f

// Voltage divider multipliers
#define SUPERCAP_V_MULT     (25.0f / 15.0f)   // 10k+15k divider
#define PROTECTION_V_MULT   (62.0f / 15.0f)   // 47k+15k divider

// Supercap Battery (2S2P, 3.0V 3000F each)
#define SUPERCAP_MAX_V          5.7f   // Max charge voltage (2×3.0V = 6.0V, derate to 5.7V)
#define SUPERCAP_FULL_V         5.5f   // Considered "fully charged" for UI
#define LOW_VOLTAGE_THRESHOLD   4.0f   // Low voltage warning (V)
#define CRITICAL_VOLTAGE        3.0f   // Refuse to weld below this (V)

// Protection thresholds
#define CONTACT_THRESHOLD       1.5f   // Contact detection (V)
#define PROTECTION_LOW          10.0f  // Gate drive rail min (V)
#define PROTECTION_HIGH         18.0f  // Gate drive rail max (V)

// Pulse parameters
#define PULSE_MIN_MS            0.0f   // Minimum pulse (0 = OFF for P2)
#define PULSE_MAX_MS            50.0f  // Maximum pulse duration
#define PULSE_STEP_MS           0.5f   // Step size (0.5ms resolution)
#define S_VALUE_MIN             0.3f   // AUTO mode: min contact delay (s)
#define S_VALUE_MAX             2.0f   // AUTO mode: max contact delay (s)
#define S_VALUE_STEP            0.1f   // AUTO mode: step size (s)

// Timing
#define DEBOUNCE_DELAY_MS       5
#define LOW_V_CONFIRM_MS        1500
#define PROTECTION_CONFIRM_MS   1500
#define CHARGER_SETTLE_US       500    // Settle time before/after pulse
```

#### 2. `welding.cpp` — Core Logic (Preserved 1:1)
- `generatePulse()` — P1/T/P2 dual-pulse with CHARGER_EN disconnect
- `checkProtectionVoltage()` — 13.5V rail monitoring
- `checkContactDetection()` — AUTO mode electrode sensing
- `checkLowVoltage()` — Supercap voltage protection
- All timing logic identical to original, only pin numbers change

#### 2b. `audio.cpp` — I2S Audio Feedback (NEW — replaces buzzer)
- Uses ESP32-S3 I2S peripheral → built-in amplifier → speaker on P6
- `playTone(freq, duration)` — generate sine wave at given frequency
- `playBeep()` — short confirmation beep (pleasant tone, not harsh buzzer)
- `playWeldReady()` — ascending 2-note chime when caps fully charged
- `playWeldFire()` — short sharp tone when pulse fires
- `playError()` — descending warning tone for protection alerts
- `playStartup()` — boot-up melody (3–4 notes)
- Sound can be muted via settings toggle

#### 3. `ui.cpp` — LVGL Dashboard (New)

**Main Dashboard Screen Layout (480×320):**
```
┌─────────────────────────────────────────────────────┐
│  ⚡ MyWeld ESP32              🔋 13.2V    AUTO/MAN  │  ← Status bar
├─────────────────────────────────────────────────────┤
│                                                     │
│   ┌─────────┐  ┌─────────┐  ┌─────────┐           │
│   │  P1     │  │   T     │  │  P2     │           │
│   │  15ms   │  │  10ms   │  │  20ms   │           │  ← Tap to edit
│   │  [−][+] │  │  [−][+] │  │  [−][+] │           │  ← Touch +/- buttons
│   └─────────┘  └─────────┘  └─────────┘           │
│                                                     │
│  S = 0.5s (AUTO only)  [−][+]                      │
│                                                     │
│  ┌────────────────────────────────────────────────┐ │
│  │  ████████████████████░░░░░░  4.8V / 5.4V  92% │ │  ← Color voltage bar
│  └────────────────────────────────────────────────┘ │
│                                                     │
│  ┌────────────────────────────────────────────────┐ │
│  │  Voltage History Graph (last 30 seconds)       │ │  ← Live voltage graph
│  └────────────────────────────────────────────────┘ │
│                                                     │
│  Welds: 42 (session) / 1,247 (total)    ⚙️ [MENU]  │  ← Weld counter + settings
│  Status: READY ● (charging animation)              │  ← Status indicator
└─────────────────────────────────────────────────────┘
```

**Settings Screen (slide-in or new screen):**
- Brightness slider (0–100%)
- Sound ON/OFF toggle
- AUTO/MAN mode switch
- S value adjustment (AUTO mode only, 0.3–2.0s)
- Preset profiles: Save/Load (up to 5 named presets)
- Weld counter reset
- About / version info

**Preset Profile Structure (NVS):**
```c
#define MAX_PRESETS 10

typedef struct {
    char name[20];      // e.g. "0.15mm Nickel"
    float p1;           // 0–50 ms (0.5ms steps)
    float t;            // 0–50 ms (0.5ms steps)
    float p2;           // 0–50 ms (0 = OFF, 0.5ms steps)
    float sValue;       // 0.3–2.0s (AUTO mode delay)
    bool autoMode;
} weld_preset_t;
```

**Factory Default Presets:**
| # | Name | P1 | T | P2 | Auto S | Notes |
|---|------|-----|-----|-----|--------|-------|
| 0 | 0.1mm Nickel | 3.0 | 5.0 | 5.0 | 0.5s | Thin battery tabs |
| 1 | 0.15mm Nickel | 5.0 | 8.0 | 8.0 | 0.5s | Standard tabs |
| 2 | 0.2mm Nickel | 8.0 | 10.0 | 12.0 | 0.5s | Thick tabs |
| 3 | 0.3mm Nickel | 12.0 | 12.0 | 18.0 | 0.6s | Heavy duty |
| 4 | 0.5mm Nickel | 18.0 | 15.0 | 25.0 | 0.8s | Busbar connections |
| 5 | Stainless Steel | 20.0 | 15.0 | 30.0 | 0.8s | Higher resistance |
| 6 | Aluminum | 25.0 | 12.0 | 35.0 | 0.8s | Oxide layer needs energy |
| 7 | Custom 1 | 10.0 | 10.0 | 10.0 | 0.5s | User-defined |
| 8 | Custom 2 | 10.0 | 10.0 | 10.0 | 0.5s | User-defined |
| 9 | Custom 3 | 10.0 | 10.0 | 10.0 | 0.5s | User-defined |

> ⚠️ Preset values are starting points — users MUST tune for their specific electrodes, cable resistance, and supercap charge level.

#### 4. `settings.c` — NVS Storage
- Uses ESP-IDF `nvs_flash` API (not Arduino Preferences)
- Stores: P1, T, P2, S, autoMode, brightness, sound, weld counters, presets, theme, BLE name
- Auto-save on parameter change (debounced 2s to prevent flash wear)
- Factory reset restores all default presets

---

## Tasks

### Phase A: Display + UI + Audio (testable with board only, no welding hardware)

- [x] **Task 1: PlatformIO project scaffolding** → ✅ Done: 15 files, platformio.ini, sdkconfig.defaults, partitions.csv.

- [x] **Task 2: Display init (LovyanGFX)** → ✅ Done: `display.cpp` with custom LGFX class (QSPI bus, backlight PWM, Touch I2C).

- [x] **Task 3: LVGL 9.x integration** → ✅ Done: `lv_conf.h`, PSRAM draw buffers, flush callback, touch input driver, mutex for thread safety.

- [x] **Task 4: I2S audio** → ✅ Done: `audio.c` with sine wave tone generator, I2S standard driver, command queue, all sound functions.

- [x] **Task 5: Main dashboard UI** → ✅ Done: `ui.c` with status bar, 3 parameter cards (P1/T/P2), S value card, voltage bar, chart (60 points), weld counter, status indicator, settings gear.

- [x] **Task 6: Settings screen + dark/light mode** → ✅ Done: Settings with preset dropdown, brightness slider, sound toggle, theme toggle, version info.

- [x] **Task 7: NVS settings persistence** → ✅ Done: `settings.c` with full NVS read/write, debounced auto-save, factory reset, preset save/load, weld counter persistence.

### Phase B: Welding Logic (requires hardware connected)

- [x] **Task 8: Core welding logic** → ✅ Done: `welding.c` with full state machine (9 states), dual-pulse firing via `ets_delay_us`, charger disconnect, protection monitoring, contact detection, ADC calibration.

- [ ] **Task 9: ADC calibration + test mode** → 🔧 ADC calibration infrastructure in place (`adc_cal_voltage/protection` factors in settings). Calibration wizard UI screen to be added during hardware testing.

- [x] **Task 10: Bluetooth serial** → ✅ Done: `ble_serial.c` with NimBLE GATT service, 3 characteristics (params R/W, status R/notify, commands W), text protocol.

### Phase C: Integration & Polish

- [ ] **Task 11: Full integration test** → ⏳ Requires hardware: flash firmware, connect to welding hardware, verify full cycle.

- [ ] **Task 12: Polish & optimization** → ⏳ Requires hardware testing: tune timings, optimize ADC, add energy calculator.

---

## Done When

- [ ] PlatformIO project compiles with ESP-IDF 5.x + LVGL 9.x + LovyanGFX
- [ ] ESP32-S3 boots and shows full dashboard UI on the 480×320 touchscreen
- [ ] Dark mode and Light mode themes switchable in settings
- [ ] P1/T/P2/S parameters adjustable via touch +/- buttons (0.5ms resolution)
- [ ] MAN mode fires weld pulse ONLY via physical START button (not touchscreen)
- [ ] AUTO mode detects electrode contact and fires after S delay
- [ ] Charger disconnects during weld pulse and reconnects after
- [ ] Live voltage graph and color-coded voltage bar update in real-time (0–5.7V)
- [ ] Weld counter tracks session and total welds
- [ ] 10 preset profiles (7 factory + 3 custom) can be saved/loaded
- [ ] All settings persist across power cycles (NVS)
- [ ] Protection systems work: low supercap voltage (4.0V warn, 3.0V block), gate drive rail out-of-range
- [ ] I2S speaker provides pleasant audio feedback (tones, melodies)
- [ ] BLE serial allows phone to read voltage, change parameters, view weld log
- [ ] ADC calibration wizard matches multimeter readings within ±0.1V

---

## Notes

### ⚠️ Critical Safety Considerations
1. **Physical START button ONLY for MAN mode** — No on-screen weld button. 2000A is not something to trigger accidentally via touch.
2. **CHARGER_EN (IO16) must default LOW on boot** — Charger enabled by default until code explicitly takes control.
3. **OUTPUT_PIN (IO46) must default LOW on boot** — `gpio_set_level(OUTPUT_PIN, 0);` in first lines of `app_main()`.
4. **Gate drive rail must be verified** before any weld is allowed — `checkProtectionVoltage()` runs continuously.
5. **BLE weld trigger requires safety confirmation** — phone command alone cannot fire weld without physical button held.

### 📚 Framework & Library Versions
- **Framework**: PlatformIO + ESP-IDF 5.x (not Arduino)
- **LovyanGFX**: Latest (v1.x) — added as ESP-IDF component
- **LVGL**: v9.x — latest API (`lv_display_*`, `lv_indev_*`)
- **NVS**: ESP-IDF `nvs_flash` API (not Arduino Preferences)
- **I2S**: ESP-IDF `driver/i2s_std.h` (new I2S driver, not legacy)
- **BLE**: ESP-IDF NimBLE stack
- **Board**: ESP32-S3 (N16R8) — PSRAM: OPI, Flash: QIO, Flash Size: 16MB

### 🔌 AXS15231B Display Specifics
- This display uses QSPI (not standard SPI) — needs correct LovyanGFX bus config
- LCD CS pin = **GPIO45** (verified), TE = **GPIO38**
- Touch controller is also AXS15231B (same chip handles display + touch)
- I2C address: 0x3B, Touch RST and INT are **NC** (not connected on this board)
- Some users report needing full-refresh mode with PSRAM buffer for best LVGL compatibility
- Hardware rotation may not be supported — use software rotation if needed

### 🔊 I2S Audio (replaces buzzer)
- Board has a **built-in I2S audio amplifier** (chip U4 on PCB)
- I2S pins: LRCLK=GPIO2, BCLK=GPIO42, DATA=GPIO41
- CTRL connected to VBAT — amp powers on when battery/power is connected
- Speaker connects to **P6** connector (-SP, -VAT)
- Use ESP32-S3 I2S peripheral to generate sine wave tones at various frequencies
- Sound library: use `driver/i2s.h` (ESP-IDF) or Arduino `I2S` library
- **Sound types:**
  - `playBeep()` — 880Hz, 50ms (parameter change confirmation)
  - `playWeldFire()` — 1200Hz, 30ms (pulse fired)
  - `playWeldReady()` — 660Hz→880Hz ascending (caps fully charged)
  - `playError()` — 440Hz→220Hz descending (protection alert)
  - `playStartup()` — C5→E5→G5 arpeggio (boot melody)
  - `playContactDetected()` — 1000Hz, 20ms (AUTO mode contact sense)

### 🔋 Power Input
- ESP32 powered via **P1 connector** (+5VIN pin) from the 3A buck module
- Do NOT use BAT/P5 for 13V — BAT is for 3.7V LiPo only
- USB-C used for programming only during development

### 📐 Voltage Divider Wiring
Original code used 5V Arduino ADC. ESP32-S3 is 3.3V. ALL analog inputs MUST use voltage dividers:
- **Supercap (0–5.7V)**: 10kΩ + 15kΩ divider → 0–2.28V at ADC
- **Protection rail (0–13.5V)**: 47kΩ + 15kΩ divider → 0–3.26V at ADC
- **Contact pin**: 10kΩ pull-up to 3.3V + 100kΩ isolation resistor

### � Supercap Bank Specifications
- **Configuration:** 2S2P (2 series × 2 parallel)
- **Individual cap:** 3.0V, 3000F
- **Bank voltage range:** 0–6.0V (2×3.0V)
- **Operating max:** 5.7V (derated for longevity)
- **Bank capacitance:** 3000F (series halves, parallel doubles → 3000F)
- **Energy at 5.7V:** E = ½CV² = ½ × 3000 × 5.7² = **48,735 Joules** ⚡
- **Estimated peak current:** ~2000A (depends on electrode resistance)
- **Charge time (10A charger):** Q = CV = 3000 × 5.7 = 17,100C → 17,100/10 ≈ **28.5 minutes** (0→5.7V)

### 🔄 NVS Storage Keys
| NVS Key | Type | Default | Notes |
|---------|------|---------|-------|
| `p1` | float | 5.0 | Pulse 1 duration (ms) |
| `t` | float | 8.0 | Pause duration (ms) |
| `p2` | float | 8.0 | Pulse 2 duration (ms, 0=OFF) |
| `autoMode` | bool | false | AUTO/MAN mode |
| `sValue` | float | 0.5 | AUTO mode contact delay (s) |
| `brightness` | uint8_t | 80 | Display brightness (0–100%) |
| `soundOn` | bool | true | Audio enable/disable |
| `theme` | uint8_t | 0 | 0=dark, 1=light |
| `sessionWelds` | uint32_t | 0 | Session weld counter |
| `totalWelds` | uint32_t | 0 | Lifetime weld counter |
| `activePreset` | uint8_t | 1 | Currently selected preset index |
| `preset_0`..`preset_9` | blob | (factory) | 10 preset profiles (weld_preset_t) |
| `adc_cal_v` | float | 1.0 | Voltage ADC calibration factor |
| `adc_cal_p` | float | 1.0 | Protection ADC calibration factor |
| `bleName` | string | "MyWeld" | BLE device name |


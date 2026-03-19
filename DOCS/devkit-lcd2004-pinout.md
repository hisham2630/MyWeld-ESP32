# ESP32-S3 DevKit + LCD 2004 — Complete Pin Map

> **Board:** Generic ESP32-S3 N16R8 DevKit-C (`BOARD_VARIANT = 2`)  
> **Display:** 20×4 I2C Character LCD (`DISPLAY_TYPE = 3`)  
> **Audio:** LEDC Buzzer (`AUDIO_TYPE = 2`) or I2S + MAX98357 (`AUDIO_TYPE = 1`)  
> **PlatformIO env:** `devkit_lcd2004`

---

## Master GPIO Table (All Used Pins)

| GPIO | Define             | Dir    | Category      | Function                                          | Wire To                              |
|------|--------------------|--------|---------------|---------------------------------------------------|--------------------------------------|
| 0    | —                  | Input  | System        | BOOT button (on-board)                            | On-board — do not connect            |
| 5    | `PIN_VOLTAGE`      | Input  | ADC (CH4)     | Supercap voltage sense                            | Voltage divider mid-point (33k+10k)  |
| 6    | `PIN_PROTECTION`   | Input  | ADC (CH5)     | Gate drive rail voltage sense                     | Voltage divider mid-point (100k+15k) |
| 7    | `PIN_CONTACT`      | Input  | ADC (CH6)     | Electrode contact detection                       | Contact divider mid-point (18k+4.7k) |
| 8    | `PIN_LCD_SDA`      | I/O    | I2C LCD       | I2C Data (SDA) for PCF8574 backpack               | LCD backpack SDA                     |
| 9    | `PIN_LCD_SCL`      | Output | I2C LCD       | I2C Clock (SCL) for PCF8574 backpack              | LCD backpack SCL                     |
| 9    | `PIN_SPARE_1`      | Input  | ADC (CH8)     | ⚠️ CONFLICT — reserved for temp sensor            | Not usable while LCD I2C is active   |
| 11   | `PIN_I2S_DOUT`     | Output | Audio (I2S)   | I2S Data Out *(only if AUDIO_TYPE=1)*             | MAX98357 DIN                         |
| 12   | `PIN_I2S_BCLK`     | Output | Audio (I2S)   | I2S Bit Clock *(only if AUDIO_TYPE=1)*            | MAX98357 BCLK                        |
| 13   | `PIN_I2S_LRCLK`    | Output | Audio (I2S)   | I2S Word Select *(only if AUDIO_TYPE=1)*          | MAX98357 LRC                         |
| 14   | `PIN_START`        | Input  | Welding       | Weld button — Manual trigger (active LOW, pull-up) | Momentary push button to GND        |
| 15   | `PIN_ENC_S1`       | Input  | Encoder       | Rotary encoder CLK (quadrature A)                 | Encoder CLK pin                      |
| 16   | `PIN_CHARGER_EN`   | Output | Welding       | Supercap charger KEY — HIGH=charge, LOW=off       | Charger module KEY/EN pin            |
| 17   | `PIN_ENC_S2`       | Input  | Encoder       | Rotary encoder DT (quadrature B)                  | Encoder DT pin                       |
| 18   | `PIN_ENC_KEY`      | Input  | Encoder       | Rotary encoder push button (active LOW)           | Encoder SW pin to GND                |
| 19   | —                  | I/O    | System (USB)  | USB JTAG D− (on-board USB-C)                      | On-board — do not connect            |
| 20   | —                  | I/O    | System (USB)  | USB JTAG D+ (on-board USB-C)                      | On-board — do not connect            |
| 26–37| —                  | —      | System        | ⛔ Internal Flash/PSRAM — NOT accessible          | —                                    |
| 46   | `PIN_OUTPUT`       | Output | Welding       | 🔥 MOSFET fire signal (gate driver input)         | Gate driver IN (opto-coupler/driver) |

> **GPIO46 Safety:** Strapping pin with internal pull-down. Boots LOW before firmware runs = MOSFETs stay OFF at power-on.

---

## Missing: Buzzer Pin (Not Yet Defined for DevKit)

When using `AUDIO_TYPE = AUDIO_BUZZER` (2), there is **no `PIN_BUZZER`
defined** in `board_config.h` for the DevKit variant. You must add one.

**Suggested addition** (choose any free GPIO):

| GPIO | Suggested Define    | Function             | Notes                |
|------|---------------------|----------------------|----------------------|
| 10   | `PIN_BUZZER`        | Passive buzzer (LEDC PWM) | Free, ADC1_CH9  |
| 3    | `PIN_BUZZER`        | Passive buzzer (LEDC PWM) | Free, ADC1_CH2  |

The GOOUUU CAM variant uses GPIO18 for its buzzer, but on DevKit+LCD that pin is taken by the encoder SW.

---

## Pin Status — Every ESP32-S3 GPIO at a Glance

| GPIO | Status       | Used By                          |
|------|--------------|----------------------------------|
| 0    | 🔒 System    | BOOT button (on-board)           |
| 1    | ✅ Free      | Available                        |
| 2    | ✅ Free      | Available                        |
| 3    | ✅ Free      | Available (good for buzzer)      |
| 4    | ✅ Free      | Available                        |
| 5    | 🟢 Used      | `PIN_VOLTAGE` — ADC supercap     |
| 6    | 🟢 Used      | `PIN_PROTECTION` — ADC gate rail |
| 7    | 🟢 Used      | `PIN_CONTACT` — ADC electrode    |
| 8    | 🟢 Used      | `PIN_LCD_SDA` — I2C LCD data     |
| 9    | 🟢 Used      | `PIN_LCD_SCL` — I2C LCD clock    |
| 10   | ✅ Free      | Available (good for buzzer)      |
| 11   | 🟡 Optional  | `PIN_I2S_DOUT` (I2S audio only)  |
| 12   | 🟡 Optional  | `PIN_I2S_BCLK` (I2S audio only)  |
| 13   | 🟡 Optional  | `PIN_I2S_LRCLK` (I2S audio only) |
| 14   | 🟢 Used      | `PIN_START` — Weld button        |
| 15   | 🟢 Used      | `PIN_ENC_S1` — Encoder CLK       |
| 16   | 🟢 Used      | `PIN_CHARGER_EN` — Charger KEY   |
| 17   | 🟢 Used      | `PIN_ENC_S2` — Encoder DT        |
| 18   | 🟢 Used      | `PIN_ENC_KEY` — Encoder SW       |
| 19   | 🔒 System    | USB JTAG D−                      |
| 20   | 🔒 System    | USB JTAG D+                      |
| 21   | ✅ Free      | Available                        |
| 22–25| ✅ Free      | Available (check your DevKit)    |
| 26–37| ⛔ Reserved  | Flash / PSRAM (not accessible)   |
| 38   | ✅ Free      | Available                        |
| 39–45| ✅ Free      | Available (check your DevKit)    |
| 46   | 🟢 Used      | `PIN_OUTPUT` — MOSFET fire       |
| 47   | ✅ Free      | Available                        |
| 48   | ✅ Free      | Available                        |

---

## Wiring Diagram

```
ESP32-S3 DevKit-C (N16R8)
┌──────────────────────────────────────────────────────┐
│                                                      │
│  ── WELDING I/O ──                                   │
│  GPIO46 ──────►  Gate Driver IN  (MOSFET fire)       │
│  GPIO16 ──────►  Charger KEY/EN  (charge on/off)     │
│  GPIO14 ◄──────  Weld Button     (to GND, pull-up)   │
│                                                      │
│  ── ADC SENSING ──                                   │
│  GPIO5  ◄──────  Supercap divider (33kΩ + 10kΩ)      │
│  GPIO6  ◄──────  Gate rail divider (100kΩ + 15kΩ)    │
│  GPIO7  ◄──────  Contact divider  (18kΩ + 4.7kΩ)     │
│                                                      │
│  ── I2C LCD (20×4) ──                                │
│  GPIO8  ◄─────►  PCF8574 SDA                         │
│  GPIO9  ──────►  PCF8574 SCL                         │
│                  (I2C addr: 0x27, bus: I2C_NUM_0)    │
│                                                      │
│  ── ROTARY ENCODER ──                                │
│  GPIO15 ◄──────  Encoder CLK                         │
│  GPIO17 ◄──────  Encoder DT                          │
│  GPIO18 ◄──────  Encoder SW  (to GND, pull-up)       │
│                                                      │
│  ── AUDIO (pick one) ──                              │
│  GPIO11 ──────►  MAX98357 DIN   (I2S mode only)      │
│  GPIO12 ──────►  MAX98357 BCLK  (I2S mode only)      │
│  GPIO13 ──────►  MAX98357 LRC   (I2S mode only)      │
│  GPIO??  ─────►  Buzzer         (LEDC mode — TBD)    │
│                                                      │
│  ── SYSTEM (on-board, don't wire) ──                 │
│  GPIO0  ◄──────  BOOT button                         │
│  GPIO19 ◄─────►  USB D−                              │
│  GPIO20 ◄─────►  USB D+                              │
│  GPIO26–37       Flash/PSRAM (inaccessible)          │
│                                                      │
│  ── POWER ──                                         │
│  3V3    ──────►  Encoder VCC, LCD VCC (logic)        │
│  5V     ──────►  LCD backpack VCC (LED backlight)    │
│  GND    ──────►  Common ground (all peripherals)     │
│                                                      │
└──────────────────────────────────────────────────────┘
```

---

## Voltage Divider Reference

| Sensor       | R_high  | R_low  | Ratio (Vout/Vin) | Multiplier | Max Input |
|--------------|---------|--------|-------------------|------------|-----------|
| Supercap     | 33 kΩ   | 10 kΩ  | 0.233             | ×4.3       | ~14.1V    |
| Gate Rail    | 100 kΩ  | 15 kΩ  | 0.130             | ×7.67      | ~25.3V    |
| Contact      | 18 kΩ   | 4.7 kΩ | 0.207             | ×4.83      | ~15.9V    |

> All ADC inputs must stay below **3.3V** at the GPIO pin.

---

## Known Conflicts

| GPIO | Issue                                        | Status                               |
|------|----------------------------------------------|--------------------------------------|
| 9    | `PIN_LCD_SCL` vs `PIN_SPARE_1` (temp sensor) | ⚠️ Open — remap if temp sensor needed |
| —    | No `PIN_BUZZER` defined for DevKit variant   | ⚠️ Open — add GPIO10 or GPIO3        |

---

> **Source files:**  
> [`board_config.h`](../src/board_config.h) — Variant-specific pin overrides  
> [`config.h`](../src/config.h) — Default pin definitions (JC3248W535 baseline)

# MyWeld — Future Improvements & Feature Ideas

> **Created:** 2026-02-26
> **Status:** Brainstorm — none of these are committed yet.
> **Context:** The project already has a complete ESP32 firmware (LVGL dashboard, I2S audio, NVS presets, BLE binary protocol, OTA updates) and a full Android companion app (Jetpack Compose, Nordic BLE, binary protocol, OTA from GitHub).

---

## 🔥 Tier 1: High-Impact Features

### 1. 📊 Weld Quality Scoring & History

**Problem:** Currently we track weld count but not weld quality.

**Idea:**
- Measure **voltage drop during pulse** (ΔV = V_before − V_after) — this correlates with energy delivered
- Calculate **energy per weld**: `E = ½C(V₁² − V₂²)` (from supercap voltage delta, C ≈ 3000F)
- Score each weld (Green / Yellow / Red) based on expected ΔV for the selected material
- **Log timestamped history to SD card** (GPIO 10-13 already wired to TF slot)
- Show weld history on the Android app as a scrollable list with charts
- Export as CSV

**Why it matters:** Transforms the welder from "I think that was a good weld" to "I **know** that was 4.2J at 0.15mm nickel = ✅ green"

**Effort:** Medium — needs ADC reads before/after pulse, SD card driver, new BLE message type for log sync, app UI

**Hardware needed:** None (SD card slot already on board)

---

### 2. 🔐 PIN/Password Lock for BLE

**Problem:** Anyone with a BLE scanner can connect and change parameters.

**Idea:**
- Extend existing OTA PIN auth to cover all BLE parameter writes
- Lock/unlock toggle configurable from the touchscreen
- PIN stored in NVS, changeable from settings
- Failed attempts → lockout timer

**Why it matters:** Essential in shared workshop environments

**Effort:** Low — auth flag already exists for OTA, just extend to params/commands

**Hardware needed:** None
[x] done
---

### 3. 📈 Temperature Monitoring (MOSFET / Supercap)

**Problem:** 2000A through 16 MOSFETs generates significant heat. No thermal protection.

**Idea:**
- Add **DS18B20** (1-Wire) or **NTC thermistor** on MOSFET heatsink → ADC on spare GPIO
- Optionally monitor supercap temperature (critical for lifetime)
- Show temperature on ESP32 dashboard + Android app
- **Thermal throttle:** refuse welding if MOSFET temp > 80°C
- Add thermal trend chart in app

**Available spare GPIO:** IO9 (ADC1_CH8) — IO15/IO17/IO18 are now used by the rotary encoder

**Why it matters:** Serious safety feature — prevents MOSFET thermal failure during rapid successive welds

**Effort:** Low — 1 sensor + 1 ADC read + UI update + new field in BLE status packet

**Hardware needed:** DS18B20 or NTC thermistor + pull-up resistor

---

### 4. 🌐 WiFi Mode for Advanced Features

**Problem:** BLE is limited to ~30KB/s and phone-only access.

**Idea:**
- **Web dashboard** — access from any device (tablet, laptop, PC) on the same network
- **WiFi OTA** — much faster firmware updates (500KB/s+ vs 30KB/s BLE)
- **Cloud logging** (optional) — upload weld history to a cloud service
- **AP mode** for initial setup (like IoT devices — connect to "MyWeld-XXXX" WiFi)
- WiFi and BLE can coexist on ESP32-S3, but BLE should remain the primary interface

**Why it matters:** Opens up advanced use cases without replacing the working BLE system

**Effort:** High — web server, mDNS, WiFi provisioning, dual-mode networking

**Hardware needed:** None (ESP32-S3 has WiFi built-in)

---

## ⚡ Tier 2: Quality-of-Life Improvements

### 5. 🔊 Custom Sound Profiles

**Problem:** Single set of beep/tone sounds for all users.

**Idea:**
- 3-4 built-in sound profiles: **Industrial** (sharp/metallic), **Arcade** (fun/8-bit), **Minimal** (subtle), **Silent**
- Different tones per event (beep, weld fire, error, ready, startup)
- Possibly load custom WAV/PCM files from SD card
- Profile selection in settings (ESP32 + Android)

**Effort:** Low-Medium — sound generation code already exists, just need alternate tone tables

**Hardware needed:** None

---

### 6. 📱 iOS Companion App (or Cross-Platform)

**Problem:** Currently Android-only. Many users have iPhones.

**Options:**
| Option | Pros | Cons |
|--------|------|------|
| **Flutter** (Dart) | Single codebase, native BLE plugins | Rewrite all UI |
| **Kotlin Multiplatform** | Share business logic with existing Kotlin | Still need SwiftUI for iOS UI |
| **Web Bluetooth** (PWA) | Zero install, works on Chrome desktop too | Chrome-only, no iOS Safari support |
| **Native Swift** | Best iOS experience | Separate codebase to maintain |

**Recommendation:** Flutter or KMP for shared protocol/BLE layer, platform-specific UI

**Effort:** High — essentially building a second app (or rewriting in cross-platform)

---

### 7. ✅ Auto-Calibration Wizard (IMPLEMENTED)

**Problem:** Task 9 (ADC calibration wizard) is still pending. Manual calibration is tedious.

**Idea:**
- **Guided wizard on ESP32 touchscreen:**
  1. "Connect multimeter to supercap terminals"
  2. "Enter measured voltage: [___] V"
  3. Firmware calculates and stores correction factor
- **One-button recalibration** via Android app
- Store calibration constants + calibration date in NVS
- Show "⚠️ Calibration expired" if last cal was > 30 days ago
- Compare ADC reading vs reference → auto-compute multiplier

**Effort:** Medium — needs new UI screen, guided flow, NVS storage for cal date

**Hardware needed:** None (multimeter required by user for reference)

---

### 8. 📍 Electrode Wear Detection

**Problem:** Electrodes degrade over time, weld quality silently drops.

**Idea:**
- Track welding resistance trends over time (inferred from voltage drop patterns at similar charge levels)
- When weld energy delivery decreases for the same settings → alert
- "⚠️ Electrode resistance 40% higher than baseline — consider cleaning or replacing tips"
- Baseline established during first few welds with fresh electrodes

**Effort:** Medium — statistical analysis of weld history data, requires Tier 1.1 (weld quality scoring) first

**Hardware needed:** None (calculated from existing ADC data)

---

### 9. 🎯 Multi-Pulse Profiles (Advanced Pulse Sequences)

**Problem:** Current P1/T/P2 is a fixed dual-pulse. Some materials need more.

**Idea:**
- **3-pulse sequences:** P1/T1/P2/T2/P3 for thick materials or stainless steel
- **Ramp-up pulses:** Gradually increasing pulse width (P1 < P2 < P3) for sensitive materials
- **Electrode conditioning pulse:** Very short, low-energy pre-pulse to burn through oxide layers (aluminum)
- **Custom sequence builder** on Android app (drag-and-drop pulse blocks)

**Impact on firmware:**
- `generatePulse()` needs to be generalized to N pulses
- Preset structure grows (need array of pulse durations)
- BLE protocol needs updated PARAMS_WRITE for variable-length pulse data

**Effort:** Medium-High — significant firmware refactor, protocol extension, app UI

**Hardware needed:** None

---

## 🎨 Tier 3: Polish & Pro Features

### 10. 📊 Battery/Supercap Health Analytics

**Idea:**
- Track charge/discharge cycles over lifetime (NVS counter)
- **ESR estimation:** Measure voltage sag under load → estimate internal resistance increase
- Predict remaining supercap lifetime based on cycle count and ESR trend
- "Supercaps at ~80% health — consider replacement in ~6 months"
- Show health dashboard on Android app

**Effort:** Medium — needs consistent data collection over time

---

### 11. 🏷️ Material Database

**Idea:**
- Built-in searchable database of materials with recommended settings:
  - **Nickel strip:** 0.1mm, 0.15mm, 0.2mm, 0.3mm
  - **Stainless steel:** 0.1mm, 0.2mm
  - **Copper foil:** various thicknesses
  - **Aluminum:** with oxide-breaking recommendations
- Each entry includes: recommended P1/T/P2, electrode tip size, clamping force
- User can select material → auto-loads optimal settings
- Community contributions via GitHub JSON file

**Effort:** Low — mostly data entry + a new UI screen

---

### 12. 📲 Android Widget / Quick Settings Tile

**Idea:**
- **Home screen widget:** Connection status, supercap voltage %, last weld score
- **Quick Settings tile:** Tap to connect/disconnect BLE
- **Notification:** Persistent notification while connected showing voltage level

**Effort:** Medium — Android widget APIs, background BLE scanning

---

### 13. 🌙 Screensaver / Display Power Management

**Idea:**
- Dim display after X seconds of inactivity (configurable: 15s / 30s / 60s / never)
- Turn off display completely after X minutes (save power + extend display life)
- Wake on: touch, weld button press, BLE command
- Don't dim during active charging (show charging animation instead)
- Currently backlight is on GPIO1 with PWM — dimming is trivial

**Effort:** Low — timer + PWM fade, already have backlight control

---

### 14. 🧪 Test / Dry-Run Mode

**Idea:**
- Fire a "virtual weld" with OUTPUT_PIN disconnected (software-only timing)
- Show oscilloscope-style timing diagram on the display:
  ```
  P1 ██████░░░░░░░░████████████ P2
          ↑ T (pause) ↑
  ```
- Verify pulse timing without any welding hardware connected
- Useful for: setting up presets, teaching, debugging timing

**Effort:** Low — conditionally skip GPIO output, draw timing chart on LVGL

---

### 15. 🔗 Export & Sharing

**Idea:**
- **Export weld log** as CSV from SD card (via USB or BLE file transfer)
- **Share presets** between users:
  - QR code on ESP32 display (encode preset as JSON)
  - Android app scan QR → import preset
  - Or share via BLE between two MyWeld devices
- **Backup/restore** all settings + presets + calibration to file

**Effort:** Medium — QR code generation (LVGL has QR widget), file export

---

## 🛡️ Tier 4: Safety & Reliability

### 16. ⚠️ Hardware Watchdog Timer

**Idea:**
- Ensure ESP-IDF hardware watchdog (`esp_task_wdt`) is configured
- If main loop hangs → OUTPUT_PIN goes LOW (hardware guarantee)
- Independent watchdog on welding task — if firing timer exceeds MAX_PULSE + margin → force stop
- Log watchdog reset events to NVS for debugging

**Effort:** Low — ESP-IDF WDT API, a few lines of config

---

### 17. 🔋 Charging Analytics & Time Estimation

**Idea:**
- Detect **CC phase** (constant current — linear voltage rise) vs **CV phase** (constant voltage — current taper)
- Estimate time to full: "Charging... ~5 min remaining" (based on dV/dt slope)
- Detect **charger fault:** no voltage rise after X minutes → alert
- Show charge rate: "Charging at ~8A" (estimated from dV/dt and known capacitance)

**Effort:** Low-Medium — math on existing ADC voltage samples over time

---

### 18. 📶 BLE Signal Strength Indicator

**Idea:**
- Show RSSI value on Android app (already available from Nordic BLE lib)
- Show connection quality indicator on ESP32 display (bars icon)
- Warn when signal is weak: "⚠️ Weak BLE connection — move closer"
- Log disconnection events with RSSI at time of disconnect

**Effort:** Low — RSSI is already available, just needs UI
[x] done
---

## ⭐ Top 3 Recommended Next Steps

| Priority | Feature | Impact | Effort | Rationale |
|----------|---------|--------|--------|-----------|
| **#1** | Weld Quality Scoring + SD Logging | 🔥🔥🔥 | Medium | Data-driven welding — biggest user value add |
| **#2** | Temperature Monitoring | 🔥🔥🔥 | Low | Safety-critical, spare GPIOs available, minimal hardware |
| **#3** | Display Power Management | 🔥🔥 | Low | Quality of life, extends display life, trivial to implement |

---

## Dependencies Between Features

```
Weld Quality Scoring (#1)
  └── Electrode Wear Detection (#8) (needs weld history data)
  └── Supercap Health Analytics (#10) (needs charge cycle tracking)
  └── Export & Sharing (#15) (needs data to export)

Temperature Monitoring (#3)
  └── Thermal throttle (safety interlock in welding.c)

WiFi Mode (#4)
  └── Web Dashboard (separate web UI)
  └── WiFi OTA (faster firmware updates)
  └── Cloud Logging (upload weld history)

Multi-Pulse Profiles (#9)
  └── Material Database (#11) (can store per-material pulse sequences)
```

---

> **Note:** All features should be implemented incrementally. Each should work independently and not break existing functionality. BLE protocol extensions should use the existing version/type system for backward compatibility.

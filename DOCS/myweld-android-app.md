# MyWeld Android Companion App — Project Plan

## Goal

Build a **native Kotlin Android companion app** for the MyWeld ESP32 spot welder that communicates via **BLE (Bluetooth Low Energy)** using a **structured binary protocol**. The app provides full control and monitoring: live voltage, parameter adjustment, preset management, weld counter, and multi-device support — all wrapped in a **premium dark/light theme** with metallic accents and glowing visual elements.

---

## Project Type

**MOBILE** — Native Kotlin + Jetpack Compose (Android only)

---

## Decisions (From Brainstorm)

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Feature Scope** | Full Control | Mirror all touchscreen features remotely |
| **BLE Protocol** | Structured binary (Option C) | Robust, extensible, proper error handling — replaces fragile text parsing |
| **Min API** | 26 (Android 8.0+) | ~95% device coverage |
| **UI Style** | Custom Dark Premium + Light mode | Metallic accents, glowing voltage bars, spark animations |
| **Background** | Foreground only | No background service needed for now |
| **Architecture** | Jetpack Compose + MVVM + Koin + Nordic BLE Library | Modern, maintainable, proven BLE handling |
| **Multi-device** | Yes | Scan, connect, save multiple MyWeld devices |

---

## Success Criteria

- [ ] App discovers and connects to MyWeld ESP32 devices via BLE
- [ ] Live supercap voltage, protection voltage, weld state shown in real-time (≤500ms latency)
- [ ] P1/T/P2/S values adjustable from the app with instant feedback
- [ ] AUTO/MAN mode switchable
- [ ] 10 presets loadable and savable
- [ ] Weld counter visible (session + total)
- [ ] Live voltage chart (at least 30 data points rolling)
- [ ] Multi-device: scan, select, remember paired devices
- [ ] Dark mode (default) + Light mode toggle
- [ ] Professional UI with safe area respect (status bar, navigation bar)
- [ ] Works on Android 8.0+ (API 26+)
- [ ] BLE protocol uses structured binary format (both firmware + app updated)

---

## Tech Stack

| Layer | Technology | Rationale |
|-------|-----------|-----------|
| **Language** | Kotlin 2.0+ | Native Android, modern syntax |
| **UI** | Jetpack Compose + Material 3 | Declarative, custom theming, no XML |
| **Architecture** | MVVM + Repository pattern | Clean separation, testable |
| **DI** | Koin 4.x | Lightweight, no annotation processing |
| **BLE** | Nordic Android BLE Library 2.x | Handles Android BLE quirks (bonding, reconnect, MTU negotiation) |
| **Navigation** | Compose Navigation | Single activity, type-safe routes |
| **State** | StateFlow + ViewModel | Lifecycle-aware, simple |
| **Local Storage** | DataStore (Preferences) | Save paired devices, app preferences |
| **Charts** | Vico (Compose-native charting) | Smooth, animated voltage graph |
| **Build** | Gradle Kotlin DSL + Version Catalogs | Modern, maintainable |
| **Min SDK** | 26 (Android 8.0) | Wide coverage, modern BLE APIs |
| **Target SDK** | 35 (Android 15) | Edge-to-edge, latest APIs |

---

## BLE Protocol V2 — Structured Binary Design

### Why Binary Over Text?

| Aspect | Old (Text) | New (Binary) |
|--------|-----------|--------------|
| **Parsing** | `sscanf("P1=%f")` — fragile | Fixed offsets — robust |
| **Size** | ~80 bytes per status | ~32 bytes — faster BLE |
| **Error handling** | None (fire-and-forget) | ACK/NAK with error codes |
| **Extensibility** | Add new field = break parser | Version + type field = safe extension |
| **Endianness** | N/A | Little-endian (ESP32 native) |

### Protocol Packet Structure

```
┌──────┬──────┬──────┬──────────────────────────┬──────┐
│ SYNC │ TYPE │  LEN │        PAYLOAD           │  CRC │
│ 0xAA │ 1B   │ 1B   │   0–240 bytes            │  1B  │
└──────┴──────┴──────┴──────────────────────────┴──────┘
```

| Field | Size | Description |
|-------|------|-------------|
| SYNC | 1 byte | Always `0xAA` — packet start marker |
| TYPE | 1 byte | Message type (see table below) |
| LEN | 1 byte | Payload length (0–240) |
| PAYLOAD | 0–240 bytes | Type-specific data |
| CRC | 1 byte | XOR checksum of TYPE + LEN + PAYLOAD |

### Message Types

| Type | Hex | Direction | Payload | Description |
|------|-----|-----------|---------|-------------|
| STATUS | `0x01` | ESP→App (notify) | 32 bytes | Periodic status update |
| PARAMS_READ | `0x02` | App→ESP (read) | 0 | Request current parameters |
| PARAMS_RESPONSE | `0x03` | ESP→App | 28 bytes | Current parameter values |
| PARAMS_WRITE | `0x04` | App→ESP (write) | 28 bytes | Set new parameters |
| CMD | `0x05` | App→ESP (write) | varies | Commands (load preset, reset, etc.) |
| ACK | `0x06` | ESP→App (notify) | 2 bytes | Success: [original_type, 0x00] |
| NAK | `0x07` | ESP→App (notify) | 3 bytes | Error: [original_type, error_code, 0x00] |
| VERSION | `0x08` | App→ESP (read) | 0 | Request firmware version |
| VERSION_RESPONSE | `0x09` | ESP→App | 12 bytes | Version + build date |
| PRESET_LIST | `0x0A` | App→ESP (read) | 0 | Request all preset names |
| PRESET_LIST_RESPONSE | `0x0B` | ESP→App (notify) | varies | Preset names (chunked if needed) |
| WELD_LOG_ENTRY | `0x0C` | ESP→App (notify) | 12 bytes | Weld log event (timestamp, energy, voltage) |

### STATUS Payload (0x01) — 32 bytes, sent every ~500ms

```c
typedef struct __attribute__((packed)) {
    uint16_t supercap_mv;       // Supercap voltage in millivolts (0–5700)
    uint16_t protection_mv;     // Protection rail in millivolts (0–18000)
    uint8_t  state;             // Weld state enum (IDLE=0, READY=1, CHARGING=2, FIRING=3, ERROR=4, ...)
    uint8_t  charge_percent;    // 0–100%
    uint8_t  auto_mode;         // 0=MAN, 1=AUTO
    uint8_t  active_preset;     // 0–9
    uint32_t session_welds;     // Session weld counter
    uint32_t total_welds;       // Lifetime weld counter
    uint8_t  ble_connected;     // 1 if BLE connected (redundant but useful)
    uint8_t  sound_on;          // 0=mute, 1=on
    uint8_t  theme;             // 0=dark, 1=light (ESP32 display theme)
    uint8_t  error_code;        // 0=none, see error enum
    uint16_t p1_x10;            // P1 in 0.1ms units (e.g., 50 = 5.0ms)
    uint16_t t_x10;             // T in 0.1ms units
    uint16_t p2_x10;            // P2 in 0.1ms units
    uint16_t s_x10;             // S in 0.1s units (e.g., 5 = 0.5s)
    uint8_t  fw_major;          // Firmware major version
    uint8_t  fw_minor;          // Firmware minor version
    uint8_t  reserved[2];       // Future use, padding to 32 bytes
} ble_status_packet_t;          // Total: 32 bytes
```

### PARAMS_WRITE Payload (0x04) — 28 bytes

```c
typedef struct __attribute__((packed)) {
    uint16_t p1_x10;            // P1 in 0.1ms units
    uint16_t t_x10;             // T in 0.1ms units
    uint16_t p2_x10;            // P2 in 0.1ms units
    uint16_t s_x10;             // S in 0.1s units
    uint8_t  auto_mode;         // 0=MAN, 1=AUTO
    uint8_t  sound_on;          // 0=mute, 1=on
    uint8_t  brightness;        // 0–100%
    uint8_t  theme;             // 0=dark, 1=light
    char     ble_name[16];      // Device BLE name (null-terminated)
} ble_params_packet_t;          // Total: 28 bytes
```

### CMD Payload (0x05)

| Sub-command | Byte 0 | Extra | Description |
|-------------|--------|-------|-------------|
| LOAD_PRESET | `0x01` | `[preset_index]` | Load preset 0–9 |
| SAVE_PRESET | `0x02` | `[index, name[20], p1, t, p2, s, mode]` | Save current params as preset |
| FACTORY_RESET | `0x03` | none | Reset all settings to factory |
| RESET_WELD_COUNTER | `0x04` | `[0=session, 1=total, 2=both]` | Reset weld counters |
| CALIBRATE_ADC | `0x05` | `[channel, reference_mv_u16]` | ADC calibration |

### Error Codes (NAK payload byte 1)

| Code | Meaning |
|------|---------|
| `0x01` | Invalid parameter range |
| `0x02` | Invalid preset index |
| `0x03` | NVS write failure |
| `0x04` | Busy (welding in progress) |
| `0x05` | Unknown command |

### BLE GATT Service (Updated UUIDs)

| Characteristic | UUID | Properties | Description |
|---------------|------|-----------|-------------|
| **TX (Status)** | `12360000-...` | READ, NOTIFY | ESP32→App: status packets, ACK/NAK |
| **RX (Command)** | `12370000-...` | WRITE, WRITE_NO_RESPONSE | App→ESP32: params write, commands |
| **Params** | `12350000-...` | READ | One-shot read of current params |

---

## App File Structure

```
MyWeld-Android/
├── app/
│   ├── build.gradle.kts
│   └── src/main/
│       ├── AndroidManifest.xml
│       ├── java/com/myweld/app/
│       │   ├── MyWeldApp.kt                    # Application class + Koin init
│       │   ├── MainActivity.kt                 # Single activity, edge-to-edge
│       │   │
│       │   ├── navigation/
│       │   │   └── NavGraph.kt                 # Compose Navigation routes
│       │   │
│       │   ├── ui/
│       │   │   ├── theme/
│       │   │   │   ├── Theme.kt                # Dark/Light premium theme
│       │   │   │   ├── Color.kt                # Color palette (metallic, glow)
│       │   │   │   ├── Type.kt                 # Typography (Roboto/custom)
│       │   │   │   └── Shape.kt                # Shapes, corner radii
│       │   │   │
│       │   │   ├── screens/
│       │   │   │   ├── ScanScreen.kt           # BLE device scanner + picker
│       │   │   │   ├── DashboardScreen.kt      # Main control dashboard
│       │   │   │   ├── PresetsScreen.kt        # Preset manager (10 slots)
│       │   │   │   ├── SettingsScreen.kt       # App + device settings
│       │   │   │   └── AboutScreen.kt          # Version info, credits
│       │   │   │
│       │   │   └── components/
│       │   │       ├── VoltageBar.kt           # Glowing voltage bar with gradient
│       │   │       ├── VoltageChart.kt         # Live rolling voltage graph
│       │   │       ├── ParamCard.kt            # P1/T/P2/S parameter card with +/-
│       │   │       ├── StatusIndicator.kt      # IDLE/READY/CHARGING/FIRING/ERROR
│       │   │       ├── WeldCounter.kt          # Session + total counter display
│       │   │       ├── ConnectionBadge.kt      # BLE connection status badge
│       │   │       ├── PresetCard.kt           # Individual preset card
│       │   │       ├── DeviceCard.kt           # BLE device in scan list
│       │   │       └── GlowButton.kt           # Custom glowing button component
│       │   │
│       │   ├── viewmodel/
│       │   │   ├── ScanViewModel.kt            # BLE scan state
│       │   │   ├── DashboardViewModel.kt       # Dashboard state + BLE data
│       │   │   ├── PresetsViewModel.kt         # Preset management
│       │   │   └── SettingsViewModel.kt        # Settings state
│       │   │
│       │   ├── data/
│       │   │   ├── model/
│       │   │   │   ├── WelderStatus.kt         # Domain model for status
│       │   │   │   ├── WeldParams.kt           # Domain model for parameters
│       │   │   │   ├── WeldPreset.kt           # Domain model for presets
│       │   │   │   ├── WeldState.kt            # Enum: IDLE, READY, etc.
│       │   │   │   └── SavedDevice.kt          # Persisted BLE device info
│       │   │   │
│       │   │   ├── ble/
│       │   │   │   ├── MyWeldBleManager.kt     # Nordic BLE Library manager
│       │   │   │   ├── BleProtocol.kt          # Binary protocol encode/decode
│       │   │   │   ├── BlePacket.kt            # Packet structure definitions
│       │   │   │   ├── BleUuids.kt             # Service/characteristic UUIDs
│       │   │   │   └── BleConnectionState.kt   # Connection state sealed class
│       │   │   │
│       │   │   └── repository/
│       │   │       ├── WelderRepository.kt     # Interface: welder data access
│       │   │       ├── WelderRepositoryImpl.kt # Implementation via BLE
│       │   │       └── DeviceRepository.kt     # Saved devices (DataStore)
│       │   │
│       │   └── di/
│       │       └── AppModule.kt                # Koin dependency definitions
│       │
│       └── res/
│           ├── values/
│           │   ├── strings.xml
│           │   └── themes.xml                  # Edge-to-edge setup
│           ├── drawable/                        # App icon, vectors
│           └── mipmap/                          # Launcher icons
│
├── build.gradle.kts                            # Root build config
├── settings.gradle.kts                         # Module settings
└── gradle/
    └── libs.versions.toml                      # Version catalog
```

---

## Task Breakdown

### Phase A: Project Foundation (No BLE yet)

#### Task A1: Create Android Project
- **Agent:** `mobile-developer`
- **Skills:** `clean-code`, `mobile-design`
- **Priority:** P0 (blocker for everything)
- **Dependencies:** None
- **INPUT:** Empty directory
- **OUTPUT:** Compilable Kotlin + Compose project with:
  - `build.gradle.kts` with all dependencies (Compose, Koin, Nordic BLE, Vico, DataStore)
  - `libs.versions.toml` version catalog
  - `AndroidManifest.xml` with BLE permissions (`BLUETOOTH_CONNECT`, `BLUETOOTH_SCAN`, `ACCESS_FINE_LOCATION` for API <31)
  - `MainActivity.kt` with edge-to-edge setup (`WindowCompat.setDecorFitsSystemWindows(window, false)`)
  - `MyWeldApp.kt` application class with Koin initialization
- **VERIFY:** `./gradlew assembleDebug` succeeds

#### Task A2: Premium Theme System
- **Agent:** `mobile-developer`
- **Skills:** `mobile-design`, `frontend-design`
- **Priority:** P0 (affects all UI)
- **Dependencies:** A1
- **INPUT:** Theme design requirements (dark premium + light mode)
- **OUTPUT:**
  - `Color.kt` — Dual palette: dark (charcoal #1A1A2E, deep navy #16213E, metallic silver #C0C0C0, neon cyan #00D4FF, ember orange #FF6B35, voltage green #39FF14) and light mode equivalents
  - `Theme.kt` — Custom M3 `MaterialTheme` with custom `ColorScheme` for both modes
  - `Type.kt` — Typography scale using Roboto + monospace for values
  - `Shape.kt` — Rounded corners with subtle metallic border effects
- **VERIFY:** Toggle dark/light, all colors are correct and readable, no purple/violet

#### Task A3: Navigation Structure
- **Agent:** `mobile-developer`
- **Skills:** `clean-code`
- **Priority:** P0
- **Dependencies:** A1, A2
- **INPUT:** Screen list: Scan → Dashboard → Presets, Settings, About
- **OUTPUT:**
  - `NavGraph.kt` with 5 routes
  - Bottom navigation bar for Dashboard/Presets/Settings (only visible after connection)
  - Scan screen as start destination
- **VERIFY:** Can navigate between all screens, back navigation works, safe areas respected

---

### Phase B: BLE Communication Layer

#### Task B1: BLE Protocol Module (App Side)
- **Agent:** `mobile-developer`
- **Skills:** `clean-code`
- **Priority:** P0 (blocker for all BLE features)
- **Dependencies:** A1
- **INPUT:** Protocol spec from this plan (binary format)
- **OUTPUT:**
  - `BlePacket.kt` — Data classes for all packet types
  - `BleProtocol.kt` — Encoder/decoder: `ByteArray ↔ Kotlin data classes`, CRC calculation
  - `BleUuids.kt` — Service and characteristic UUIDs (matching firmware)
  - `BleConnectionState.kt` — Sealed class: `Disconnected`, `Scanning`, `Connecting`, `Connected`, `Error`
- **VERIFY:** Unit tests: encode STATUS packet → decode → original values match, CRC validates

#### Task B2: BLE Manager (Nordic Library Integration)
- **Agent:** `mobile-developer`
- **Skills:** `clean-code`
- **Priority:** P0
- **Dependencies:** B1
- **INPUT:** Nordic BLE Library API, UUID config
- **OUTPUT:**
  - `MyWeldBleManager.kt` extending `BleManager`:
    - `scan()` — discover MyWeld devices (filter by service UUID)
    - `connect(device)` — connect with MTU negotiation (request 247)
    - `disconnect()`
    - `StatusFlow: StateFlow<WelderStatus>` — parsed status updates
    - `connectionState: StateFlow<BleConnectionState>`
    - `writeParams(params)` — send PARAMS_WRITE
    - `sendCommand(cmd)` — send CMD packets
    - `readParams()` — one-shot params read
    - Handle notifications on TX characteristic → parse → emit to flows
- **VERIFY:** Connect to ESP32 (or mock), receive status notifications, send a param write and get ACK

#### Task B3: Repository Layer
- **Agent:** `mobile-developer`
- **Skills:** `clean-code`
- **Priority:** P1
- **Dependencies:** B2
- **INPUT:** BLE manager, DataStore
- **OUTPUT:**
  - `WelderRepository.kt` — Interface with `statusFlow`, `paramsFlow`, `connect()`, `writeParams()`, `loadPreset()`, etc.
  - `WelderRepositoryImpl.kt` — Delegates to BleManager, handles reconnection logic
  - `DeviceRepository.kt` — Persists paired devices (MAC, name, last seen) via DataStore
- **VERIFY:** Repository correctly exposes BLE data as Kotlin Flows

#### Task B4: Firmware BLE Protocol Update (ESP32 Side)
- **Agent:** `mobile-developer` (embedded context)
- **Skills:** `clean-code`
- **Priority:** P0 (must match app protocol)
- **Dependencies:** None (can be parallel with app tasks)
- **INPUT:** Current `ble_serial.c` + binary protocol spec
- **OUTPUT:**
  - Updated `ble_serial.c`:
    - Replace text protocol with binary packet encoding/decoding
    - Add CRC calculation
    - Add ACK/NAK responses for writes
    - Add VERSION_RESPONSE handler
    - Add PRESET_LIST_RESPONSE handler
    - Status notify sends binary `ble_status_packet_t` every 500ms
  - New `ble_protocol.h` — Shared packet structures and constants
  - Updated `ble_serial.h` — Same public API, different internal format
- **VERIFY:** `pio run` compiles, BLE protocol visible via nRF Connect app (raw bytes match spec)

---

### Phase C: UI Screens

#### Task C1: Scan Screen
- **Agent:** `mobile-developer`
- **Skills:** `mobile-design`
- **Priority:** P1
- **Dependencies:** A2, A3, B2
- **INPUT:** BLE scan results, saved devices
- **OUTPUT:**
  - `ScanScreen.kt`:
    - "Pull to scan" or scan button with animated radar/pulse effect
    - `DeviceCard.kt` for each discovered device (name, signal strength, last seen)
    - Saved devices section (previously connected)
    - Permission request dialog for BLE/Location
    - Connection progress indicator
  - `ScanViewModel.kt` — scan start/stop, device list, connect action
- **VERIFY:** Scan discovers ESP32, tap connects, navigates to Dashboard

#### Task C2: Dashboard Screen (Main)
- **Agent:** `mobile-developer`
- **Skills:** `mobile-design`, `frontend-design`
- **Priority:** P1
- **Dependencies:** A2, A3, B3, C1
- **INPUT:** `WelderStatus` flow from repository
- **OUTPUT:**
  - `DashboardScreen.kt`:
    - **Top:** Connection badge (device name + signal + state) — respects status bar safe area
    - **Voltage section:** `VoltageBar.kt` (gradient green→yellow→red with glow effect, percentage, actual V)
    - **Parameters:** 3× `ParamCard.kt` for P1/T/P2 (big value, +/- buttons, 0.5ms step)
    - **S value card** (only visible in AUTO mode)
    - **Mode toggle:** AUTO/MAN animated switch
    - **Chart:** `VoltageChart.kt` (rolling 30-point line chart using Vico)
    - **Bottom:** `WeldCounter.kt` (session/total) + `StatusIndicator.kt` (IDLE/READY/CHARGING/FIRING/ERROR with color coding)
    - All respects navigation bar safe area
  - `DashboardViewModel.kt` — observes repo flows, dispatches param writes
  - Custom components: `VoltageBar.kt`, `VoltageChart.kt`, `ParamCard.kt`, `StatusIndicator.kt`, `WeldCounter.kt`, `ConnectionBadge.kt`
- **VERIFY:** Live data updates, tap +/- changes values, mode toggle works, chart animates

#### Task C3: Presets Screen
- **Agent:** `mobile-developer`
- **Skills:** `mobile-design`
- **Priority:** P2
- **Dependencies:** A2, A3, B3
- **INPUT:** Preset list from BLE
- **OUTPUT:**
  - `PresetsScreen.kt`:
    - Grid/list of 10 `PresetCard.kt` (name, P1/T/P2 values, active indicator)
    - Tap to load preset (sends CMD to ESP32)
    - Long-press to edit name
    - "Save Current" button — saves current params to selected slot
    - Factory presets have a "lock" icon (can still be overwritten)
  - `PresetsViewModel.kt`
- **VERIFY:** Load preset → dashboard values update, save preset → persists on ESP32

#### Task C4: Settings Screen
- **Agent:** `mobile-developer`
- **Skills:** `mobile-design`
- **Priority:** P2
- **Dependencies:** A2, A3, B3
- **INPUT:** Device + app settings
- **OUTPUT:**
  - `SettingsScreen.kt`:
    - **Device settings** (synced via BLE): BLE device name, sound on/off, display brightness, display theme
    - **App settings** (local): Dark/Light mode toggle, auto-reconnect on/off
    - **Saved devices:** List of remembered devices with forget option
    - **Factory reset** button (with confirmation dialog)
    - **Weld counter reset** (session/total)
    - **About section:** Firmware version (from VERSION response), app version
  - `SettingsViewModel.kt`
- **VERIFY:** Toggle settings → ESP32 responds with ACK, app theme switches instantly

#### Task C5: Custom UI Components Polish
- **Agent:** `mobile-developer`
- **Skills:** `mobile-design`, `frontend-design`
- **Priority:** P2
- **Dependencies:** C2
- **INPUT:** All working screens
- **OUTPUT:**
  - `GlowButton.kt` — Custom button with neon glow/pulse animation
  - `VoltageBar.kt` polish — Canvas-drawn gradient bar with animated glow shader
  - `StatusIndicator.kt` polish — Pulsing dot animation for CHARGING, spark animation for FIRING
  - `ParamCard.kt` polish — Metallic card border, haptic feedback on +/-
  - Dark/Light mode transition animation
  - Connection lost overlay with reconnect animation
- **VERIFY:** Visual polish matches "premium" quality, all animations smooth at 60fps

---

### Phase D: Integration & Testing

#### Task D1: End-to-End Integration Test
- **Agent:** `mobile-developer`
- **Skills:** `testing-patterns`
- **Priority:** P1
- **Dependencies:** B4, C2
- **INPUT:** Flashed ESP32 + Android app
- **OUTPUT:**
  - Connect to real ESP32 via BLE
  - Verify status updates arrive and parse correctly
  - Verify param writes succeed (ACK received)
  - Verify preset load works
  - Verify reconnection after disconnect
  - Test at edge of BLE range
- **VERIFY:** Full cycle works: scan → connect → monitor → adjust → preset → disconnect → reconnect

#### Task D2: Unit Tests
- **Agent:** `mobile-developer`
- **Skills:** `testing-patterns`
- **Priority:** P2
- **Dependencies:** B1, B3
- **INPUT:** Protocol module, repository
- **OUTPUT:**
  - Protocol tests: encode/decode all packet types, CRC validation, edge cases (max values, zero values)
  - ViewModel tests: state transitions, error handling
  - Repository tests: mock BLE manager
- **VERIFY:** `./gradlew test` passes with ≥80% coverage on protocol module

#### Task D3: Error Handling & Edge Cases
- **Agent:** `mobile-developer`
- **Skills:** `clean-code`
- **Priority:** P2
- **Dependencies:** D1
- **INPUT:** Test results from D1
- **OUTPUT:**
  - BLE disconnection handling (show reconnect overlay, auto-retry 3×)
  - Permission denied handling (guide user to settings)
  - BLE off handling (prompt to enable)
  - Timeout handling (NAK not received within 2s → show error)
  - Invalid packet handling (CRC mismatch → log + ignore)
- **VERIFY:** App never crashes on BLE disconnect, always shows meaningful error state

---

### Phase X: Verification

- [ ] `./gradlew assembleDebug` succeeds
- [ ] `./gradlew test` passes
- [ ] App installs on API 26 device/emulator
- [ ] BLE scan finds ESP32 device
- [ ] Full control cycle works (connect → monitor → adjust → disconnect)
- [ ] Dark mode looks premium (no plain colors, has glow/metallic effects)
- [ ] Light mode looks clean and readable
- [ ] Status bar and navigation bar safe areas respected
- [ ] No purple/violet colors anywhere
- [ ] Touch targets ≥ 48dp
- [ ] Back navigation works correctly on all screens
- [ ] `python .agent/skills/mobile-design/scripts/mobile_audit.py` passes

---

## Architecture Diagram

```
┌───────────────────────────────────────────────────────────┐
│                    ANDROID APP                            │
│                                                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐ │
│  │  Scan    │  │Dashboard │  │ Presets  │  │ Settings │ │
│  │ Screen   │  │ Screen   │  │ Screen   │  │ Screen   │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘ │
│       │              │              │              │       │
│  ┌────▼─────┐  ┌────▼─────┐  ┌────▼─────┐  ┌────▼─────┐ │
│  │  Scan   │  │Dashboard │  │ Presets │  │ Settings │ │
│  │ViewModel│  │ViewModel │  │ViewModel│  │ViewModel │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘ │
│       │              │              │              │       │
│       └──────────────┼──────────────┼──────────────┘       │
│                      │              │                      │
│              ┌───────▼──────────────▼────────┐             │
│              │     WelderRepository          │             │
│              │  (Interface + Impl)           │             │
│              └───────────────┬───────────────┘             │
│                              │                             │
│              ┌───────────────▼───────────────┐             │
│              │     MyWeldBleManager          │             │
│              │  (Nordic BLE Library)         │             │
│              └───────────────┬───────────────┘             │
│                              │                             │
│              ┌───────────────▼───────────────┐             │
│              │     BleProtocol               │             │
│              │  (Binary encode/decode)       │             │
│              └───────────────┬───────────────┘             │
│                              │                             │
└──────────────────────────────┼─────────────────────────────┘
                               │ BLE GATT
                               │
┌──────────────────────────────┼─────────────────────────────┐
│              ESP32-S3        │                              │
│              ┌───────────────▼───────────────┐             │
│              │     ble_serial.c              │             │
│              │  (Binary protocol V2)         │             │
│              └───────────────┬───────────────┘             │
│                              │                             │
│              ┌───────────────▼───────────────┐             │
│              │  welding.c / settings.c / ui.c│             │
│              └───────────────────────────────┘             │
└────────────────────────────────────────────────────────────┘
```

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| BLE Android quirks (Samsung, Xiaomi) | High | Nordic BLE Library handles most; test on 2+ devices |
| Binary protocol sync issues | Medium | CRC + SYNC byte + timeout/retry logic |
| BLE MTU too small for preset list | Low | Chunked PRESET_LIST_RESPONSE or request one-by-one |
| Compose performance on old devices | Medium | Profile on API 26 device, use `remember` aggressively |
| Firmware binary protocol change | High | Version field in STATUS allows backward compat |

---

## Notes

- **Safety:** The app CANNOT trigger a weld. The physical START button is the only weld trigger. The app can only set parameters and monitor — this is enforced at the firmware level.
- **BLE Name:** Default is "MyWeld" — the app filters for devices advertising this service UUID, not by name, so custom names work.
- **Weld Log:** For V1, weld log is limited to the session/total counters. Full timestamped weld history would require SD card logging on the ESP32 (future feature).
- **Firmware backward compat:** If the app connects and receives a text-format response instead of binary, it should show "Firmware update required" message. Check first byte — if it's ASCII (> 0x20) instead of 0xAA, it's legacy text protocol.

# BLE OTA Firmware Update

## Goal
Enable users to update ESP32 firmware wirelessly via BLE from the Android app. User picks a `.bin` file locally OR the app auto-detects new GitHub releases. PIN authentication required before flashing. Progress displayed on both phone and device. Rollback on bad firmware.

---

## Architecture Overview

```
Android App                          ESP32 Firmware
──────────────                       ──────────────
┌──────────────┐                     ┌────────────────┐
│ FirmwareVM   │ ── OTA_BEGIN ──►    │ ota.c          │
│              │ ── OTA_DATA  ──►    │  esp_ota_*()   │
│ OtaService   │ ◄── OTA_ACK ──     │  partition mgr │
│              │ ── OTA_END   ──►    │  rollback      │
│ GitHub API   │ ◄── OTA_DONE ──     │                │
└──────────────┘                     └────────────────┘

BLE Protocol Extension (0x10–0x15):
  OTA_BEGIN  (0x10) App→ESP : [total_size_u32][sha256_32B][fw_major][fw_minor][fw_patch]
  OTA_DATA   (0x11) App→ESP : [seq_u16][chunk_data_up_to_240B]
  OTA_END    (0x12) App→ESP : (no payload — signals transfer complete)
  OTA_ACK    (0x13) ESP→App : [status_u8][progress_u8][seq_u16]
  OTA_ABORT  (0x14) Either  : [reason_u8]
  OTA_RESULT (0x15) ESP→App : [result_u8] (0=OK, 1=CRC_FAIL, 2=FLASH_ERR, 3=ROLLBACK)
```

## Key Design Decisions

1. **Partition Table**: Dual OTA slots (`ota_0`, `ota_1`) + `otadata`, 3MB per slot (fw is ~1.14MB)
2. **Chunk Size**: 240 bytes per `OTA_DATA` packet (fits in 255-byte protocol LEN limit)
3. **Flow Control**: ESP32 sends `OTA_ACK` per chunk; app waits before next chunk
4. **Welding Lockout**: Welding disabled during OTA (safety critical)
5. **ESP32 Display**: Shows OTA progress bar + percentage via LVGL overlay
6. **Rollback**: `esp_ota_mark_app_valid_cancel_rollback()` called after stable boot

---

## Tasks

### Phase 1: ESP32 Partition & OTA Foundation

- [x] **Task 1**: Update `partitions.csv` for dual OTA layout
  ```
  # Name,   Type, SubType, Offset,   Size
  nvs,      data, nvs,     0x9000,   0x6000,
  otadata,  data, ota,     0xF000,   0x2000,
  phy_init, data, phy,     0x11000,  0x1000,
  ota_0,    app,  ota_0,   0x20000,  0x300000,
  ota_1,    app,  ota_1,   0x320000, 0x300000,
  storage,  data, spiffs,  0x620000, 0x9E0000,
  ```
  → Verify: `idf.py build` succeeds, flash size < 3MB

- [x] **Task 2**: Add OTA protocol definitions to `ble_protocol.h`
  - New message types `0x10–0x15`
  - `ble_ota_begin_t` struct (39 bytes: u32 total_size + 32B SHA256 + 3B version)
  - `ble_ota_data_t` struct (2B seq + up to 240B data)
  - `ble_ota_ack_t` struct (1B status + 1B progress + 2B seq)
  - `ble_ota_result_t` struct (1B result)
  - OTA error codes
  → Verify: Compiles with no warnings

- [x] **Task 3**: Create `ota.c` / `ota.h` — ESP32 OTA engine
  - `ota_begin(total_size, sha256_expected)` → open inactive OTA partition, erase
  - `ota_write_chunk(seq, data, len)` → `esp_ota_write()`, track progress
  - `ota_finish()` → validate SHA256, `esp_ota_set_boot_partition()`, reboot
  - `ota_abort()` → cleanup, release partition handle
  - `ota_get_progress()` → returns 0–100
  - Internal state machine: `IDLE → RECEIVING → VALIDATING → REBOOTING`
  → Verify: Functions compile, state machine transitions correct

- [x] **Task 4**: Wire OTA into `ble_serial.c`
  - Add OTA characteristic (UUID `0x1238`) with WRITE+NOTIFY
  - Handle `OTA_BEGIN`, `OTA_DATA`, `OTA_END` in new `ota_access_cb()`
  - Check PIN auth before accepting `OTA_BEGIN` (reuse `s_authenticated` flag)
  - Disable status notify during OTA (to save bandwidth)
  - Lock out welding during OTA via `welding_set_ota_lock(true)`
  → Verify: OTA characteristic appears in GATT service scan

- [x] **Task 5**: Add rollback validation in `main.c`
  - On boot: check `esp_ota_check_rollback_is_possible()`
  - After successful init (display + BLE + welding all OK): `esp_ota_mark_app_valid_cancel_rollback()`
  - If boot fails 3 times → auto-rollback to previous firmware
  → Verify: Boot log shows "App marked valid" or rollback message

- [x] **Task 6**: OTA progress UI overlay on ESP32 display
  - Add `ui_show_ota_progress(uint8_t percent)` and `ui_hide_ota_progress()`
  - Full-screen overlay: "Firmware Update" title + progress bar + percentage
  - "DO NOT POWER OFF" warning text
  - Called from `ota.c` on each chunk ACK
  → Verify: Calling `ui_show_ota_progress(50)` displays 50% progress

### Phase 2: Android Protocol & OTA Service

- [x] **Task 7**: Add OTA protocol to `BleProtocol.kt`
  - `encodeOtaBegin(totalSize, sha256, major, minor, patch)` → packet
  - `encodeOtaData(seq, chunkData)` → packet
  - `encodeOtaEnd()` → packet
  - `encodeOtaAbort(reason)` → packet
  - `decodeOtaAck(payload)` → `OtaAckResult(status, progress, seq)`
  - `decodeOtaResult(payload)` → `OtaResult(success, errorCode)`
  → Verify: Unit tests for encode/decode

- [x] **Task 8**: Add OTA UUID + handlers to `BleUuids.kt` & `MyWeldBleManager.kt`
  - New UUID `CHAR_OTA = 0x1238`
  - Discover + subscribe to OTA characteristic
  - `sendOtaPacket()` method with write-with-response for reliability
  - Handle `OTA_ACK` and `OTA_RESULT` notifications
  - Expose `otaProgress: SharedFlow<OtaProgress>` and `otaResult: SharedFlow<OtaResult>`
  → Verify: OTA characteristic discovered on connect

- [x] **Task 9**: Create `OtaService.kt` — chunked transfer engine
  - Input: `InputStream` (from file picker or download) + file size
  - Compute SHA256 of entire binary before starting
  - Send `OTA_BEGIN` → wait for ACK
  - Loop: read 240B chunk → `OTA_DATA(seq)` → wait for ACK → update progress
  - Send `OTA_END` → wait for `OTA_RESULT`
  - Timeout handling: 5s per chunk, 3 retries before abort
  - Cancel support (user can cancel mid-transfer)
  - Emit progress events for UI
  → Verify: Can send a test binary of ~1MB, progress flows correctly

- [x] **Task 10**: Create `FirmwareViewModel.kt`
  - State: `currentVersion`, `availableVersion`, `otaState` (Idle/Checking/Downloading/Uploading/Done/Error)
  - `checkGitHubReleases()` — fetch latest release from GitHub API, compare versions
  - `startOtaFromFile(uri: Uri)` — file picker flow
  - `startOtaFromGitHub(releaseUrl)` — download + flash flow
  - Require PIN auth before starting (verify `isAuthenticated` from BleManager)
  → Verify: ViewModel states transition correctly

### Phase 3: Android UI — Firmware Update Screen

- [x] **Task 11**: Create `FirmwareUpdateScreen.kt`
  - **Top section**: Current firmware version (from device status)
  - **Update available card**: Shows new version + changelog (from GitHub)
  - **Manual update button**: Opens file picker (`.bin` files)
  - **Auto-update button**: Downloads latest from GitHub
  - **Progress section** (during OTA): Circular progress + percentage + "Do not disconnect" warning
  - **Result section**: Success (with reboot countdown) or Error (with retry option)
  - PIN dialog before starting OTA
  → Verify: Screen renders, file picker opens, progress animates

- [x] **Task 12**: Add navigation route + menu entry
  - Add "Firmware Update" to navigation graph
  - Add entry in Settings screen or bottom nav
  - Badge indicator when update available (optional)
  → Verify: Can navigate to Firmware Update screen from Settings

### Phase 4: GitHub Release Integration

- [x] **Task 13**: Create `GitHubReleaseChecker.kt`
  - Uses GitHub REST API: `GET /repos/{owner}/{repo}/releases/latest`
  - Parse version tag (e.g., `v1.0.1`) → compare with device version
  - Download `.bin` asset URL
  - Cache check result (don't spam API — check max once per app launch)
  - No API key required for public repos
  → Verify: Fetches release info from test GitHub repo

### Phase 5: Verification

- [x] **Task 14**: End-to-end test
  - Build firmware v1.0.1 with a small change (e.g., version string)
  - Flash v1.0.0 normally via USB
  - Use Android app to OTA-flash v1.0.1 via file picker
  - Verify: Device reboots, shows v1.0.1, rollback protection active
  - Verify: Progress shown on both phone and device display
  - Test abort mid-transfer → device stays on old firmware
  - Test rollback: flash bad binary → device auto-reverts

---

## Done When
- [ ] ESP32 boots from OTA partition with rollback protection
- [ ] Android app can pick `.bin` file and flash over BLE with progress
- [ ] Android app can check GitHub releases and auto-download + flash
- [ ] PIN required before OTA starts
- [ ] Progress shown on both Android app and ESP32 display
- [ ] Welding locked out during OTA
- [ ] Abort and rollback work correctly

## Notes
- **Chunk math**: 1.14MB / 240B = ~4,979 chunks. At ~30KB/s = ~38 seconds. Acceptable.
- **MTU negotiation**: NordicSemi BleManager handles MTU automatically; request 517 for max throughput.
- **GitHub repo**: User needs to provide `{owner}/{repo}` — can be hardcoded or configurable in app settings.
- **sdkconfig**: May need `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` and `CONFIG_BOOTLOADER_OTA_DATA_ERASE=y`.
- **Current protocol LEN limit**: LEN is `uint8_t` (max 255). OTA_DATA payload = 2B seq + 240B data = 242B ✅ fits.

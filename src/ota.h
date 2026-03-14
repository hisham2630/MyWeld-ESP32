#ifndef OTA_H
#define OTA_H

/**
 * OTA Firmware Update Module
 *
 * Handles BLE-based firmware updates using ESP-IDF's OTA API.
 * Writes to the inactive OTA partition, validates SHA-256, and
 * switches boot partition on success.
 *
 * Flow:
 *   1. App sends OTA_BEGIN (size + SHA256 + version)
 *   2. App sends OTA_DATA chunks (seq + data)
 *   3. App sends OTA_END
 *   4. ESP validates SHA256, sets boot partition, reboots
 *
 * Safety:
 *   - Welding is locked out during OTA
 *   - Rollback protection on bad firmware
 *   - SHA-256 validation before boot switch
 */

#include "ble_protocol.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize OTA module. Call once at startup.
 * Checks for pending rollback and marks app valid if boot succeeded.
 */
void ota_init(void);

/**
 * Begin an OTA update session.
 * @param begin  OTA_BEGIN payload from the app
 * @return BLE_OTA_STATUS_OK on success, error code otherwise
 */
uint8_t ota_begin(const ble_ota_begin_t *begin);

/**
 * Write a firmware chunk.
 * @param seq       Expected sequence number
 * @param data      Chunk data bytes
 * @param data_len  Number of bytes in chunk (1–240)
 * @return BLE_OTA_STATUS_OK on success, error code otherwise
 */
uint8_t ota_write_chunk(uint16_t seq, const uint8_t *data, uint16_t data_len);

/**
 * Finalize OTA: validate SHA-256 and set boot partition.
 * @return BLE_OTA_RESULT_SUCCESS on success, error code otherwise
 */
uint8_t ota_finish(void);

/**
 * Abort the current OTA session and clean up.
 */
void ota_abort(void);

/**
 * Get current OTA state.
 */
ota_state_t ota_get_state(void);

/**
 * Get current OTA progress (0–100%).
 */
uint8_t ota_get_progress(void);

/**
 * Check if OTA is currently in progress.
 */
bool ota_is_active(void);

#endif // OTA_H

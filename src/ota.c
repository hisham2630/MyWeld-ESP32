/**
 * OTA Firmware Update Module — Implementation
 *
 * Uses ESP-IDF's esp_ota_ops API to write firmware to the inactive
 * OTA partition. Validates SHA-256 hash after complete transfer.
 *
 * Safety features:
 *   - Welding locked out during OTA (via welding module)
 *   - SHA-256 verified before switching boot partition
 *   - Rollback on boot failure (esp_ota_mark_app_valid)
 *   - Status notifications suppress during OTA (bandwidth)
 */

#include "ota.h"
#include "ble_protocol.h"
#include "config.h"
#include "ui.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "mbedtls/sha256.h"
#include <string.h>

static const char *TAG = "OTA";

// OTA session state
static ota_state_t       s_state       = OTA_STATE_IDLE;
static esp_ota_handle_t  s_ota_handle  = 0;
static const esp_partition_t *s_update_partition = NULL;
static uint32_t          s_total_size  = 0;
static uint32_t          s_received    = 0;
static uint16_t          s_expected_seq = 0;
static uint8_t           s_expected_sha256[32];

// Running SHA-256 context for validation
static mbedtls_sha256_context s_sha256_ctx;

// ============================================================================
// Public: Init — rollback check
// ============================================================================

void ota_init(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // We booted from a new OTA image that hasn't been confirmed yet.
            // Mark it valid now that we've successfully reached this point.
            esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA: New firmware validated — rollback cancelled");
        }
    }

    ESP_LOGI(TAG, "Running from partition '%s' @ 0x%08lx",
             running->label, (unsigned long)running->address);

    s_state = OTA_STATE_IDLE;
}

// ============================================================================
// Public: Begin OTA session
// ============================================================================

uint8_t ota_begin(const ble_ota_begin_t *begin)
{
    if (s_state != OTA_STATE_IDLE) {
        ESP_LOGW(TAG, "OTA already in progress");
        return BLE_OTA_STATUS_BUSY;
    }

    // Find the next OTA partition to write to
    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (s_update_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        return BLE_OTA_STATUS_FLASH_ERR;
    }

    // Check if firmware fits
    if (begin->total_size > s_update_partition->size) {
        ESP_LOGE(TAG, "Firmware too large: %lu > partition %lu",
                 (unsigned long)begin->total_size,
                 (unsigned long)s_update_partition->size);
        return BLE_OTA_STATUS_TOO_LARGE;
    }

    if (begin->total_size == 0) {
        ESP_LOGE(TAG, "Firmware size is 0");
        return BLE_OTA_STATUS_FLASH_ERR;
    }

    // Start OTA write session
    esp_err_t err = esp_ota_begin(s_update_partition, begin->total_size, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        s_update_partition = NULL;
        return BLE_OTA_STATUS_FLASH_ERR;
    }

    // Store expected hash and size
    s_total_size = begin->total_size;
    s_received   = 0;
    s_expected_seq = 0;
    memcpy(s_expected_sha256, begin->sha256, 32);

    // Initialize SHA-256 context for running hash
    mbedtls_sha256_init(&s_sha256_ctx);
    mbedtls_sha256_starts(&s_sha256_ctx, 0); // 0 = SHA-256 (not SHA-224)

    s_state = OTA_STATE_RECEIVING;

    ESP_LOGI(TAG, "OTA started: %lu bytes → partition '%s' @ 0x%08lx",
             (unsigned long)s_total_size,
             s_update_partition->label,
             (unsigned long)s_update_partition->address);
    ESP_LOGI(TAG, "New firmware version: %d.%d.%d",
             begin->fw_major, begin->fw_minor, begin->fw_patch);

    // Show progress overlay on display
    ui_show_ota_progress(0);

    return BLE_OTA_STATUS_OK;
}

// ============================================================================
// Public: Write a firmware chunk
// ============================================================================

uint8_t ota_write_chunk(uint16_t seq, const uint8_t *data, uint16_t data_len)
{
    if (s_state != OTA_STATE_RECEIVING) {
        return BLE_OTA_STATUS_BUSY;
    }

    // Validate sequence number
    if (seq != s_expected_seq) {
        ESP_LOGW(TAG, "OTA seq mismatch: got %u, expected %u", seq, s_expected_seq);
        return BLE_OTA_STATUS_SEQ_ERR;
    }

    // Don't write past total size
    if (s_received + data_len > s_total_size) {
        data_len = (uint16_t)(s_total_size - s_received);
    }

    if (data_len == 0) {
        return BLE_OTA_STATUS_OK;
    }

    // Write to flash
    esp_err_t err = esp_ota_write(s_ota_handle, data, data_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed at offset %lu: %s",
                 (unsigned long)s_received, esp_err_to_name(err));
        return BLE_OTA_STATUS_FLASH_ERR;
    }

    // Update running SHA-256
    mbedtls_sha256_update(&s_sha256_ctx, data, data_len);

    s_received += data_len;
    s_expected_seq++;

    // Update display progress
    uint8_t progress = (uint8_t)((uint64_t)s_received * 100 / s_total_size);
    ui_show_ota_progress(progress);

    // Log every 10%
    static uint8_t last_logged = 0;
    if (progress / 10 != last_logged / 10) {
        ESP_LOGI(TAG, "OTA progress: %u%% (%lu / %lu bytes)",
                 progress, (unsigned long)s_received, (unsigned long)s_total_size);
        last_logged = progress;
    }

    return BLE_OTA_STATUS_OK;
}

// ============================================================================
// Public: Finish OTA — validate & switch boot partition
// ============================================================================

uint8_t ota_finish(void)
{
    if (s_state != OTA_STATE_RECEIVING) {
        return BLE_OTA_RESULT_ABORTED;
    }

    s_state = OTA_STATE_VALIDATING;
    ui_show_ota_progress(100);

    // Check we received all bytes
    if (s_received != s_total_size) {
        ESP_LOGE(TAG, "Size mismatch: received %lu, expected %lu",
                 (unsigned long)s_received, (unsigned long)s_total_size);
        ota_abort();
        return BLE_OTA_RESULT_FLASH_ERR;
    }

    // Compute final SHA-256
    uint8_t computed_sha256[32];
    mbedtls_sha256_finish(&s_sha256_ctx, computed_sha256);
    mbedtls_sha256_free(&s_sha256_ctx);

    // Compare with expected
    if (memcmp(computed_sha256, s_expected_sha256, 32) != 0) {
        ESP_LOGE(TAG, "SHA-256 mismatch!");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, s_expected_sha256, 32, ESP_LOG_ERROR);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, computed_sha256, 32, ESP_LOG_ERROR);
        ota_abort();
        return BLE_OTA_RESULT_CRC_FAIL;
    }

    ESP_LOGI(TAG, "SHA-256 verified OK");

    // End OTA write session
    esp_err_t err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        s_state = OTA_STATE_ERROR;
        return BLE_OTA_RESULT_FLASH_ERR;
    }

    // Set boot partition to the newly written one
    err = esp_ota_set_boot_partition(s_update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        s_state = OTA_STATE_ERROR;
        return BLE_OTA_RESULT_FLASH_ERR;
    }

    s_state = OTA_STATE_REBOOTING;

    ESP_LOGI(TAG, "OTA complete! Boot partition set to '%s'. Rebooting in 2s...",
             s_update_partition->label);

    return BLE_OTA_RESULT_SUCCESS;
}

// ============================================================================
// Public: Abort OTA
// ============================================================================

void ota_abort(void)
{
    if (s_state == OTA_STATE_IDLE) return;

    ESP_LOGW(TAG, "OTA aborted (state=%d, received=%lu/%lu)",
             s_state, (unsigned long)s_received, (unsigned long)s_total_size);

    if (s_ota_handle != 0) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
    }

    mbedtls_sha256_free(&s_sha256_ctx);

    s_update_partition = NULL;
    s_total_size  = 0;
    s_received    = 0;
    s_expected_seq = 0;
    s_state = OTA_STATE_IDLE;

    // Hide OTA overlay on display
    ui_hide_ota_progress();
}

// ============================================================================
// Public: Getters
// ============================================================================

ota_state_t ota_get_state(void)
{
    return s_state;
}

uint8_t ota_get_progress(void)
{
    if (s_total_size == 0) return 0;
    return (uint8_t)((uint64_t)s_received * 100 / s_total_size);
}

bool ota_is_active(void)
{
    return s_state != OTA_STATE_IDLE;
}

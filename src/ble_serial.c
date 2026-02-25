/**
 * BLE Serial Module — Binary Protocol V2 Implementation
 *
 * Replaces the old text-based protocol with a structured binary format.
 * Packet format: [SYNC=0xAA] [TYPE] [LEN] [PAYLOAD...] [CRC]
 *
 * Changes from V1:
 *   - STATUS notifications now send 32-byte binary packets (was ~80 byte text)
 *   - PARAMS characteristic accepts binary PARAMS_WRITE packets
 *   - CMD characteristic accepts binary CMD packets with sub-commands
 *   - ACK/NAK responses for every write operation
 *   - CRC validation on all incoming packets
 *
 * Safety: BLE CANNOT trigger weld — physical START button required.
 */

#include "ble_serial.h"
#include "ble_protocol.h"
#include "settings.h"
#include "welding.h"
#include "audio.h"
#include "ui.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "BLE";

static bool     s_connected     = false;
static bool     s_authenticated = false;  // Reset on every disconnect
static uint16_t s_conn_handle = 0;
static uint16_t s_status_attr_handle = 0;

// Forward declaration — defined later in this file
static int gap_event_cb(struct ble_gap_event *event, void *arg);

// ============================================================================
// UUIDs (same as V1 — no change needed)
// ============================================================================
//
// NimBLE BLE_UUID128_INIT() stores bytes in little-endian (LSB-first) order.
// UUID string: 00001234-0000-1000-8000-00805F9B34FB
// As 16 bytes (BE): 00 00 12 34  00 00  10 00  80 00  00 80 5F 9B 34 FB
// Fully reversed:   FB 34 9B 5F  80 00  00 80  00 10  00 00 34 12 00 00
//
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x34, 0x12, 0x00, 0x00);

static const ble_uuid128_t chr_params_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x35, 0x12, 0x00, 0x00);

static const ble_uuid128_t chr_status_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x36, 0x12, 0x00, 0x00);

static const ble_uuid128_t chr_cmd_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x37, 0x12, 0x00, 0x00);

// ============================================================================
// Helper: map firmware weld_state_t → protocol state byte
// ============================================================================

static uint8_t map_weld_state(weld_state_t state)
{
    switch (state) {
        case WELD_STATE_IDLE:      return 0; // IDLE
        case WELD_STATE_ARMED:     return 1; // READY (armed = ready to fire)
        case WELD_STATE_PRE_FIRE:  return 2; // CHARGING (pre-fire settling)
        case WELD_STATE_FIRING_P1:
        case WELD_STATE_PAUSE:
        case WELD_STATE_FIRING_P2: return 3; // FIRING
        case WELD_STATE_COOLDOWN:  return 4; // cooldown
        case WELD_STATE_BLOCKED:   return 5; // BLOCKED
        case WELD_STATE_ERROR:     return 6; // ERROR
        default:                   return 0;
    }
}

// ============================================================================
// Helper: compute charge percentage
// ============================================================================

static uint8_t compute_charge_percent(float voltage)
{
    if (voltage <= LOW_VOLTAGE_BLOCK) return 0;
    if (voltage >= SUPERCAP_MAX_V)    return 100;
    return (uint8_t)((voltage - LOW_VOLTAGE_BLOCK) /
                     (SUPERCAP_MAX_V - LOW_VOLTAGE_BLOCK) * 100.0f);
}

// ============================================================================
// Helper: send ACK notification
// ============================================================================

static void send_ack(uint8_t original_type)
{
    if (!s_connected || s_status_attr_handle == 0) return;

    uint8_t payload[2] = { original_type, 0x00 };
    uint8_t pkt[BLE_PROTO_HEADER_SIZE + 2 + BLE_PROTO_CRC_SIZE];
    int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_ACK, payload, 2);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, pkt_len);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
    }
}

// ============================================================================
// Helper: send NAK notification
// ============================================================================

static void send_nak(uint8_t original_type, uint8_t error_code)
{
    if (!s_connected || s_status_attr_handle == 0) return;

    uint8_t payload[3] = { original_type, error_code, 0x00 };
    uint8_t pkt[BLE_PROTO_HEADER_SIZE + 3 + BLE_PROTO_CRC_SIZE];
    int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_NAK, payload, 3);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, pkt_len);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
    }
}

// ============================================================================
// Helper: validate and extract a binary packet from raw BLE data
// Returns true if valid packet found, populates type/payload/payload_len
// ============================================================================

static bool validate_packet(const uint8_t *data, uint16_t data_len,
                            uint8_t *out_type, const uint8_t **out_payload,
                            uint8_t *out_payload_len)
{
    if (data_len < BLE_PROTO_HEADER_SIZE + BLE_PROTO_CRC_SIZE) return false;
    if (data[0] != BLE_PROTO_SYNC) return false;

    uint8_t type = data[1];
    uint8_t len  = data[2];

    if (data_len < (uint16_t)(BLE_PROTO_HEADER_SIZE + len + BLE_PROTO_CRC_SIZE)) return false;

    const uint8_t *payload = data + BLE_PROTO_HEADER_SIZE;
    uint8_t expected_crc = ble_proto_calc_crc(type, len, payload);
    uint8_t actual_crc   = data[BLE_PROTO_HEADER_SIZE + len];

    if (expected_crc != actual_crc) {
        ESP_LOGW(TAG, "CRC mismatch: expected 0x%02X, got 0x%02X", expected_crc, actual_crc);
        return false;
    }

    *out_type = type;
    *out_payload = payload;
    *out_payload_len = len;
    return true;
}

// ============================================================================
// Helper: build status packet payload from current state
// ============================================================================

static void build_status_payload(ble_status_packet_t *pkt)
{
    memset(pkt, 0, sizeof(*pkt));

    pkt->supercap_mv    = (uint16_t)(g_weld_status.supercap_voltage * 1000.0f);
    pkt->protection_mv  = (uint16_t)(g_weld_status.protection_voltage * 1000.0f);
    pkt->state          = map_weld_state(g_weld_state);
    pkt->charge_percent = compute_charge_percent(g_weld_status.supercap_voltage);
    pkt->auto_mode      = g_settings.auto_mode ? 1 : 0;
    pkt->active_preset  = g_settings.active_preset;
    pkt->session_welds  = g_settings.session_welds;
    pkt->total_welds    = g_settings.total_welds;
    pkt->ble_connected  = s_connected ? 1 : 0;
    pkt->sound_on       = g_settings.sound_on ? 1 : 0;
    pkt->theme          = g_settings.theme;
    pkt->error_code     = (g_weld_state == WELD_STATE_ERROR)   ? 0xFF :
                          (g_weld_state == WELD_STATE_BLOCKED) ? BLE_ERR_BUSY : 0;
    pkt->p1_x10         = (uint16_t)(g_settings.p1 * 10.0f);
    pkt->t_x10          = (uint16_t)(g_settings.t * 10.0f);
    pkt->p2_x10         = (uint16_t)(g_settings.p2 * 10.0f);
    pkt->s_x10          = (uint16_t)(g_settings.s_value * 10.0f);
    pkt->fw_major       = FW_VERSION_MAJOR;
    pkt->fw_minor       = FW_VERSION_MINOR;
}

// ============================================================================
// PARAMS characteristic handler (READ → binary params, WRITE → binary params)
// ============================================================================

static int params_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Build a PARAMS_RESPONSE packet
        ble_params_packet_t params;
        memset(&params, 0, sizeof(params));
        params.p1_x10    = (uint16_t)(g_settings.p1 * 10.0f);
        params.t_x10     = (uint16_t)(g_settings.t * 10.0f);
        params.p2_x10    = (uint16_t)(g_settings.p2 * 10.0f);
        params.s_x10     = (uint16_t)(g_settings.s_value * 10.0f);
        params.auto_mode = g_settings.auto_mode ? 1 : 0;
        params.sound_on  = g_settings.sound_on ? 1 : 0;
        params.brightness = g_settings.brightness;
        params.theme     = g_settings.theme;
        strncpy(params.ble_name, g_settings.ble_name, sizeof(params.ble_name) - 1);

        uint8_t pkt[BLE_PROTO_HEADER_SIZE + sizeof(params) + BLE_PROTO_CRC_SIZE];
        int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_PARAMS_RESPONSE,
                                              (const uint8_t *)&params, sizeof(params));
        os_mbuf_append(ctxt->om, pkt, pkt_len);
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t buf[64];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(buf)) len = sizeof(buf) - 1;
        os_mbuf_copydata(ctxt->om, 0, len, buf);

        // Validate binary packet
        uint8_t msg_type;
        const uint8_t *payload;
        uint8_t payload_len;

        if (!validate_packet(buf, len, &msg_type, &payload, &payload_len)) {
            ESP_LOGW(TAG, "Invalid packet on PARAMS write (len=%d)", len);
            send_nak(BLE_MSG_PARAMS_WRITE, BLE_ERR_INVALID_RANGE);
            return 0;
        }

        if (msg_type != BLE_MSG_PARAMS_WRITE || payload_len < sizeof(ble_params_packet_t)) {
            ESP_LOGW(TAG, "Wrong type (0x%02X) or short payload (%d)", msg_type, payload_len);
            send_nak(BLE_MSG_PARAMS_WRITE, BLE_ERR_INVALID_RANGE);
            return 0;
        }

        // Decode parameters
        const ble_params_packet_t *p = (const ble_params_packet_t *)payload;

        float new_p1 = p->p1_x10 / 10.0f;
        float new_t  = p->t_x10  / 10.0f;
        float new_p2 = p->p2_x10 / 10.0f;
        float new_s  = p->s_x10  / 10.0f;

        // Validate ranges
        if (new_p1 < PULSE_MIN_MS || new_p1 > PULSE_MAX_MS ||
            new_t  < PULSE_MIN_MS || new_t  > PULSE_MAX_MS ||
            new_p2 < PULSE_MIN_MS || new_p2 > PULSE_MAX_MS ||
            new_s  < S_VALUE_MIN  || new_s  > S_VALUE_MAX) {
            ESP_LOGW(TAG, "Params out of range: P1=%.1f T=%.1f P2=%.1f S=%.1f",
                     new_p1, new_t, new_p2, new_s);
            send_nak(BLE_MSG_PARAMS_WRITE, BLE_ERR_INVALID_RANGE);
            return 0;
        }

        // Reject parameter changes while firing
        if (g_weld_state == WELD_STATE_FIRING_P1 ||
            g_weld_state == WELD_STATE_PAUSE ||
            g_weld_state == WELD_STATE_FIRING_P2) {
            send_nak(BLE_MSG_PARAMS_WRITE, BLE_ERR_BUSY);
            return 0;
        }

        // Reject if not authenticated
        if (!s_authenticated) {
            send_nak(BLE_MSG_PARAMS_WRITE, BLE_ERR_AUTH_REQUIRED);
            return 0;
        }

        // Apply
        g_settings.p1        = new_p1;
        g_settings.t         = new_t;
        g_settings.p2        = new_p2;
        g_settings.s_value   = new_s;
        g_settings.auto_mode = p->auto_mode != 0;
        g_settings.sound_on  = p->sound_on != 0;
        g_settings.brightness = p->brightness;
        g_settings.theme     = p->theme;

        // Update BLE name if changed
        if (p->ble_name[0] != '\0') {
            if (strncmp(g_settings.ble_name, p->ble_name, sizeof(p->ble_name)) != 0) {
                strncpy(g_settings.ble_name, p->ble_name, sizeof(g_settings.ble_name) - 1);
                g_settings.ble_name[sizeof(g_settings.ble_name) - 1] = '\0';

                // Update the GAP service name record
                ble_svc_gap_device_name_set(g_settings.ble_name);

                // Re-apply advertising data so scanners see the new name immediately
                // without requiring a device reboot.
                ble_gap_adv_stop();
                struct ble_hs_adv_fields fields = {0};
                fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
                fields.name = (uint8_t *)g_settings.ble_name;
                fields.name_len = strlen(g_settings.ble_name);
                fields.name_is_complete = 1;
                fields.uuids128 = (ble_uuid128_t[]) { svc_uuid };
                fields.num_uuids128 = 1;
                fields.uuids128_is_complete = 1;
                ble_gap_adv_set_fields(&fields);

                struct ble_gap_adv_params adv_params = {0};
                adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
                adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
                ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                  &adv_params, gap_event_cb, NULL);

                ESP_LOGI(TAG, "BLE device name updated to: '%s'", g_settings.ble_name);
            }
        }

        settings_save();
        ESP_LOGI(TAG, "Params updated: P1=%.1f T=%.1f P2=%.1f S=%.1f MODE=%s",
                 new_p1, new_t, new_p2, new_s,
                 g_settings.auto_mode ? "AUTO" : "MAN");

        // Refresh ESP32 UI labels — g_settings was updated from BLE
        ui_refresh_params();

        send_ack(BLE_MSG_PARAMS_WRITE);
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

// ============================================================================
// STATUS characteristic handler (READ/NOTIFY → binary status)
// ============================================================================

static int status_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ble_status_packet_t status;
        build_status_payload(&status);

        uint8_t pkt[BLE_PROTO_HEADER_SIZE + sizeof(status) + BLE_PROTO_CRC_SIZE];
        int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_STATUS,
                                              (const uint8_t *)&status, sizeof(status));
        os_mbuf_append(ctxt->om, pkt, pkt_len);
    }
    return 0;
}

// ============================================================================
// CMD characteristic handler (WRITE → binary commands)
// ============================================================================

static int cmd_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint8_t buf[256];
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    os_mbuf_copydata(ctxt->om, 0, len, buf);

    // Validate binary packet
    uint8_t msg_type;
    const uint8_t *payload;
    uint8_t payload_len;

    if (!validate_packet(buf, len, &msg_type, &payload, &payload_len)) {
        ESP_LOGW(TAG, "Invalid CMD packet (len=%d)", len);
        send_nak(BLE_MSG_CMD, BLE_ERR_UNKNOWN_CMD);
        return 0;
    }

    if (msg_type == BLE_MSG_PRESET_LIST) {
        // Authentication required to read preset names
        if (!s_authenticated) {
            send_nak(BLE_MSG_PRESET_LIST, BLE_ERR_AUTH_REQUIRED);
            return 0;
        }
        // Send preset names in 2 paged notifications (10 presets × 20 bytes each = 201b/page)
        for (uint8_t page = 0; page < (BLE_MAX_PRESETS / BLE_PRESETS_PER_PAGE); page++) {
            ble_preset_list_resp_t resp;
            memset(&resp, 0, sizeof(resp));
            resp.page_index = page;
            int base = page * BLE_PRESETS_PER_PAGE;
            for (int i = 0; i < BLE_PRESETS_PER_PAGE && (base + i) < MAX_PRESETS; i++) {
                strncpy(resp.names[i], g_settings.presets[base + i].name,
                        BLE_PRESET_NAME_LEN - 1);
            }
            uint8_t pkt[BLE_PROTO_HEADER_SIZE + sizeof(resp) + BLE_PROTO_CRC_SIZE];
            int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_PRESET_LIST_RESP,
                                                  (const uint8_t *)&resp, sizeof(resp));
            struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, pkt_len);
            if (om) {
                ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
            }
        }
        ESP_LOGI(TAG, "Sent preset list (2 pages, %d presets total)", BLE_MAX_PRESETS);
        return 0;
    }

    if (msg_type == BLE_MSG_VERSION) {
        // Version request — respond with version info
        ble_version_packet_t ver;
        memset(&ver, 0, sizeof(ver));
        ver.major = FW_VERSION_MAJOR;
        ver.minor = FW_VERSION_MINOR;
        ver.patch = FW_VERSION_PATCH;
        // Build date: extract YYMMDD from __DATE__ ("Feb 23 2026")
        snprintf(ver.build_date, sizeof(ver.build_date), "%d%02d%02d",
                 FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);

        uint8_t pkt[BLE_PROTO_HEADER_SIZE + sizeof(ver) + BLE_PROTO_CRC_SIZE];
        int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_VERSION_RESPONSE,
                                              (const uint8_t *)&ver, sizeof(ver));

        struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, pkt_len);
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
        }
        return 0;
    }

    if (msg_type != BLE_MSG_CMD || payload_len < 1) {
        send_nak(BLE_MSG_CMD, BLE_ERR_UNKNOWN_CMD);
        return 0;
    }

    uint8_t sub_cmd = payload[0];

    // BLE_CMD_AUTH is always allowed (even before authentication)
    if (sub_cmd == BLE_CMD_AUTH) {
        if (payload_len < 6) {
            send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_RANGE);
            return 0;
        }
        char pin[PIN_MAX_LEN];
        memset(pin, 0, sizeof(pin));
        memcpy(pin, &payload[1], PIN_MAX_LEN - 1);
        pin[PIN_MAX_LEN - 1] = '\0';

        if (settings_verify_pin(pin)) {
            s_authenticated = true;
            ESP_LOGI(TAG, "BLE authenticated successfully");
            send_ack(BLE_MSG_CMD);
        } else {
            s_authenticated = false;
            ESP_LOGW(TAG, "BLE auth failed — wrong PIN");
            send_nak(BLE_MSG_CMD, BLE_ERR_AUTH_FAILED);
        }
        return 0;
    }

    // All other commands require authentication
    if (!s_authenticated) {
        ESP_LOGW(TAG, "CMD 0x%02X rejected — not authenticated", sub_cmd);
        send_nak(BLE_MSG_CMD, BLE_ERR_AUTH_REQUIRED);
        return 0;
    }

    switch (sub_cmd) {
        case BLE_CMD_LOAD_PRESET: {
            if (payload_len < 2) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_PRESET);
                return 0;
            }
            uint8_t idx = payload[1];
            if (idx >= MAX_PRESETS) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_PRESET);
                return 0;
            }
            settings_load_preset(idx);
            ESP_LOGI(TAG, "Loaded preset %d: '%s'", idx, g_settings.presets[idx].name);
            // Refresh ESP32 display immediately — don't wait for next UI cycle
            ui_refresh_params();
            send_ack(BLE_MSG_CMD);
            break;
        }

        case BLE_CMD_SAVE_PRESET: {
            // Payload: [sub_cmd, index, name[20], p1_x10(2), t_x10(2), p2_x10(2), s_x10(2), auto_mode(1)]
            // = 1 + 1 + 20 + 2 + 2 + 2 + 2 + 1 = 31 bytes minimum
            if (payload_len < 30) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_RANGE);
                return 0;
            }
            uint8_t idx = payload[1];
            if (idx >= MAX_PRESETS) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_PRESET);
                return 0;
            }

            // Extract name (20 bytes, null-terminated)
            char name[PRESET_NAME_LEN];
            memset(name, 0, sizeof(name));
            memcpy(name, &payload[2], PRESET_NAME_LEN);
            name[PRESET_NAME_LEN - 1] = '\0';

            // Extract weld parameters (little-endian uint16 × 0.1)
            uint16_t p1_x10  = (uint16_t)(payload[22] | (payload[23] << 8));
            uint16_t t_x10   = (uint16_t)(payload[24] | (payload[25] << 8));
            uint16_t p2_x10  = (uint16_t)(payload[26] | (payload[27] << 8));
            uint16_t s_x10   = (uint16_t)(payload[28] | (payload[29] << 8));
            bool     am      = (payload_len > 30) ? (payload[30] != 0) : false;

            float p1      = p1_x10 / 10.0f;
            float t       = t_x10  / 10.0f;
            float p2      = p2_x10 / 10.0f;
            float s_value = s_x10  / 10.0f;

            // Validate ranges
            if (p1 < PULSE_MIN_MS || p1 > PULSE_MAX_MS ||
                t  < PULSE_MIN_MS || t  > PULSE_MAX_MS ||
                p2 < PULSE_MIN_MS || p2 > PULSE_MAX_MS ||
                s_value < S_VALUE_MIN || s_value > S_VALUE_MAX) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_RANGE);
                return 0;
            }

            settings_save_preset(idx, name, p1, t, p2, s_value, am);
            ESP_LOGI(TAG, "Saved preset %d: '%s'", idx, name);
            send_ack(BLE_MSG_CMD);
            break;
        }

        case BLE_CMD_FACTORY_RESET:
            ESP_LOGW(TAG, "Factory reset requested via BLE");
            settings_factory_reset();
            send_ack(BLE_MSG_CMD);              // Notify app before reboot
            ui_trigger_reboot_countdown();      // 3-2-1 on screen, then esp_restart()
            break;

        case BLE_CMD_RESET_WELD_COUNTER: {
            if (payload_len < 2) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_RANGE);
                return 0;
            }
            uint8_t target = payload[1];
            if (target == 0 || target == 2) {
                g_settings.session_welds = 0;
            }
            if (target == 1 || target == 2) {
                g_settings.total_welds = 0;
            }
            settings_save_now();
            ESP_LOGI(TAG, "Weld counter reset (target=%d)", target);
            send_ack(BLE_MSG_CMD);
            break;
        }

        case BLE_CMD_CHANGE_PIN: {
            if (payload_len < 6) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_RANGE);
                return 0;
            }
            char new_pin[PIN_MAX_LEN];
            memset(new_pin, 0, sizeof(new_pin));
            memcpy(new_pin, &payload[1], PIN_MAX_LEN - 1);
            new_pin[PIN_MAX_LEN - 1] = '\0';
            // Validate: must be exactly 4 ASCII digits
            bool valid = true;
            for (int i = 0; i < 4; i++) {
                if (new_pin[i] < '0' || new_pin[i] > '9') { valid = false; break; }
            }
            if (!valid) {
                send_nak(BLE_MSG_CMD, BLE_ERR_INVALID_RANGE);
                return 0;
            }
            settings_change_pin(new_pin);
            ESP_LOGI(TAG, "PIN changed via BLE");
            send_ack(BLE_MSG_CMD);
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown CMD sub-command: 0x%02X", sub_cmd);
            send_nak(BLE_MSG_CMD, BLE_ERR_UNKNOWN_CMD);
            break;
    }

    return 0;
}

// ============================================================================
// GATT Service Definition
// ============================================================================

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // PARAMS: read current params, write new params
                .uuid = &chr_params_uuid.u,
                .access_cb = params_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                // STATUS: periodic binary status notifications
                .uuid = &chr_status_uuid.u,
                .access_cb = status_access_cb,
                .val_handle = &s_status_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // CMD: commands (load preset, factory reset, etc.)
                .uuid = &chr_cmd_uuid.u,
                .access_cb = cmd_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }, // Terminator
        },
    },
    { 0 }, // Terminator
};

// ============================================================================
// GAP Event Handler
// ============================================================================

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_connected = true;
                s_conn_handle = event->connect.conn_handle;

                // Set preferred MTU — the Android app initiates the exchange
                ble_att_set_preferred_mtu(247);

                // Welcome chime — soft 2-note ascending tone (like JK BMS)
                audio_play_ble_connect();

                ESP_LOGI(TAG, "BLE client connected (handle=%d)", s_conn_handle);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            s_connected = false;
            s_authenticated = false;  // Force re-auth on next connection
            ESP_LOGI(TAG, "BLE client disconnected, re-advertising");
            // Re-start advertising
            {
                struct ble_gap_adv_params adv_params = {0};
                adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
                adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
                ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                  &adv_params, gap_event_cb, NULL);
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU negotiated: %d", event->mtu.value);
            break;

        case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
            // Android 8+ requests 2M PHY immediately after connecting.
            // Log and ignore — the connection is still valid even if PHY
            // upgrade fails (BLE_ERR_UNK_CONN_ID is benign here).
            if (event->phy_updated.status != 0) {
                ESP_LOGW(TAG, "PHY update failed (status=%d) — connection maintained on 1M PHY",
                         event->phy_updated.status);
            } else {
                ESP_LOGI(TAG, "PHY updated: TX=%d RX=%d",
                         event->phy_updated.tx_phy, event->phy_updated.rx_phy);
            }
            break;

        default:
            break;
    }
    return 0;
}

// ============================================================================
// BLE Host Task
// ============================================================================

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_on_sync(void)
{
    // Set device name
    ble_svc_gap_device_name_set(g_settings.ble_name);

    // Configure advertising data WITH service UUID for filtering
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)g_settings.ble_name;
    fields.name_len = strlen(g_settings.ble_name);
    fields.name_is_complete = 1;

    // Advertise service UUID so the app can filter by it
    fields.uuids128 = (ble_uuid128_t[]) { svc_uuid };
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    // Start advertising
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_cb, NULL);

    ESP_LOGI(TAG, "BLE advertising as '%s' (Binary Protocol V2)", g_settings.ble_name);
}

// ============================================================================
// Public API
// ============================================================================

void ble_serial_init(void)
{
    // Initialize NimBLE
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(ret));
        return;
    }

    // Register GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    // Set sync callback
    ble_hs_cfg.sync_cb = ble_on_sync;

    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE serial initialized — Binary Protocol V2");
}

bool ble_serial_is_connected(void)
{
    return s_connected;
}

void ble_serial_send_status(void)
{
    if (!s_connected || s_status_attr_handle == 0) return;

    // Build binary STATUS packet
    ble_status_packet_t status;
    build_status_payload(&status);

    uint8_t pkt[BLE_PROTO_HEADER_SIZE + sizeof(status) + BLE_PROTO_CRC_SIZE];
    int pkt_len = ble_proto_build_packet(pkt, BLE_MSG_STATUS,
                                          (const uint8_t *)&status, sizeof(status));

    struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, pkt_len);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
    }
}

void ble_serial_notify(const char *message)
{
    // Legacy text notification — still works for debug logging
    if (!s_connected || !message || s_status_attr_handle == 0) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(message, strlen(message));
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
    }
}

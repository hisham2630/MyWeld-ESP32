#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

/**
 * MyWeld BLE Binary Protocol V2 — Shared Header
 *
 * This file defines the binary protocol used between the ESP32 firmware
 * and the Android companion app. Both sides must use the same definitions.
 *
 * Packet format: [SYNC=0xAA] [TYPE] [LEN] [PAYLOAD...] [CRC]
 * CRC = XOR of TYPE + LEN + all PAYLOAD bytes.
 * All multi-byte values are little-endian (ESP32 native).
 */

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Protocol Constants
// ============================================================================

#define BLE_PROTO_SYNC          0xAA    // Packet start marker
#define BLE_PROTO_HEADER_SIZE   3       // SYNC + TYPE + LEN
#define BLE_PROTO_CRC_SIZE      1       // CRC byte
#define BLE_PROTO_MAX_PAYLOAD   420     // Max payload; preset list = 20×20 = 400 bytes

// ============================================================================
// Message Types
// ============================================================================

#define BLE_MSG_STATUS              0x01    // ESP→App: periodic status (notify)
#define BLE_MSG_PARAMS_READ         0x02    // App→ESP: request params (read)
#define BLE_MSG_PARAMS_RESPONSE     0x03    // ESP→App: current params
#define BLE_MSG_PARAMS_WRITE        0x04    // App→ESP: set params (write)
#define BLE_MSG_CMD                 0x05    // App→ESP: commands (write)
#define BLE_MSG_ACK                 0x06    // ESP→App: success response
#define BLE_MSG_NAK                 0x07    // ESP→App: error response
#define BLE_MSG_VERSION             0x08    // App→ESP: request version
#define BLE_MSG_VERSION_RESPONSE    0x09    // ESP→App: version info
#define BLE_MSG_PRESET_LIST         0x0A    // App→ESP: request preset names
#define BLE_MSG_PRESET_LIST_RESP    0x0B    // ESP→App: preset names
#define BLE_MSG_WELD_LOG_ENTRY      0x0C    // ESP→App: weld log event
#define BLE_MSG_AUTH_REQUEST        0x0D    // App→ESP: request authentication
#define BLE_MSG_AUTH_RESPONSE       0x0E    // ESP→App: authentication result

// ============================================================================
// Command Sub-Types (payload byte 0 of BLE_MSG_CMD)
// ============================================================================

#define BLE_CMD_LOAD_PRESET         0x01    // [preset_index]
#define BLE_CMD_SAVE_PRESET         0x02    // [index, name[20], p1, t, p2, s, mode]
#define BLE_CMD_FACTORY_RESET       0x03    // (no extra data)
#define BLE_CMD_RESET_WELD_COUNTER  0x04    // [target: 0=session, 1=total, 2=both]
#define BLE_CMD_CALIBRATE_ADC       0x05    // [channel, reference_mv_u16]
#define BLE_CMD_AUTH                0x06    // [pin: ASCII 4 digits + null = 5 bytes]
#define BLE_CMD_CHANGE_PIN          0x07    // [new_pin: ASCII 4 digits + null = 5 bytes]

// ============================================================================
// Error Codes (NAK payload byte 1)
// ============================================================================

#define BLE_ERR_INVALID_RANGE       0x01    // Parameter value out of range
#define BLE_ERR_INVALID_PRESET      0x02    // Preset index out of range
#define BLE_ERR_NVS_FAILURE         0x03    // NVS write failed
#define BLE_ERR_BUSY                0x04    // Welding in progress
#define BLE_ERR_UNKNOWN_CMD         0x05    // Unknown command type
#define BLE_ERR_AUTH_FAILED         0x06    // Wrong PIN
#define BLE_ERR_AUTH_REQUIRED       0x07    // Command rejected — not authenticated

// ============================================================================
// Packet Structures (all packed, little-endian)
// ============================================================================

/**
 * STATUS payload (BLE_MSG_STATUS) — 32 bytes, sent every ~500ms via notify.
 */
typedef struct __attribute__((packed)) {
    uint16_t supercap_mv;       // Supercap voltage in millivolts (0–5700)
    uint16_t protection_mv;     // Protection rail in millivolts (0–18000)
    uint8_t  state;             // Weld state enum (0=IDLE, 1=READY, ... 8=CONTACT)
    uint8_t  charge_percent;    // 0–100%
    uint8_t  auto_mode;         // 0=MAN, 1=AUTO
    uint8_t  active_preset;     // 0–9
    uint32_t session_welds;     // Session weld counter
    uint32_t total_welds;       // Lifetime weld counter
    uint8_t  ble_connected;     // 1 if BLE connected
    uint8_t  sound_on;          // 0=mute, 1=on
    uint8_t  theme;             // 0=dark, 1=light
    uint8_t  error_code;        // 0=none, see BLE_ERR_*
    uint16_t p1_x10;            // P1 in 0.1ms units (e.g., 50 = 5.0ms)
    uint16_t t_x10;             // T in 0.1ms units
    uint16_t p2_x10;            // P2 in 0.1ms units
    uint16_t s_x10;             // S in 0.1s units (e.g., 5 = 0.5s)
    uint8_t  fw_major;          // Firmware major version
    uint8_t  fw_minor;          // Firmware minor version
    uint8_t  volume;            // Master volume 0–100%
    uint8_t  reserved;          // Future use, padding to 32 bytes
} ble_status_packet_t;          // Total: 32 bytes

_Static_assert(sizeof(ble_status_packet_t) == 32, "STATUS packet must be 32 bytes");

/**
 * PARAMS_WRITE payload (BLE_MSG_PARAMS_WRITE) — 29 bytes.
 */
typedef struct __attribute__((packed)) {
    uint16_t p1_x10;            // P1 in 0.1ms units
    uint16_t t_x10;             // T in 0.1ms units
    uint16_t p2_x10;            // P2 in 0.1ms units
    uint16_t s_x10;             // S in 0.1s units
    uint8_t  auto_mode;         // 0=MAN, 1=AUTO
    uint8_t  sound_on;          // 0=mute, 1=on
    uint8_t  brightness;        // 0–100%
    uint8_t  volume;            // Master volume 0–100%
    uint8_t  theme;             // 0=dark, 1=light
    char     ble_name[16];      // Device BLE name (null-terminated, padded)
} ble_params_packet_t;          // Total: 29 bytes

_Static_assert(sizeof(ble_params_packet_t) == 29, "PARAMS packet must be 29 bytes");

/**
 * VERSION_RESPONSE payload (BLE_MSG_VERSION_RESPONSE) — 12 bytes.
 */
typedef struct __attribute__((packed)) {
    uint8_t  major;
    uint8_t  minor;
    uint8_t  patch;
    uint8_t  reserved;
    char     build_date[8];     // "YYMMDD\0\0" format
} ble_version_packet_t;         // Total: 12 bytes

/**
 * PRESET_LIST_RESP payload (BLE_MSG_PRESET_LIST_RESP) — paginated, 201 bytes per page.
 * Sent in response to BLE_MSG_PRESET_LIST request (firmware sends 2 notifications).
 *
 * Payload layout per packet:
 *   [0]      page_index  (0 = slots 0–9, 1 = slots 10–19)
 *   [1–200]  names: 10 × 20 bytes, each null-terminated and zero-padded
 *
 * Total: 201 bytes payload per page, 2 pages → 20 presets.
 * 201 bytes fits in the 1-byte LEN field (max 255) and one BLE MTU frame.
 */
#define BLE_PRESET_NAME_LEN     20
#define BLE_MAX_PRESETS         20
#define BLE_PRESETS_PER_PAGE    10   // Presets per PRESET_LIST_RESP packet

typedef struct __attribute__((packed)) {
    uint8_t page_index;                                    // 0 or 1
    char    names[BLE_PRESETS_PER_PAGE][BLE_PRESET_NAME_LEN]; // 10 × 20 = 200 bytes
} ble_preset_list_resp_t;       // Total payload: 201 bytes

// ============================================================================
// CRC Calculation
// ============================================================================

/**
 * Calculate XOR checksum for a packet.
 * @param type   Message type byte
 * @param len    Payload length byte
 * @param payload Payload data
 * @return CRC byte
 */
static inline uint8_t ble_proto_calc_crc(uint8_t type, uint8_t len, const uint8_t *payload)
{
    uint8_t crc = type ^ len;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= payload[i];
    }
    return crc;
}

/**
 * Build a complete packet into the output buffer.
 * @param out    Output buffer (must be at least HEADER + len + CRC bytes)
 * @param type   Message type
 * @param payload Payload data (can be NULL if len == 0)
 * @param len    Payload length
 * @return Total packet size
 */
static inline int ble_proto_build_packet(uint8_t *out, uint8_t type,
                                          const uint8_t *payload, uint8_t len)
{
    out[0] = BLE_PROTO_SYNC;
    out[1] = type;
    out[2] = len;
    if (payload && len > 0) {
        for (uint8_t i = 0; i < len; i++) {
            out[3 + i] = payload[i];
        }
    }
    out[3 + len] = ble_proto_calc_crc(type, len, payload ? payload : out + 3);
    return BLE_PROTO_HEADER_SIZE + len + BLE_PROTO_CRC_SIZE;
}

#endif // BLE_PROTOCOL_H

#ifndef BLE_SERIAL_H
#define BLE_SERIAL_H

#include "config.h"

/**
 * BLE Serial Module — Binary Protocol V2
 *
 * Communicates with the MyWeld Android companion app using a structured
 * binary packet protocol (see ble_protocol.h).
 *
 * GATT Service (UUID 0x1234) with 3 characteristics:
 *   - PARAMS (0x1235): READ → binary params response, WRITE → binary params
 *   - STATUS (0x1236): READ/NOTIFY → binary status packet every ~500ms
 *   - CMD    (0x1237): WRITE → binary commands (load preset, reset, etc.)
 *
 * Safety: BLE cannot trigger weld — physical START button required.
 */

/**
 * Initialize BLE stack, GATT services, and start advertising.
 */
void ble_serial_init(void);

/**
 * Check if a BLE client is connected.
 * @return true if connected
 */
bool ble_serial_is_connected(void);

/**
 * Send a binary status update to the connected BLE client.
 * Builds a STATUS packet (0x01) from current g_settings and g_weld_status
 * and sends it as a GATT notification.
 *
 * Called periodically (~500ms) from the ADC task.
 */
void ble_serial_send_status(void);

/**
 * Send a notification string to the connected client (legacy).
 * @param message Null-terminated string
 * @deprecated Use binary protocol instead. Kept for debug logging.
 */
void ble_serial_notify(const char *message);

#endif // BLE_SERIAL_H

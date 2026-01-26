/*
 * bt_keyboard.h - BLE HID Keyboard Driver
 */

#ifndef BT_KEYBOARD_H
#define BT_KEYBOARD_H

#include "esp_err.h"

/**
 * Initialize the BLE keyboard subsystem
 */
esp_err_t bt_keyboard_init(void);

/**
 * Start scanning for BLE HID keyboards
 */
esp_err_t bt_keyboard_scan(void);

/**
 * Start scanning with optional verbose output
 * @param verbose If non-zero, print all discovered devices
 */
esp_err_t bt_keyboard_scan_ex(int verbose);

/**
 * Check if a keyboard is connected
 * @return 1 if connected, 0 otherwise
 */
int bt_keyboard_connected(void);

/**
 * Reconnect to the last discovered keyboard
 */
esp_err_t bt_keyboard_connect_native(void);

/**
 * Clear all BLE bonds (for troubleshooting pairing issues)
 */
esp_err_t bt_keyboard_clear_bonds(void);

#endif // BT_KEYBOARD_H
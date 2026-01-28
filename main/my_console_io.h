#pragma once

#include "esp_err.h"

/**
 * @brief Initialize custom console I/O
 * 
 * Registers a VFS device at /dev/breezy and redirects stdin/stdout to it.
 * - Output goes to: VTerm buffer -> LCD (textmode) + USB Serial (debug)
 * - Input comes from: BT keyboard OR USB Serial, routed through VTerm
 * 
 * Hotkeys for VT switching:
 *   Alt+1, F1 -> VT0
 *   Alt+2, F2 -> VT1
 * 
 * Call this BEFORE breezybox_start_stdio()
 */
esp_err_t my_console_init(void);

/**
 * @brief Feed a character from BT keyboard
 * 
 * Call this from your BT HID callback when a key is received.
 * Routes through VTerm input handling (hotkey detection).
 * 
 * @param c ASCII character
 */
void my_console_bt_receive(char c);

/**
 * @brief Check if BT keyboard is providing input
 * @return 1 if active, 0 if not
 */
int my_console_bt_active(void);

/**
 * @brief Set which VTerm this console writes to
 *
 * Used when spawning tasks that should output to a different VT.
 * Default is VT0.
 *
 * @param vt_id VTerm ID (0 to VTERM_COUNT-1)
 */
void my_console_set_vt(int vt_id);

// Console output routing modes
typedef enum {
    CONSOLE_OUT_BOTH = 0,   // Output to both LCD and USB (default)
    CONSOLE_OUT_LCD = 1,    // Output to LCD only
    CONSOLE_OUT_USB = 2,    // Output to USB only
} console_output_mode_t;

/**
 * @brief Set console output routing mode
 *
 * Controls where stdout/stderr output goes:
 * - CONSOLE_OUT_BOTH: Both LCD (via VTerm) and USB Serial (default)
 * - CONSOLE_OUT_LCD: LCD only (useful for benchmarking VTerm performance)
 * - CONSOLE_OUT_USB: USB Serial only
 *
 * @param mode Output routing mode
 */
void my_console_set_output_mode(console_output_mode_t mode);

/**
 * @brief Get current console output routing mode
 * @return Current output mode
 */
console_output_mode_t my_console_get_output_mode(void);

/**
 * @brief Check if USB console is connected
 *
 * Returns the detected USB connection state. USB is auto-detected at startup
 * and monitored during writes. When USB is disconnected and mode is BOTH,
 * USB writes are automatically skipped to avoid performance degradation.
 *
 * @return 1 if USB is connected, 0 if disconnected
 */
int my_console_usb_connected(void);

/**
 * @brief Reset USB connection detection
 *
 * Call this to re-enable USB writes after the cable has been reconnected.
 * The next write will re-probe USB connectivity.
 */
void my_console_usb_reconnect(void);
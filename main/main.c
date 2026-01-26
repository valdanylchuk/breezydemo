#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_log.h"
#include "esp_console.h"
#include "nvs_flash.h"
#include "host/ble_store.h"

#include "breezybox.h"
#include "my_display.h"
#include "my_console_io.h"
#include "bt_keyboard.h"
#include "vterm.h"

static const char *TAG = "main";

// BreezyBox command to scan for BT
int cmd_btscan(int argc, char **argv) {
    int verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
    return bt_keyboard_scan_ex(verbose) == ESP_OK ? 0 : 1;
}

// Add declaration
esp_err_t bt_keyboard_connect_native(void);

// Add Command Wrapper
int cmd_btconnect(int argc, char **argv) {
    return bt_keyboard_connect_native() == ESP_OK ? 0 : 1;
}

static int cmd_btclear(int argc, char **argv) {
    bt_keyboard_clear_bonds();
    printf("Bonds cleared. Restart device.\n");
    return 0;
}

// BreezyBox command to check BT status
static int cmd_btstatus(int argc, char **argv)
{
    if (bt_keyboard_connected()) {
        printf("BT keyboard: connected\n");
    } else {
        printf("BT keyboard: not connected\n");
        printf("Use 'btscan' to search for keyboards\n");
    }
    return 0;
}

// DEBUG
static int cmd_vt(int argc, char **argv)
{
    if (argc < 2) {
        printf("Active: VT%d\n", vterm_get_active());
        return 0;
    }
    int n = atoi(argv[1]);
    if (n >= 0 && n < VTERM_COUNT) {
        vterm_switch(n);
        printf("Switched to VT%d\n", n);
    }
    return 0;
}

// DEBUG
static int cmd_keytest(int argc, char **argv)
{
    printf("Press keys (Ctrl+C to exit):\n");
    while (1) {
        int c = getchar();
        if (c == 3) break;  // Ctrl+C
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c >= 32 && c < 127) {
            printf("0x%02X '%c'\n", c, c);
        } else {
            printf("0x%02X\n", c);
        }
    }
    return 0;
}

// DEBUG
static int cmd_colortest(int argc, char **argv)
{
    printf("\033[31mRed\033[0m ");
    printf("\033[32mGreen\033[0m ");
    printf("\033[33mYellow\033[0m ");
    printf("\033[34mBlue\033[0m ");
    printf("\033[1;35mBright Magenta\033[0m\n");
    printf("\033[41;37mWhite on Red\033[0m\n");
    return 0;
}

// Set console output routing (lcd, usb, or both)
static int cmd_setcon(int argc, char **argv)
{
    if (argc < 2) {
        console_output_mode_t mode = my_console_get_output_mode();
        const char *mode_str = (mode == CONSOLE_OUT_LCD) ? "lcd" :
                                (mode == CONSOLE_OUT_USB) ? "usb" : "both";
        printf("Console output: %s\n", mode_str);
        printf("Usage: setcon <lcd|usb|both>\n");
        return 0;
    }

    const char *arg = argv[1];
    console_output_mode_t mode;

    if (strcmp(arg, "lcd") == 0) {
        mode = CONSOLE_OUT_LCD;
    } else if (strcmp(arg, "usb") == 0) {
        mode = CONSOLE_OUT_USB;
    } else if (strcmp(arg, "both") == 0) {
        mode = CONSOLE_OUT_BOTH;
    } else {
        printf("Invalid mode: %s\n", arg);
        printf("Usage: setcon <lcd|usb|both>\n");
        return 1;
    }

    my_console_set_output_mode(mode);
    printf("Console output: %s\n", arg);
    return 0;
}

// Main loop - keeps task alive while DMA renders display
static void main_loop(void)
{
    // Display renders via DMA bounce-buffer callbacks (zero-copy from vterm)
    // This loop just keeps the main task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    usb_serial_jtag_driver_config_t usb_config = {
        .tx_buffer_size = 256,
        .rx_buffer_size = 256,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));
    usb_serial_jtag_vfs_use_driver();

    printf("\n--- Boot sequence complete. Starting ESP32-DOS ---\n");

    // 1. Initialize Display Hardware
    printf("Initializing display...\n");
    my_display_init();
    printf("Display initialized\n");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. Initialize VTerm & Console (also links vterm buffer to display)
    if (my_console_init() != ESP_OK) {
        ESP_LOGE(TAG, "Console init failed!");
        return;
    }

    if (bt_keyboard_init() == ESP_OK) {
        // BT initialized successfully
    } else {
        ESP_LOGW(TAG, "BT init failed, USB-only mode");
    }

    breezybox_start_stdio(8192, 5);

    // Register BT commands
    const esp_console_cmd_t bt_cmds[] = {
        { .command = "btscan", .help = "Scan for BT keyboards", .hint = "[-v]", .func = &cmd_btscan },
        { .command = "btconnect", .help = "Connect to found HID", .func = &cmd_btconnect },
        { .command = "btclear", .help = "Clear saved BT devices", .func = &cmd_btclear },
        { .command = "btstatus", .help = "Show BT keyboard status", .func = &cmd_btstatus },
        { .command = "vt", .help = "Switch VT", .func = &cmd_vt },
        { .command = "keytest", .help = "Keys test", .func = &cmd_keytest },
        { .command = "colortest", .help = "ANSI colors test", .func = &cmd_colortest },
        { .command = "setcon", .help = "Set console output", .hint = "<lcd|usb|both>", .func = &cmd_setcon },
    };
    for (int i = 0; i < sizeof(bt_cmds)/sizeof(bt_cmds[0]); i++) {
        esp_console_cmd_register(&bt_cmds[i]);
    }

    // 4. Keep main task alive (display renders via DMA callbacks)
    main_loop();
}
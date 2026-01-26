/*
 * my_console_io.c - Custom VFS console with VTerm integration
 * 
 * Redirects stdin/stdout to virtual terminals, supporting multiple
 * VTs with hotkey switching (F1-F4, Ctrl+F1-F4).
 */

#include "my_console_io.h"
#include "my_display.h"
#include "vterm.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <sys/stat.h>

#define CONSOLE_DEV_PATH "/dev/breezy"

// Track O_NONBLOCK flag
static int s_stdin_flags = 0;
static int s_stdout_flags = 0;

// Console output routing mode
static console_output_mode_t s_output_mode = CONSOLE_OUT_BOTH;

void my_console_bt_receive(char c)
{
    vterm_input_feed(c);
}

int my_console_bt_active(void)
{
    return 0;
}

// ============ VFS Implementation ============

// Check if data contains terminal probe sequences that shouldn't go to USB
// Returns true if the data should NOT be sent to USB serial
// (These are queries/probes that vterm handles internally)
static int is_terminal_probe(const char *data, size_t size)
{
    if (size < 4) return 0;
    
    for (size_t i = 0; i + 3 < size; i++) {
        if (data[i] == '\x1b' && data[i+1] == '[') {
            // ESC[5n - Device status query
            // ESC[6n - Cursor position query
            if ((data[i+2] == '5' || data[i+2] == '6') && data[i+3] == 'n') {
                return 1;
            }
            // ESC[999C - Move cursor to far right (used by getColumns probe)
            if (i + 5 <= size && 
                data[i+2] == '9' && data[i+3] == '9' && data[i+4] == '9' && data[i+5] == 'C') {
                return 1;
            }
        }
    }
    
    // Also check exact 4-byte queries
    if (size == 4 && data[0] == '\x1b' && data[1] == '[' &&
        (data[2] == '5' || data[2] == '6') && data[3] == 'n') {
        return 1;
    }
    
    return 0;
}

static ssize_t my_console_write(int fd, const void *data, size_t size)
{
    const char *str = (const char *)data;

    // Write to LCD (via VTerm) if enabled
    if (s_output_mode == CONSOLE_OUT_BOTH || s_output_mode == CONSOLE_OUT_LCD) {
        int active = vterm_get_active();
        vterm_write(active, str, size);
    }

    // Write to USB Serial if enabled
    if (s_output_mode == CONSOLE_OUT_BOTH || s_output_mode == CONSOLE_OUT_USB) {
        // Skip device status queries to avoid duplicate responses from remote terminal
        if (!is_terminal_probe(str, size)) {
            usb_serial_jtag_write_bytes(data, size, pdMS_TO_TICKS(10));
        }
    }

    return size;
}

static ssize_t my_console_read(int fd, void *data, size_t size)
{
    char *buf = (char *)data;
    size_t count = 0;
    int nonblock = (s_stdin_flags & O_NONBLOCK);
    
    while (count < size) {
        // Drain USB bytes into vterm input queue
        char c;
        while (usb_serial_jtag_read_bytes(&c, 1, 0) > 0) {
            // Convert CR to LF - terminals send \r, linenoise expects \n
            if (c == '\r') c = '\n';
            vterm_input_feed(c);
        }
        
        int active = vterm_get_active();
        // First char: wait with timeout. Subsequent chars: no wait (fill buffer if available)
        int timeout = (count == 0) ? (nonblock ? 0 : 50) : 0;
        int ch = vterm_getchar(active, timeout);
        
        if (ch >= 0) {
            // Convert CR to LF on output too (in case BT keyboard sends \r)
            if (ch == '\r') ch = '\n';
            
            buf[count++] = (char)ch;
            
            // Only break early on newline (end of line input)
            // Otherwise keep reading to fill buffer (needed for escape sequences)
            if (ch == '\n') {
                break;
            }
        } else {
            // No data available
            if (count > 0) {
                // Already have some data, return it
                break;
            }
            if (nonblock) {
                errno = EAGAIN;
                return -1;
            }
            // Blocking mode with no data yet - small delay and retry
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    
    return count;
}

static int my_console_open(const char *path, int flags, int mode)
{
    return 0;
}

static int my_console_close(int fd)
{
    return 0;
}

static int my_console_fstat(int fd, struct stat *st)
{
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;  // Character device - makes isatty() return true
    return 0;
}

static int my_console_fsync(int fd)
{
    // No buffering, nothing to sync
    return 0;
}

#if CONFIG_VFS_SUPPORT_TERMIOS
static int my_console_tcsetattr(int fd, int optional_actions, const struct termios *p)
{
    // Accept any termios settings - we're already in raw-like mode
    return 0;
}

static int my_console_tcgetattr(int fd, struct termios *p)
{
    memset(p, 0, sizeof(*p));
    // Report as a basic terminal
    p->c_cflag = CS8;      // 8-bit chars
    p->c_cc[VMIN] = 1;     // Read returns after 1 char
    p->c_cc[VTIME] = 0;    // No timeout
    return 0;
}
#endif

static int my_console_fcntl(int fd, int cmd, int arg)
{
    // Determine which fd we're dealing with (stdin=0, stdout=1)
    int *flags = (fd == STDIN_FILENO) ? &s_stdin_flags : &s_stdout_flags;
    
    switch (cmd) {
    case F_GETFL:
        return *flags;
    case F_SETFL:
        *flags = arg;
        return 0;
    default:
        return 0;
    }
}

// Callback when VT is switched
static void on_vt_switch(int new_vt)
{
    char msg[32];
    snprintf(msg, sizeof(msg), "\r\n[Switched to VT%d]\r\n", new_vt);
    usb_serial_jtag_write_bytes(msg, strlen(msg), pdMS_TO_TICKS(10));
}

// Custom log handler - always writes to USB, bypassing console VFS
static int usb_log_vprintf(const char *fmt, va_list args)
{
    char buf[256];
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    if (len > 0) {
        usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(10));
    }
    return len;
}

esp_err_t my_console_init(void)
{
    // Initialize vterm system
    esp_err_t ret = vterm_init();
    if (ret != ESP_OK) return ret;

    // Link vterm buffer directly to display (zero-copy)
    vterm_cell_t *buf = vterm_get_direct_buffer();
    if (buf) {
        my_display_set_buffer(buf);
    }

    vterm_set_switch_callback(on_vt_switch);
    
    // Register VFS
    esp_vfs_t vfs = {
        .flags = ESP_VFS_FLAG_DEFAULT,
        .write = my_console_write,
        .read = my_console_read,
        .open = my_console_open,
        .close = my_console_close,
        .fstat = my_console_fstat,
        .fcntl = my_console_fcntl,
        .fsync = my_console_fsync,
#if CONFIG_VFS_SUPPORT_TERMIOS
        .tcsetattr = my_console_tcsetattr,
        .tcgetattr = my_console_tcgetattr,
#endif
    };
    
    ret = esp_vfs_register(CONSOLE_DEV_PATH, &vfs, NULL);
    if (ret != ESP_OK) return ret;
    
    // Redirect stdin/stdout
    if (!freopen(CONSOLE_DEV_PATH, "r", stdin)) return ESP_FAIL;
    if (!freopen(CONSOLE_DEV_PATH, "w", stdout)) return ESP_FAIL;
    
    // Disable buffering
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    // Redirect ESP_LOG to always use USB (bypasses stdout redirection)
    esp_log_set_vprintf(usb_log_vprintf);

    return ESP_OK;
}

void my_console_set_output_mode(console_output_mode_t mode)
{
    s_output_mode = mode;
}

console_output_mode_t my_console_get_output_mode(void)
{
    return s_output_mode;
}
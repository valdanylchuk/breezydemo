/*
* my_display.c - Tuned Text Renderer (Zero-Copy)
*
* Reads directly from interleaved vterm_cell_t buffer (IRAM).
* Optimized for 32-bit reads with 2-byte aligned cells.
* Uses vterm's configurable 16-color palette.
*/

#include "my_display.h"
#include "vterm.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include <string.h>

static const char *TAG = "display";

#define SCREEN_WIDTH    1024
#define SCREEN_HEIGHT   600
#define BOUNCE_HEIGHT_PX 10
#define FONT_WIDTH      8
#define FONT_HEIGHT     16
#define TEXT_COLS       128
#define TEXT_ROWS       37

// Pointer to external buffer (managed by vterm)
static vterm_cell_t *s_display_buffer = NULL;

static esp_lcd_panel_handle_t panel_handle = NULL;

// LUTs
static uint8_t font_ram[256][16];
static uint32_t BYTE_MASKS[256][4];
static const uint32_t MASK_LUT[4] = { 0x00000000, 0xFFFF0000, 0x0000FFFF, 0xFFFFFFFF };

// ATTR_LUT: precomputed bg32 and xor32 for each attribute byte
// ATTR_LUT[attr][0] = bg32, ATTR_LUT[attr][1] = xor32
static uint32_t ATTR_LUT[256][2];

// External font data
extern const uint8_t terminus16_glyph_bitmap[];

static void rebuild_attr_lut(void)
{
    const uint16_t *palette = vterm_get_palette();

    for (int attr = 0; attr < 256; attr++) {
        uint8_t fg_idx = VTERM_ATTR_FG(attr);
        uint8_t bg_idx = VTERM_ATTR_BG(attr);

        uint16_t fg_color = palette[fg_idx];
        uint16_t bg_color = palette[bg_idx];

        uint32_t bg32 = (bg_color << 16) | bg_color;
        uint32_t fg32 = (fg_color << 16) | fg_color;

        ATTR_LUT[attr][0] = bg32;
        ATTR_LUT[attr][1] = fg32 ^ bg32;  // xor32
    }
}

static void precompute_tables(void)
{
    // Build ATTR_LUT from vterm palette
    rebuild_attr_lut();

    // Pre-compute glyph byte to pixel masks
    for (int i = 0; i < 256; i++) {
        BYTE_MASKS[i][0] = MASK_LUT[(i >> 6) & 0x03];
        BYTE_MASKS[i][1] = MASK_LUT[(i >> 4) & 0x03];
        BYTE_MASKS[i][2] = MASK_LUT[(i >> 2) & 0x03];
        BYTE_MASKS[i][3] = MASK_LUT[i & 0x03];
    }
}

static IRAM_ATTR bool on_bounce_empty(esp_lcd_panel_handle_t panel, void *buf,
                                    int pos_px, int len_bytes, void *user_ctx)
{
    // Clear to black - also serves as fallback if vterm isn't ready yet
    memset(buf, 0, len_bytes);

    if (!s_display_buffer) return false;

    int y_start = pos_px / SCREEN_WIDTH;
    int num_lines = (len_bytes / 2) / SCREEN_WIDTH;
    const vterm_cell_t *src_buf = s_display_buffer;

    for (int line = 0; line < num_lines; line++) {
        int y = y_start + line;
        int text_row = y / FONT_HEIGHT;
        if (text_row >= TEXT_ROWS) continue;

        int glyph_y = y % FONT_HEIGHT;
        uint32_t *dest = (uint32_t *)((uint8_t *)buf + (line * SCREEN_WIDTH * 2));

        // Get pointer to the start of the row in the cell buffer
        const vterm_cell_t *cell_row_ptr = &src_buf[text_row * TEXT_COLS];

        // Process 2 cells at a time using 32-bit aligned reads
        // With 2-byte cells, reading 4 bytes gives us 2 cells
        const uint32_t *cell_pairs = (const uint32_t *)cell_row_ptr;

        for (int pair = 0; pair < TEXT_COLS / 2; pair++) {
            uint32_t cell_data = cell_pairs[pair];

            // Extract cell 0 (low 16 bits): ch in bits 0-7, attr in bits 8-15
            uint8_t ch0 = cell_data & 0xFF;
            uint8_t attr0 = (cell_data >> 8) & 0xFF;

            // Extract cell 1 (high 16 bits): ch in bits 16-23, attr in bits 24-31
            uint8_t ch1 = (cell_data >> 16) & 0xFF;
            uint8_t attr1 = (cell_data >> 24) & 0xFF;

            // --- Render cell 0 ---
            uint32_t bg32_0 = ATTR_LUT[attr0][0];
            uint32_t xor32_0 = ATTR_LUT[attr0][1];
            uint8_t glyph0 = font_ram[ch0][glyph_y];

            if (glyph0 == 0) {
                *dest++ = bg32_0; *dest++ = bg32_0; *dest++ = bg32_0; *dest++ = bg32_0;
            } else {
                const uint32_t *m = BYTE_MASKS[glyph0];
                *dest++ = (xor32_0 & m[0]) ^ bg32_0;
                *dest++ = (xor32_0 & m[1]) ^ bg32_0;
                *dest++ = (xor32_0 & m[2]) ^ bg32_0;
                *dest++ = (xor32_0 & m[3]) ^ bg32_0;
            }

            // --- Render cell 1 ---
            uint32_t bg32_1 = ATTR_LUT[attr1][0];
            uint32_t xor32_1 = ATTR_LUT[attr1][1];
            uint8_t glyph1 = font_ram[ch1][glyph_y];

            if (glyph1 == 0) {
                *dest++ = bg32_1; *dest++ = bg32_1; *dest++ = bg32_1; *dest++ = bg32_1;
            } else {
                const uint32_t *m = BYTE_MASKS[glyph1];
                *dest++ = (xor32_1 & m[0]) ^ bg32_1;
                *dest++ = (xor32_1 & m[1]) ^ bg32_1;
                *dest++ = (xor32_1 & m[2]) ^ bg32_1;
                *dest++ = (xor32_1 & m[3]) ^ bg32_1;
            }
        }
    }
    return false;
}

void my_display_init(void)
{
    ESP_LOGI(TAG, "Initializing RGB LCD (Bounce Buffer Text Mode - Zero Copy)");

    volatile const void *exports[] = { // for ELF binaries
        (void *)my_display_refresh_palette,
    };
    (void)exports; // suppress unused warning
    
    precompute_tables();

    // Load font to RAM
    memset(font_ram, 0, sizeof(font_ram));
    for (int i = 0x20; i < 0x100; i++)
        memcpy(font_ram[i], &terminus16_glyph_bitmap[(i - 0x20) * 16], 16);

    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .timings = {
            .pclk_hz = 20 * 1000 * 1000,
            .h_res = SCREEN_WIDTH,
            .v_res = SCREEN_HEIGHT,
            .hsync_pulse_width = 162,
            .hsync_back_porch = 152,
            .hsync_front_porch = 48,
            .vsync_pulse_width = 45,
            .vsync_back_porch = 13,
            .vsync_front_porch = 3,
            .flags.pclk_active_neg = 1,
        },
        .data_width = 16,
        .bits_per_pixel = 16,
        .num_fbs = 0,
        .flags.no_fb = 1,
        .bounce_buffer_size_px = SCREEN_WIDTH * BOUNCE_HEIGHT_PX,
        .hsync_gpio_num = 46,
        .vsync_gpio_num = 3,
        .de_gpio_num = 5,
        .pclk_gpio_num = 7,
        .disp_gpio_num = -1,
        .data_gpio_nums = {14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40},
    };

    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_bounce_empty = on_bounce_empty
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, NULL));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_LOGI(TAG, "Display ready: %dx%d pixels, %dx%d chars",
            SCREEN_WIDTH, SCREEN_HEIGHT, TEXT_COLS, TEXT_ROWS);
}

void my_display_set_buffer(vterm_cell_t *cells)
{
    s_display_buffer = cells;
}

// Rebuild ATTR_LUT when palette changes (call after vterm_set_palette*)
void my_display_refresh_palette(void)
{
    rebuild_attr_lut();
}

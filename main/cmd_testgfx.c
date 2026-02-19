/*
* testgfx.c - VGA 320x200 graphics mode demo
*/

#include "rgb_display.h"
#include "rgb_gfx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

// Build a smooth 6-segment rainbow across palette entries 1-255.
// Entry 0 stays black (background).
static void init_rainbow_palette(uint16_t pal[256])
{
    pal[0] = 0;
    for (int i = 1; i < 256; i++) {
        int h = (i - 1) * 1530 / 255; // 0..1524, six 255-step segments
        int seg = h / 255, t = h % 255;
        uint8_t r, g, b;
        switch (seg) {
            case 0: r=255;     g=t;       b=0;       break; // red → yellow
            case 1: r=255-t;   g=255;     b=0;       break; // yellow → green
            case 2: r=0;       g=255;     b=t;       break; // green → cyan
            case 3: r=0;       g=255-t;   b=255;     break; // cyan → blue
            case 4: r=t;       g=0;       b=255;     break; // blue → magenta
            default:r=255;     g=0;       b=255-t;   break; // magenta → red
        }
        pal[i] = ((uint16_t)(r >> 3) << 11) | ((uint16_t)(g >> 2) << 5) | (b >> 3);
    }
}

#define W 320
#define H 200
#define RECT_PER_FRAME 10

int cmd_testgfx(int argc, char **argv)
{
    printf("Entering VGA 320x200 graphics mode...\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    if (rgb_display_set_mode(SM_VGA13H) != 0) {
        printf("Failed to enter graphics mode!\n");
        return 1;
    }

    int x = 10, y = 10, vx = 1, vy = 1;
    int bw = 40, bh = 30;
    uint8_t color = 1;
    int frame_count = 0;
    int max_frames = 300;
    int use_vsync = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            max_frames = atoi(argv[++i]) * 30;
            if (max_frames <= 0) max_frames = 300;
        } else if (strcmp(argv[i], "-v") == 0) {
            use_vsync = 1;
        }
    }

    uint16_t pal[256];
    init_rainbow_palette(pal);
    rgb_display_set_vga_palette(pal);

    rgb_gfx_clear(0);
    int64_t start_time = esp_timer_get_time();

    while (frame_count < max_frames) {
        if (use_vsync)
            rgb_display_wait_vsync();

        for (int i = 0; i < RECT_PER_FRAME; i++) {
            rgb_gfx_rect(x, y, bw, bh, color++);
            if (color == 0) color = 1;
            x += vx; y += vy;
            if (x <= 0 || x + bw >= W) vx = -vx;
            if (y <= 0 || y + bh >= H) vy = -vy;
        }

        // Palette rotation: shift entries 1-255 left by one each frame.
        // The framebuffer is untouched; only the color mapping changes.
        uint16_t tmp = pal[1];
        memmove(&pal[1], &pal[2], 254 * sizeof(uint16_t));
        pal[255] = tmp;
        rgb_display_set_vga_palette(pal);

        vTaskDelay(pdMS_TO_TICKS(20));
        frame_count++;
    }

    int64_t elapsed = esp_timer_get_time() - start_time;
    double fps = (double)frame_count / (elapsed / 1000000.0);

    rgb_display_set_mode(SM_TEXT);
    printf("Demo complete. %d frames, %.1f FPS\n", frame_count, fps);
    return 0;
}
#pragma once
#include <stdint.h>
#include "vterm.h"

#define DISPLAY_COLS 128
#define DISPLAY_ROWS 37

void my_display_init(void);
void my_display_set_buffer(vterm_cell_t *cells);

// Palette support - call after vterm_set_palette*() to update display LUT
void my_display_refresh_palette(void);

// Cursor support - set position for blinking underscore cursor
// Pass col=-1 or row=-1 to hide cursor
void my_display_set_cursor(int col, int row);

#pragma once
#include <stdint.h>
#include "vterm.h"

#define DISPLAY_COLS 128
#define DISPLAY_ROWS 37

void my_display_init(void);
void my_display_set_buffer(vterm_cell_t *cells);

// Palette support - call after vterm_set_palette*() to update display LUT
void my_display_refresh_palette(void);

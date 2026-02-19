# Register the 'main' component with the build system.
idf_component_register(
    # --- Source Files ---
    SRCS
        "main.c"
        "my_console_io.c"
        "cmd_testgfx.c"

    # --- Dependencies ---
    PRIV_REQUIRES
        spi_flash
        driver
        fatfs
        esp_lcd
        breezybox
        breezy_bt
        breezy_rgb_lcd
        heap
        esp_hid
        bt
        nvs_flash
        hal

    # --- Include Directories ---
    INCLUDE_DIRS
        "."
)

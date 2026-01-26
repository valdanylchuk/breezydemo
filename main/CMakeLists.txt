# Register the 'main' component with the build system.
idf_component_register(
    # --- Source Files ---
    SRCS
        "main.c"
        "my_display.c"
        "terminus16.c"
        "my_console_io.c"
        "bt_keyboard.c"

    # --- Dependencies ---
    PRIV_REQUIRES
        spi_flash
        driver
        fatfs
        esp_lcd
        breezybox
        heap
        esp_hid
        bt
        nvs_flash
        hal

    # --- Include Directories ---
    INCLUDE_DIRS
        "."
)

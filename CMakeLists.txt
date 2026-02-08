cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(breezybox_example)

# Copy our custom symbol table into the elf_loader managed component.
# Runs every build; copy_if_different avoids needless recompilation.
add_custom_target(copy_all_symbols ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${PROJECT_DIR}/main/all_my_symbols.c
        ${PROJECT_DIR}/managed_components/espressif__elf_loader/src/esp_all_symbol.c
    COMMENT "Copying all_my_symbols.c -> esp_all_symbol.c"
)
# Ensure the copy finishes before the elf_loader component compiles.
if(TARGET __idf_espressif__elf_loader)
    add_dependencies(__idf_espressif__elf_loader copy_all_symbols)
endif()

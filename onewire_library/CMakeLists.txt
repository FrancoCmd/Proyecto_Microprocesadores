add_library(onewire_library STATIC onewire_library.c)

# Ensamblar el programa PIO
pico_generate_pio_header(onewire_library ${CMAKE_CURRENT_LIST_DIR}/onewire_library.pio)

# Enlazar librerías necesarias para compilar correctamente
target_link_libraries(onewire_library PRIVATE
    pico_stdlib
    hardware_pio
)

# Incluir directorios para que el compilador encuentre los headers
target_include_directories(onewire_library PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

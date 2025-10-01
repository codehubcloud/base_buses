# OneWire protocol module
add_library(onewire STATIC
    onewire.c
    onewire_hal.c
)

target_include_directories(onewire PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)

target_link_libraries(onewire PRIVATE securec)

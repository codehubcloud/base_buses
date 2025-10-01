# AXI (Advanced eXtensible Interface) bus protocol module

# Create static library for AXI module
add_library(axi STATIC
    axi.c
    axi_hal.c
)

# Set include directories for this module
target_include_directories(axi PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against securec for safe string operations
target_link_libraries(axi PRIVATE
    securec
)

# Add platform-specific compile definitions
if(PLATFORM STREQUAL "STM32F4")
    target_compile_definitions(axi PRIVATE PLATFORM_STM32F4)
elseif(PLATFORM STREQUAL "STM32F1")
    target_compile_definitions(axi PRIVATE PLATFORM_STM32F1)
elseif(PLATFORM STREQUAL "ESP32")
    target_compile_definitions(axi PRIVATE PLATFORM_ESP32)
elseif(PLATFORM STREQUAL "LINUX")
    target_compile_definitions(axi PRIVATE PLATFORM_LINUX)
endif()

# Set C standard
set_property(TARGET axi PROPERTY C_STANDARD 99)

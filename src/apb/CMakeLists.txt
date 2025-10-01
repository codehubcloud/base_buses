# APB Protocol Module CMakeLists.txt
# Advanced Peripheral Bus implementation for embedded systems

# Create static library for APB protocol
add_library(apb STATIC
    apb.c
    apb_hal.c
)

# Include directories
target_include_directories(apb PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)

# Link securec library
target_link_libraries(apb PRIVATE
    securec
)

# Set compile options
target_compile_options(apb PRIVATE
    -Wall
    -Wextra
    -Werror
)

# Platform-specific configurations
if(PLATFORM STREQUAL "STM32F4")
    target_compile_definitions(apb PRIVATE PLATFORM_STM32F4)
elseif(PLATFORM STREQUAL "STM32F1")
    target_compile_definitions(apb PRIVATE PLATFORM_STM32F1)
elseif(PLATFORM STREQUAL "ESP32")
    target_compile_definitions(apb PRIVATE PLATFORM_ESP32)
elseif(PLATFORM STREQUAL "LINUX")
    target_compile_definitions(apb PRIVATE PLATFORM_LINUX)
endif()

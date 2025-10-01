# PCIe module CMake configuration

# Create PCIe static library
add_library(pcie STATIC
    pcie.c
    pcie_hal.c
)

# Include directories
target_include_directories(pcie PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)

# Link securec library
target_link_libraries(pcie PRIVATE
    securec
)

# Platform-specific compiler flags
if(PLATFORM STREQUAL "STM32F4")
    target_compile_definitions(pcie PRIVATE STM32F407xx)
elseif(PLATFORM STREQUAL "STM32F1")
    target_compile_definitions(pcie PRIVATE STM32F103xE)
elseif(PLATFORM STREQUAL "ESP32")
    target_compile_definitions(pcie PRIVATE ESP32)
elseif(PLATFORM STREQUAL "LINUX")
    # Linux is default, no additional definitions needed
endif()

# Compiler warnings
target_compile_options(pcie PRIVATE
    -Wall
    -Wextra
    -Werror
)

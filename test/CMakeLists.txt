# Test executable CMakeLists.txt
cmake_minimum_required(VERSION 3.10)

# Include all module directories for headers
include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/uart
    ${CMAKE_SOURCE_DIR}/src/spi
    ${CMAKE_SOURCE_DIR}/src/i2c
    ${CMAKE_SOURCE_DIR}/src/i3c
    ${CMAKE_SOURCE_DIR}/src/i2s
    ${CMAKE_SOURCE_DIR}/src/can
    ${CMAKE_SOURCE_DIR}/src/canfd
    ${CMAKE_SOURCE_DIR}/src/lin
    ${CMAKE_SOURCE_DIR}/src/flexray
    ${CMAKE_SOURCE_DIR}/src/modbus
    ${CMAKE_SOURCE_DIR}/src/smbus
    ${CMAKE_SOURCE_DIR}/src/pmbus
    ${CMAKE_SOURCE_DIR}/src/rs232
    ${CMAKE_SOURCE_DIR}/src/rs422
    ${CMAKE_SOURCE_DIR}/src/rs485
    ${CMAKE_SOURCE_DIR}/src/usb
    ${CMAKE_SOURCE_DIR}/src/sdio
    ${CMAKE_SOURCE_DIR}/src/onewire
    ${CMAKE_SOURCE_DIR}/src/ethernet
    ${CMAKE_SOURCE_DIR}/src/pcie
    ${CMAKE_SOURCE_DIR}/src/emmc
    ${CMAKE_SOURCE_DIR}/src/jtag
    ${CMAKE_SOURCE_DIR}/src/swd
    ${CMAKE_SOURCE_DIR}/src/axi
    ${CMAKE_SOURCE_DIR}/src/ahb
    ${CMAKE_SOURCE_DIR}/src/apb
    ${CMAKE_SOURCE_DIR}/src/dma
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)

# Create test executable
add_executable(bus_test main.c)

# Link with all bus modules
target_link_libraries(bus_test bus_modules securec)

# Set output directory
set_target_properties(bus_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

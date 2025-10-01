cmake_minimum_required(VERSION 3.10)
project(BusSystem C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Platform selection (default: LINUX)
# Options: STM32F4, STM32F1, ESP32, LINUX
if(NOT DEFINED PLATFORM)
    set(PLATFORM "LINUX")
endif()

message(STATUS "Building for platform: ${PLATFORM}")

# Platform-specific compiler definitions
if(PLATFORM STREQUAL "STM32F4")
    add_definitions(-DPLATFORM_STM32F4)
    add_definitions(-DSTM32F4)
    # Add STM32F4 HAL include paths here if needed
    # include_directories(${STM32_HAL_INCLUDE_DIR})
elseif(PLATFORM STREQUAL "STM32F1")
    add_definitions(-DPLATFORM_STM32F1)
    add_definitions(-DSTM32F1)
    # Add STM32F1 HAL include paths here if needed
    # include_directories(${STM32_HAL_INCLUDE_DIR})
elseif(PLATFORM STREQUAL "ESP32")
    add_definitions(-DPLATFORM_ESP32)
    add_definitions(-DESP32)
    # ESP32 specific settings
elseif(PLATFORM STREQUAL "LINUX")
    add_definitions(-DPLATFORM_LINUX)
else()
    message(FATAL_ERROR "Unsupported platform: ${PLATFORM}")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_C_FLAGS_DEBUG "-g -O0 -Wall -Wextra")
set(CMAKE_C_FLAGS_RELEASE "-O2 -Wall -Wextra")

include_directories(
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/uart
    ${CMAKE_SOURCE_DIR}/src/spi
    ${CMAKE_SOURCE_DIR}/src/i2c
)

add_subdirectory(src)
add_subdirectory(3rd_party/securelib/src)
add_subdirectory(test)

install(TARGETS bus_test RUNTIME DESTINATION bin)
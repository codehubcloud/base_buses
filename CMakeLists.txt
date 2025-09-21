cmake_minimum_required(VERSION 3.10)
project(BusSystem C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

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

# Create executable target
add_executable(bus_app src/main.c)

# Link with bus modules
target_link_libraries(bus_app bus_modules)

install(TARGETS bus_app RUNTIME DESTINATION bin)
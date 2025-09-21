add_subdirectory(uart)
add_subdirectory(spi)
add_subdirectory(i2c)
add_subdirectory(i3c)
add_subdirectory(modbus)
add_subdirectory(smbus)
add_subdirectory(rs232)
add_subdirectory(rs485)
add_subdirectory(can)

set(BUS_MODULES 
    uart 
    spi 
    i2c 
    i3c 
    modbus 
    smbus 
    rs232 
    rs485 
    can
)

# Create an interface library for all bus modules
add_library(bus_modules INTERFACE)

# Link all bus modules to the interface library
target_link_libraries(bus_modules INTERFACE ${BUS_MODULES})

# Make include directories available
target_include_directories(bus_modules INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)
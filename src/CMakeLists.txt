add_subdirectory(uart)
add_subdirectory(spi)
add_subdirectory(i2c)
add_subdirectory(i3c)
add_subdirectory(i2s)
add_subdirectory(modbus)
add_subdirectory(smbus)
add_subdirectory(pmbus)
add_subdirectory(rs232)
add_subdirectory(rs422)
add_subdirectory(rs485)
add_subdirectory(can)
add_subdirectory(canfd)
add_subdirectory(lin)
add_subdirectory(flexray)
add_subdirectory(onewire)
add_subdirectory(usb)
add_subdirectory(sdio)
add_subdirectory(ethernet)
add_subdirectory(swd)
add_subdirectory(jtag)
add_subdirectory(pcie)
add_subdirectory(emmc)
add_subdirectory(ahb)
add_subdirectory(apb)
add_subdirectory(axi)
add_subdirectory(dma)

set(BUS_MODULES
    uart
    spi
    i2c
    i3c
    i2s
    modbus
    smbus
    pmbus
    rs232
    rs422
    rs485
    can
    canfd
    lin
    flexray
    onewire
    usb
    sdio
    ethernet
    swd
    jtag
    pcie
    emmc
    ahb
    apb
    axi
    dma
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
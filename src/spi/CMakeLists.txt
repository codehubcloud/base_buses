set(SPI_SOURCES spi.c spi_hal.c)
set(SPI_HEADERS spi.h spi_hal.h)
add_library(spi STATIC ${SPI_SOURCES} ${SPI_HEADERS})
target_include_directories(spi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
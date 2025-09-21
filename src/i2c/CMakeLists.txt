set(I2C_SOURCES i2c.c i2c_hal.c)
set(I2C_HEADERS i2c.h i2c_hal.h)
add_library(i2c STATIC ${I2C_SOURCES} ${I2C_HEADERS})
target_include_directories(i2c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
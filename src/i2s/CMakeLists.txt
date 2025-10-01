set(I2S_SOURCES i2s.c i2s_hal.c)
set(I2S_HEADERS i2s.h i2s_hal.h)
add_library(i2s STATIC ${I2S_SOURCES} ${I2S_HEADERS})
target_include_directories(i2s PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

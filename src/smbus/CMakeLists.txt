set(SMBUS_SOURCES smbus.c smbus_hal.c)
set(SMBUS_HEADERS smbus.h smbus_hal.h)
add_library(smbus STATIC ${SMBUS_SOURCES} ${SMBUS_HEADERS})
target_include_directories(smbus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
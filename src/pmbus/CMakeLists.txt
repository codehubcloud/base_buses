set(PMBUS_SOURCES pmbus.c pmbus_hal.c)
set(PMBUS_HEADERS pmbus.h pmbus_hal.h)
add_library(pmbus STATIC ${PMBUS_SOURCES} ${PMBUS_HEADERS})
target_include_directories(pmbus PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

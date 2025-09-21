set(RS232_SOURCES rs232.c rs232_hal.c)
set(RS232_HEADERS rs232.h rs232_hal.h)
add_library(rs232 STATIC ${RS232_SOURCES} ${RS232_HEADERS})
target_include_directories(rs232 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
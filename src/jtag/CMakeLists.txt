set(JTAG_SOURCES jtag.c jtag_hal.c)
set(JTAG_HEADERS jtag.h jtag_hal.h)
add_library(jtag STATIC ${JTAG_SOURCES} ${JTAG_HEADERS})
target_include_directories(jtag PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

set(UART_SOURCES uart.c uart_hal.c)
set(UART_HEADERS uart.h uart_hal.h)
add_library(uart STATIC ${UART_SOURCES} ${UART_HEADERS})
target_include_directories(uart PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
set(SWD_SOURCES swd.c swd_hal.c)
set(SWD_HEADERS swd.h swd_hal.h)
add_library(swd STATIC ${SWD_SOURCES} ${SWD_HEADERS})
target_include_directories(swd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

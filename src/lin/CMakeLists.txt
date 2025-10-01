set(LIN_SOURCES lin.c lin_hal.c)
set(LIN_HEADERS lin.h lin_hal.h)
add_library(lin STATIC ${LIN_SOURCES} ${LIN_HEADERS})
target_include_directories(lin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

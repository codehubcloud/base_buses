set(I3C_SOURCES i3c.c i3c_hal.c)
set(I3C_HEADERS i3c.h i3c_hal.h)
add_library(i3c STATIC ${I3C_SOURCES} ${I3C_HEADERS})
target_include_directories(i3c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
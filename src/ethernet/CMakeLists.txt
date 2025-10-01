set(ETHERNET_SOURCES ethernet.c ethernet_hal.c)
set(ETHERNET_HEADERS ethernet.h ethernet_hal.h)
add_library(ethernet STATIC ${ETHERNET_SOURCES} ${ETHERNET_HEADERS})
target_include_directories(ethernet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

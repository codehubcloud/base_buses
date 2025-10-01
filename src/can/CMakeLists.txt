set(CAN_SOURCES can.c can_hal.c)
set(CAN_HEADERS can.h can_hal.h)
add_library(can STATIC ${CAN_SOURCES} ${CAN_HEADERS})
target_include_directories(can PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)
set(CANFD_SOURCES canfd.c canfd_hal.c)
set(CANFD_HEADERS canfd.h canfd_hal.h)
add_library(canfd STATIC ${CANFD_SOURCES} ${CANFD_HEADERS})
target_include_directories(canfd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

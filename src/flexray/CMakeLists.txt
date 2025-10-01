set(FLEXRAY_SOURCES flexray.c flexray_hal.c)
set(FLEXRAY_HEADERS flexray.h flexray_hal.h)
add_library(flexray STATIC ${FLEXRAY_SOURCES} ${FLEXRAY_HEADERS})
target_include_directories(flexray PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

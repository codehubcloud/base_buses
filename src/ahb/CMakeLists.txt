set(AHB_SOURCES ahb.c ahb_hal.c)
set(AHB_HEADERS ahb.h ahb_hal.h)
add_library(ahb STATIC ${AHB_SOURCES} ${AHB_HEADERS})
target_include_directories(ahb PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

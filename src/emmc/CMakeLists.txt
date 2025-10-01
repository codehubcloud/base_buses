set(EMMC_SOURCES emmc.c emmc_hal.c)
set(EMMC_HEADERS emmc.h emmc_hal.h)
add_library(emmc STATIC ${EMMC_SOURCES} ${EMMC_HEADERS})
target_include_directories(emmc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

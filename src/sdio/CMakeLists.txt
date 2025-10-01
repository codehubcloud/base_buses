# SDIO protocol module
add_library(sdio STATIC
    sdio.c
    sdio_hal.c
)

target_include_directories(sdio PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)

target_link_libraries(sdio PRIVATE securec)

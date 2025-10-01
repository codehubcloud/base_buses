# USB protocol module
add_library(usb STATIC
    usb.c
    usb_hal.c
)

target_include_directories(usb PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include
)

target_link_libraries(usb PRIVATE securec)

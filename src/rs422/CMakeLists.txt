set(RS422_SOURCES rs422.c rs422_hal.c)
set(RS422_HEADERS rs422.h rs422_hal.h)
add_library(rs422 STATIC ${RS422_SOURCES} ${RS422_HEADERS})
target_include_directories(rs422 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

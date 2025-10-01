# Get all source files
file(GLOB SECURELIB_SOURCES *.c)

# Exclude secinput.h, securecutil.h, and secureprintoutput.h as they are headers only used internally
list(FILTER SECURELIB_SOURCES EXCLUDE REGEX ".*secinput.h")
list(FILTER SECURELIB_SOURCES EXCLUDE REGEX ".*securecutil.h")
list(FILTER SECURELIB_SOURCES EXCLUDE REGEX ".*secureprintoutput.h")

add_library(securec STATIC ${SECURELIB_SOURCES})

target_include_directories(securec PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
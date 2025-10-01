set(DMA_SOURCES dma.c dma_hal.c)
set(DMA_HEADERS dma.h dma_hal.h)
add_library(dma STATIC ${DMA_SOURCES} ${DMA_HEADERS})
target_include_directories(dma PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/3rd_party/securelib/include)

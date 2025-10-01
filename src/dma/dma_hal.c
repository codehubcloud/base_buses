#include "dma_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static DMA_HandleTypeDef g_dmaHandles[2][8] = {0}; /* DMA1 and DMA2, 8 streams each */
#endif

#ifdef PLATFORM_STM32F1
static DMA_HandleTypeDef g_dmaHandles[2][7] = {0}; /* DMA1: 7 channels, DMA2: 5 channels */
#endif

#ifdef PLATFORM_ESP32
#include "driver/gdma.h"
static gdma_channel_handle_t g_gdmaChannel = NULL;
#endif

#ifdef PLATFORM_LINUX
#include <string.h>
#include <time.h>
#include <unistd.h>
typedef struct {
    uint8_t active;
    uint32_t srcAddr;
    uint32_t destAddr;
    uint32_t dataLength;
    uint32_t remaining;
    DmaStatus status;
} LinuxDmaChannel;
static LinuxDmaChannel g_linuxDmaChannels[2][8] = {0};
#endif

/******************************************************************************
 * @brief     : Enable DMA controller clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA clock managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable DMA controller clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaDisableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_DMA2_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_DMA2_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA clock managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require clock disabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Initialize DMA hardware resources
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaInitializeHardware(void)
{
#ifdef PLATFORM_STM32F4
    /* Hardware initialized per-channel in configure */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Hardware initialized per-channel in configure */
    return 0;
#elif defined(PLATFORM_ESP32)
    gdma_channel_alloc_config_t channelConfig = {
        .direction = GDMA_CHANNEL_DIRECTION_TX,
    };
    if (gdma_new_channel(&channelConfig, &g_gdmaChannel) != ESP_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Initialize simulated DMA channels */
    if (memset_s(g_linuxDmaChannels, sizeof(g_linuxDmaChannels), 0, sizeof(g_linuxDmaChannels)) != EOK) {
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Release DMA hardware resources
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaReleaseHardware(void)
{
#ifdef PLATFORM_STM32F4
    /* Release all DMA handles */
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 8; j++) {
            if (g_dmaHandles[i][j].Instance != NULL) {
                HAL_DMA_DeInit(&g_dmaHandles[i][j]);
            }
        }
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Release all DMA handles */
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 7; j++) {
            if (g_dmaHandles[i][j].Instance != NULL) {
                HAL_DMA_DeInit(&g_dmaHandles[i][j]);
            }
        }
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    if (g_gdmaChannel != NULL) {
        gdma_del_channel(g_gdmaChannel);
        g_gdmaChannel = NULL;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    if (memset_s(g_linuxDmaChannels, sizeof(g_linuxDmaChannels), 0, sizeof(g_linuxDmaChannels)) != EOK) {
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Validate DMA configuration parameters
 * @param[in] : config - Pointer to DMA configuration structure
 * @param[out]: None
 * @return    : 0 if valid, -1 if invalid
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaValidateConfig(const DmaConfig* config)
{
    if (config == NULL) {
        return -1;
    }

    if (config->controller < 1 || config->controller > 2) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    if (config->channel > 7) {
        return -1;
    }
#elif defined(PLATFORM_STM32F1)
    if (config->channel > 6) {
        return -1;
    }
#endif

    if (config->direction > DMA_DIR_PERIPH2PERIPH) {
        return -1;
    }

    if (config->dataWidth > DMA_DATA_WIDTH_WORD) {
        return -1;
    }

    if (config->priority > DMA_PRIORITY_VERY_HIGH) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Configure DMA channel with hardware-specific settings
 * @param[in] : config - Pointer to DMA configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalConfigureChannel(const DmaConfig* config)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[config->controller - 1][config->channel];

    /* Get DMA stream instance */
    DMA_Stream_TypeDef* streamTable[2][8] = {{DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5,
                                              DMA1_Stream6, DMA1_Stream7},
                                             {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5,
                                              DMA2_Stream6, DMA2_Stream7}};

    hdma->Instance = streamTable[config->controller - 1][config->channel];

    /* Configure DMA parameters */
    hdma->Init.Channel = DMA_CHANNEL_0;
    hdma->Init.Direction = (config->direction == DMA_DIR_MEM2MEM)      ? DMA_MEMORY_TO_MEMORY
                           : (config->direction == DMA_DIR_PERIPH2MEM) ? DMA_PERIPH_TO_MEMORY
                                                                       : DMA_MEMORY_TO_PERIPH;
    hdma->Init.PeriphInc = config->peripheralIncrement ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;
    hdma->Init.MemInc = config->memoryIncrement ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    hdma->Init.PeriphDataAlignment = (config->dataWidth == DMA_DATA_WIDTH_BYTE)       ? DMA_PDATAALIGN_BYTE
                                     : (config->dataWidth == DMA_DATA_WIDTH_HALFWORD) ? DMA_PDATAALIGN_HALFWORD
                                                                                      : DMA_PDATAALIGN_WORD;
    hdma->Init.MemDataAlignment = hdma->Init.PeriphDataAlignment;
    hdma->Init.Mode = (config->mode == DMA_MODE_CIRCULAR) ? DMA_CIRCULAR : DMA_NORMAL;
    hdma->Init.Priority = (config->priority == DMA_PRIORITY_LOW)      ? DMA_PRIORITY_LOW
                          : (config->priority == DMA_PRIORITY_MEDIUM) ? DMA_PRIORITY_MEDIUM
                          : (config->priority == DMA_PRIORITY_HIGH)   ? DMA_PRIORITY_HIGH
                                                                      : DMA_PRIORITY_VERY_HIGH;
    hdma->Init.FIFOMode = config->enableFifo ? DMA_FIFOMODE_ENABLE : DMA_FIFOMODE_DISABLE;
    hdma->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma->Init.MemBurst = DMA_MBURST_SINGLE;
    hdma->Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(hdma) != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[config->controller - 1][config->channel];

    /* Get DMA channel instance */
    DMA_Channel_TypeDef* channelTable[2][7] = {{DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6,
                                                DMA1_Channel7},
                                               {DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5, NULL, NULL}};

    hdma->Instance = channelTable[config->controller - 1][config->channel];
    if (hdma->Instance == NULL) {
        return -1;
    }

    /* Configure DMA parameters */
    hdma->Init.Direction = (config->direction == DMA_DIR_MEM2MEM)      ? DMA_MEMORY_TO_MEMORY
                           : (config->direction == DMA_DIR_PERIPH2MEM) ? DMA_PERIPH_TO_MEMORY
                                                                       : DMA_MEMORY_TO_PERIPH;
    hdma->Init.PeriphInc = config->peripheralIncrement ? DMA_PINC_ENABLE : DMA_PINC_DISABLE;
    hdma->Init.MemInc = config->memoryIncrement ? DMA_MINC_ENABLE : DMA_MINC_DISABLE;
    hdma->Init.PeriphDataAlignment = (config->dataWidth == DMA_DATA_WIDTH_BYTE)       ? DMA_PDATAALIGN_BYTE
                                     : (config->dataWidth == DMA_DATA_WIDTH_HALFWORD) ? DMA_PDATAALIGN_HALFWORD
                                                                                      : DMA_PDATAALIGN_WORD;
    hdma->Init.MemDataAlignment = hdma->Init.PeriphDataAlignment;
    hdma->Init.Mode = (config->mode == DMA_MODE_CIRCULAR) ? DMA_CIRCULAR : DMA_NORMAL;
    hdma->Init.Priority = (config->priority == DMA_PRIORITY_LOW)      ? DMA_PRIORITY_LOW
                          : (config->priority == DMA_PRIORITY_MEDIUM) ? DMA_PRIORITY_MEDIUM
                          : (config->priority == DMA_PRIORITY_HIGH)   ? DMA_PRIORITY_HIGH
                                                                      : DMA_PRIORITY_VERY_HIGH;

    if (HAL_DMA_Init(hdma) != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA configured per transfer */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulated configuration */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Start DMA transfer at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, srcAddr - Source address, destAddr - Destination
 *address, dataLength - Number of data items
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalStartTransfer(uint8_t controller, uint8_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t dataLength)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    if (HAL_DMA_Start(hdma, srcAddr, destAddr, dataLength) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    if (HAL_DMA_Start(hdma, srcAddr, destAddr, dataLength) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA transfer - simplified simulation */
    memcpy((void*)destAddr, (void*)srcAddr, dataLength);
    return 0;
#elif defined(PLATFORM_LINUX)
    LinuxDmaChannel* ch = &g_linuxDmaChannels[controller - 1][channel];
    ch->active = 1;
    ch->srcAddr = srcAddr;
    ch->destAddr = destAddr;
    ch->dataLength = dataLength;
    ch->remaining = dataLength;
    ch->status = DMA_STATUS_IN_PROGRESS;

    /* Simulate DMA transfer with memcpy */
    memcpy((void*)destAddr, (void*)srcAddr, dataLength);

    ch->remaining = 0;
    ch->status = DMA_STATUS_COMPLETE;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Stop DMA transfer at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalStopTransfer(uint8_t controller, uint8_t channel)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    if (HAL_DMA_Abort(hdma) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    if (HAL_DMA_Abort(hdma) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA stop */
    return 0;
#elif defined(PLATFORM_LINUX)
    LinuxDmaChannel* ch = &g_linuxDmaChannels[controller - 1][channel];
    ch->active = 0;
    ch->status = DMA_STATUS_IDLE;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Get DMA transfer status from hardware
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : DmaStatus value
 * @note      : Platform-specific implementation
 *****************************************************************************/
DmaStatus DmaHalGetStatus(uint8_t controller, uint8_t channel)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    HAL_DMA_StateTypeDef state = HAL_DMA_GetState(hdma);

    if (state == HAL_DMA_STATE_READY) {
        return DMA_STATUS_COMPLETE;
    } else if (state == HAL_DMA_STATE_BUSY) {
        return DMA_STATUS_IN_PROGRESS;
    } else if (state == HAL_DMA_STATE_ERROR) {
        return DMA_STATUS_ERROR;
    } else {
        return DMA_STATUS_IDLE;
    }
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    HAL_DMA_StateTypeDef state = HAL_DMA_GetState(hdma);

    if (state == HAL_DMA_STATE_READY) {
        return DMA_STATUS_COMPLETE;
    } else if (state == HAL_DMA_STATE_BUSY) {
        return DMA_STATUS_IN_PROGRESS;
    } else if (state == HAL_DMA_STATE_ERROR) {
        return DMA_STATUS_ERROR;
    } else {
        return DMA_STATUS_IDLE;
    }
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA status */
    return DMA_STATUS_COMPLETE;
#elif defined(PLATFORM_LINUX)
    return g_linuxDmaChannels[controller - 1][channel].status;
#else
    return DMA_STATUS_ERROR;
#endif
}

/******************************************************************************
 * @brief     : Enable DMA interrupts at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, interruptFlags - Interrupt flags to enable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalEnableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];

    if (interruptFlags & DMA_INT_TC) {
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);
    }
    if (interruptFlags & DMA_INT_HT) {
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_HT);
    }
    if (interruptFlags & DMA_INT_TE) {
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];

    if (interruptFlags & DMA_INT_TC) {
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);
    }
    if (interruptFlags & DMA_INT_HT) {
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_HT);
    }
    if (interruptFlags & DMA_INT_TE) {
        __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA interrupts */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulated interrupts */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable DMA interrupts at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, interruptFlags - Interrupt flags to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalDisableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];

    if (interruptFlags & DMA_INT_TC) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
    if (interruptFlags & DMA_INT_HT) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
    }
    if (interruptFlags & DMA_INT_TE) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE);
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];

    if (interruptFlags & DMA_INT_TC) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
    if (interruptFlags & DMA_INT_HT) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);
    }
    if (interruptFlags & DMA_INT_TE) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TE);
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA interrupts */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulated interrupts */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for DMA interrupts
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaConfigureNvic(uint8_t controller, uint8_t channel)
{
#ifdef PLATFORM_STM32F4
    IRQn_Type irqTable[2][8] = {{DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn,
                                 DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn},
                                {DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn,
                                 DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn}};

    IRQn_Type irqn = irqTable[controller - 1][channel];
    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);
    return 0;
#elif defined(PLATFORM_STM32F1)
    IRQn_Type irqTable[2][7] = {{DMA1_Channel1_IRQn, DMA1_Channel2_IRQn, DMA1_Channel3_IRQn, DMA1_Channel4_IRQn, DMA1_Channel5_IRQn,
                                 DMA1_Channel6_IRQn, DMA1_Channel7_IRQn},
                                {DMA2_Channel1_IRQn, DMA2_Channel2_IRQn, DMA2_Channel3_IRQn, DMA2_Channel4_5_IRQn, DMA2_Channel4_5_IRQn, 0,
                                 0}};

    IRQn_Type irqn = irqTable[controller - 1][channel];
    if (irqn != 0) {
        HAL_NVIC_SetPriority(irqn, 0, 0);
        HAL_NVIC_EnableIRQ(irqn);
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA NVIC */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use NVIC */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Set DMA circular mode
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, enable - 1 to enable, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalSetCircularMode(uint8_t controller, uint8_t channel, uint8_t enable)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    hdma->Init.Mode = enable ? DMA_CIRCULAR : DMA_NORMAL;
    if (HAL_DMA_Init(hdma) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];
    hdma->Init.Mode = enable ? DMA_CIRCULAR : DMA_NORMAL;
    if (HAL_DMA_Init(hdma) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA circular mode */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulated circular mode */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Get remaining data count from hardware
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : Remaining data count
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint32_t DmaHalGetRemainingCount(uint8_t controller, uint8_t channel)
{
#ifdef PLATFORM_STM32F4
    DMA_Stream_TypeDef* streamTable[2][8] = {{DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5,
                                              DMA1_Stream6, DMA1_Stream7},
                                             {DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5,
                                              DMA2_Stream6, DMA2_Stream7}};
    DMA_Stream_TypeDef* stream = streamTable[controller - 1][channel];
    return stream->NDTR;
#elif defined(PLATFORM_STM32F1)
    DMA_Channel_TypeDef* channelTable[2][7] = {{DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6,
                                                DMA1_Channel7},
                                               {DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4, DMA2_Channel5, NULL, NULL}};
    DMA_Channel_TypeDef* ch = channelTable[controller - 1][channel];
    if (ch == NULL) {
        return 0;
    }
    return ch->CNDTR;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA remaining count */
    return 0;
#elif defined(PLATFORM_LINUX)
    return g_linuxDmaChannels[controller - 1][channel].remaining;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Clear DMA interrupt flags at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, interruptFlags - Interrupt flags to clear
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t DmaHalClearInterruptFlags(uint8_t controller, uint8_t channel, uint8_t interruptFlags)
{
#ifdef PLATFORM_STM32F4
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];

    if (interruptFlags & DMA_INT_TC) {
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TCIF0_4 << (channel * 6));
    }
    if (interruptFlags & DMA_INT_HT) {
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_HTIF0_4 << (channel * 6));
    }
    if (interruptFlags & DMA_INT_TE) {
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TEIF0_4 << (channel * 6));
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    DMA_HandleTypeDef* hdma = &g_dmaHandles[controller - 1][channel];

    if (interruptFlags & DMA_INT_TC) {
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TC1 << ((channel - 1) * 4));
    }
    if (interruptFlags & DMA_INT_HT) {
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_HT1 << ((channel - 1) * 4));
    }
    if (interruptFlags & DMA_INT_TE) {
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TE1 << ((channel - 1) * 4));
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GDMA clear interrupts */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulated interrupt clearing */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Delay for specified milliseconds
 * @param[in] : delayMs - Delay time in milliseconds
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void DmaDelayMs(uint32_t delayMs)
{
#ifdef PLATFORM_STM32F4
    HAL_Delay(delayMs);
#elif defined(PLATFORM_STM32F1)
    HAL_Delay(delayMs);
#elif defined(PLATFORM_ESP32)
    vTaskDelay(delayMs / portTICK_PERIOD_MS);
#elif defined(PLATFORM_LINUX)
    usleep(delayMs * 1000);
#endif
}

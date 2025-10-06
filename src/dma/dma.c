#include <string.h>
#include "dma.h"
#include "dma_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize DMA controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables DMA clock and initializes hardware resources
 *****************************************************************************/
int32_t DmaInit(void)
{
    int32_t result = 0;

    result = DmaEnableClock();
    if (result != 0) {
        return -1;
    }

    result = DmaInitializeHardware();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize DMA controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables DMA clock and releases hardware resources
 *****************************************************************************/
int32_t DmaDeinit(void)
{
    int32_t result = 0;

    result = DmaReleaseHardware();
    if (result != 0) {
        return -1;
    }

    result = DmaDisableClock();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Configure DMA channel with specified parameters
 * @param[in] : config --Pointer to DMA configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before starting DMA transfer
 *****************************************************************************/
int32_t DmaConfigureChannel(const DmaConfig* config)
{
    if (config == NULL) {
        return -1;
    }

    if (DmaValidateConfig(config) != 0) {
        return -1;
    }

    if (DmaHalConfigureChannel(config) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Start DMA transfer
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number, srcAddr - Source address, destAddr -
 *Destination address, dataLength - Number of data items to transfer
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Transfer parameters must be properly aligned
 *****************************************************************************/
int32_t DmaStartTransfer(uint8_t controller, uint8_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t dataLength)
{
    if (dataLength == 0) {
        return -1;
    }

    if (srcAddr == 0 || destAddr == 0) {
        return -1;
    }

    if (DmaHalStartTransfer(controller, channel, srcAddr, destAddr, dataLength) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Stop DMA transfer
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Stops ongoing transfer immediately
 *****************************************************************************/
int32_t DmaStopTransfer(uint8_t controller, uint8_t channel)
{
    if (DmaHalStopTransfer(controller, channel) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Get DMA transfer status
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number
 * @param[out]: None
 * @return    : DmaStatus value, or DMA_STATUS_ERROR if invalid parameters
 * @note      : Returns current transfer state
 *****************************************************************************/
DmaStatus DmaGetStatus(uint8_t controller, uint8_t channel)
{
    return DmaHalGetStatus(controller, channel);
}

/******************************************************************************
 * @brief     : Wait for DMA transfer completion
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number, timeoutMs - Timeout in milliseconds (0 =
 *wait forever)
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout or error
 * @note      : Blocking function that polls transfer status
 *****************************************************************************/
int32_t DmaWaitComplete(uint8_t controller, uint8_t channel, uint32_t timeoutMs)
{
    uint32_t elapsedMs = 0;
    DmaStatus status;

    while (1) {
        status = DmaGetStatus(controller, channel);

        if (status == DMA_STATUS_COMPLETE) {
            return 0;
        }

        if (status == DMA_STATUS_ERROR) {
            return -1;
        }

        if (timeoutMs > 0 && elapsedMs >= timeoutMs) {
            return -1;
        }

        DmaDelayMs(1);
        elapsedMs++;
    }
}

/******************************************************************************
 * @brief     : Enable DMA interrupts
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number, interruptFlags - Interrupt flags to enable
 *(bitwise OR of DmaInterruptFlag)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables specified interrupt types and configures NVIC
 *****************************************************************************/
int32_t DmaEnableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags)
{
    if (interruptFlags == 0) {
        return -1;
    }

    if (DmaHalEnableInterrupt(controller, channel, interruptFlags) != 0) {
        return -1;
    }

    if (DmaConfigureNvic(controller, channel) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Disable DMA interrupts
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number, interruptFlags - Interrupt flags to
 *disable (bitwise OR of DmaInterruptFlag)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables specified interrupt types
 *****************************************************************************/
int32_t DmaDisableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags)
{
    if (interruptFlags == 0) {
        return -1;
    }

    if (DmaHalDisableInterrupt(controller, channel, interruptFlags) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Perform memory-to-memory copy using DMA
 * @param[in] : srcAddr --Source memory address, destAddr - Destination memory address, dataLength - Number of bytes to copy
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : High-performance alternative to memcpy for large data
 *****************************************************************************/
int32_t DmaMemCopy(uint32_t srcAddr, uint32_t destAddr, uint32_t dataLength)
{
    DmaConfig config = {0};

    if (srcAddr == 0 || destAddr == 0 || dataLength == 0) {
        return -1;
    }

    config.controller = 1;
    config.channel = 0;
    config.direction = DMA_DIR_MEM2MEM;
    config.dataWidth = DMA_DATA_WIDTH_WORD;
    config.priority = DMA_PRIORITY_HIGH;
    config.mode = DMA_MODE_NORMAL;
    config.memoryIncrement = 1;
    config.peripheralIncrement = 1;
    config.enableFifo = 0;

    if (DmaConfigureChannel(&config) != 0) {
        return -1;
    }

    if (DmaStartTransfer(config.controller, config.channel, srcAddr, destAddr, dataLength) != 0) {
        return -1;
    }

    if (DmaWaitComplete(config.controller, config.channel, 5000) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Configure circular buffer mode
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number, srcAddr - Source address, destAddr -
 *Destination address, bufferSize - Size of circular buffer
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Automatically restarts transfer after completion
 *****************************************************************************/
int32_t DmaCircularMode(uint8_t controller, uint8_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t bufferSize)
{
    if (srcAddr == 0 || destAddr == 0 || bufferSize == 0) {
        return -1;
    }

    if (DmaHalSetCircularMode(controller, channel, 1) != 0) {
        return -1;
    }

    if (DmaStartTransfer(controller, channel, srcAddr, destAddr, bufferSize) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Get remaining data count in current transfer
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number
 * @param[out]: None
 * @return    : Remaining data count, or 0 if error
 * @note      : Useful for monitoring transfer progress
 *****************************************************************************/
uint32_t DmaGetRemainingCount(uint8_t controller, uint8_t channel)
{
    return DmaHalGetRemainingCount(controller, channel);
}

/******************************************************************************
 * @brief     : Clear DMA interrupt flags
 * @param[in] : controller --DMA controller number (1 or 2), channel - DMA channel/stream number, interruptFlags - Interrupt flags to clear
 *(bitwise OR of DmaInterruptFlag)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called in interrupt handler to clear flags
 *****************************************************************************/
int32_t DmaClearInterruptFlags(uint8_t controller, uint8_t channel, uint8_t interruptFlags)
{
    if (interruptFlags == 0) {
        return -1;
    }

    if (DmaHalClearInterruptFlags(controller, channel, interruptFlags) != 0) {
        return -1;
    }

    return 0;
}

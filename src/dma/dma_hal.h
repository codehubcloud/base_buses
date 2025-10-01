#ifndef DMA_HAL_H
#define DMA_HAL_H

#include <stdint.h>
#include "dma.h"

/******************************************************************************
 * @brief     : Enable DMA controller clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaEnableClock(void);

/******************************************************************************
 * @brief     : Disable DMA controller clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaDisableClock(void);

/******************************************************************************
 * @brief     : Initialize DMA hardware resources
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaInitializeHardware(void);

/******************************************************************************
 * @brief     : Release DMA hardware resources
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaReleaseHardware(void);

/******************************************************************************
 * @brief     : Validate DMA configuration parameters
 * @param[in] : config - Pointer to DMA configuration structure
 * @param[out]: None
 * @return    : 0 if valid, -1 if invalid
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaValidateConfig(const DmaConfig* config);

/******************************************************************************
 * @brief     : Configure DMA channel with hardware-specific settings
 * @param[in] : config - Pointer to DMA configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalConfigureChannel(const DmaConfig* config);

/******************************************************************************
 * @brief     : Start DMA transfer at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, srcAddr - Source address, destAddr - Destination address, dataLength - Number of data items
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalStartTransfer(uint8_t controller, uint8_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t dataLength);

/******************************************************************************
 * @brief     : Stop DMA transfer at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalStopTransfer(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Get DMA transfer status from hardware
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : DmaStatus value
 * @note      : Platform-specific implementation required
 *****************************************************************************/
DmaStatus DmaHalGetStatus(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Enable DMA interrupts at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, interruptFlags - Interrupt flags to enable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalEnableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags);

/******************************************************************************
 * @brief     : Disable DMA interrupts at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, interruptFlags - Interrupt flags to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalDisableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags);

/******************************************************************************
 * @brief     : Configure NVIC for DMA interrupts
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaConfigureNvic(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Set DMA circular mode
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, enable - 1 to enable, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalSetCircularMode(uint8_t controller, uint8_t channel, uint8_t enable);

/******************************************************************************
 * @brief     : Get remaining data count from hardware
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number
 * @param[out]: None
 * @return    : Remaining data count
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint32_t DmaHalGetRemainingCount(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Clear DMA interrupt flags at hardware level
 * @param[in] : controller - DMA controller number, channel - DMA channel/stream number, interruptFlags - Interrupt flags to clear
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t DmaHalClearInterruptFlags(uint8_t controller, uint8_t channel, uint8_t interruptFlags);

/******************************************************************************
 * @brief     : Delay for specified milliseconds
 * @param[in] : delayMs - Delay time in milliseconds
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void DmaDelayMs(uint32_t delayMs);

#endif // DMA_HAL_H

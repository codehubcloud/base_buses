#ifndef FLEXRAY_HAL_H
#define FLEXRAY_HAL_H

#include <stdint.h>
#include "flexray.h"

/******************************************************************************
 * @brief     : Enable FlexRay clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayEnableClock(void);

/******************************************************************************
 * @brief     : Configure FlexRay GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayConfigureGpio(void);

/******************************************************************************
 * @brief     : Initialize FlexRay HAL
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalInit(void);

/******************************************************************************
 * @brief     : Deinitialize FlexRay HAL
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalDeinit(void);

/******************************************************************************
 * @brief     : Configure FlexRay bit rate
 * @param[in] : bitRate --Desired bit rate in bps
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalConfigureBitRate(uint32_t bitRate);

/******************************************************************************
 * @brief     : Configure FlexRay slot in hardware
 * @param[in] : slotId --Slot identifier
 * @param[in] : channel --Channel selection
 * @param[in] : payloadLength --Payload length in bytes
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalConfigureSlot(uint16_t slotId, uint8_t channel, uint16_t payloadLength);

/******************************************************************************
 * @brief     : Send FlexRay frame
 * @param[in] : slotId --Slot identifier
 * @param[in] : frame --Pointer to frame structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalSendFrame(uint16_t slotId, FlexRayFrame_t* frame);

/******************************************************************************
 * @brief     : Check if RX slot has data
 * @param[in] : slotId --Slot identifier
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalRxSlotHasData(uint16_t slotId);

/******************************************************************************
 * @brief     : Receive FlexRay frame
 * @param[in] : slotId --Slot identifier
 * @param[out]: frame --Pointer to frame structure to fill
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalReceiveFrame(uint16_t slotId, FlexRayFrame_t* frame);

/******************************************************************************
 * @brief     : Start FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalStartCommunication(void);

/******************************************************************************
 * @brief     : Stop FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalStopCommunication(void);

/******************************************************************************
 * @brief     : Update FlexRay status from hardware
 * @param[in] : status --Pointer to status structure
 * @param[out]: status --Updated status structure
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalUpdateStatus(FlexRayStatus_t* status);

/******************************************************************************
 * @brief     : Configure FlexRay timing parameters
 * @param[in] : cycleLength --Cycle length in macroticks
 * @param[in] : staticSlots --Number of static slots
 * @param[in] : dynamicSlots --Number of dynamic slots
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t FlexRayHalConfigureTiming(uint16_t cycleLength, uint16_t staticSlots, uint16_t dynamicSlots);

/******************************************************************************
 * @brief     : Enable FlexRay slot interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalEnableSlotInterrupt(void);

/******************************************************************************
 * @brief     : Enable FlexRay error interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalEnableErrorInterrupt(void);

/******************************************************************************
 * @brief     : Configure NVIC for FlexRay
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalConfigureNvic(void);

/******************************************************************************
 * @brief     : Disable FlexRay slot interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalDisableSlotInterrupt(void);

/******************************************************************************
 * @brief     : Disable FlexRay error interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalDisableErrorInterrupt(void);

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void FlexRayHalUpdateNvic(void);

#endif // FLEXRAY_HAL_H

#ifndef CANFD_HAL_H
#define CANFD_HAL_H

#include <stdint.h>
#include "canfd.h"

/******************************************************************************
 * @brief     : Enable CAN FD clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdEnableClock(void);

/******************************************************************************
 * @brief     : Disable CAN FD clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdDisableClock(void);

/******************************************************************************
 * @brief     : Configure CAN FD GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable CAN FD module
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdEnable(void);

/******************************************************************************
 * @brief     : Disable CAN FD module
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdDisable(void);

/******************************************************************************
 * @brief     : Check if CAN FD TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdTxBufferEmpty(void);

/******************************************************************************
 * @brief     : Write CAN FD frame
 * @param[in] : frame --Pointer to CAN FD frame structure to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdWriteFrame(const CanFdFrame* frame);

/******************************************************************************
 * @brief     : Check if CAN FD RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdRxBufferHasData(void);

/******************************************************************************
 * @brief     : Read CAN FD frame
 * @param[in] : None
 * @param[out]: frame --Pointer to CAN FD frame structure to store received data
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdReadFrame(CanFdFrame* frame);

/******************************************************************************
 * @brief     : Configure CAN FD bit rates
 * @param[in] : nominalBitRate --Nominal phase bit rate in bps
 * @param[in] : dataBitRate --Data phase bit rate in bps
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdConfigureBitRate(uint32_t nominalBitRate, uint32_t dataBitRate);

/******************************************************************************
 * @brief     : Configure CAN FD acceptance filter
 * @param[in] : filterId --Filter ID value
 * @param[in] : filterMask --Filter mask value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdConfigureFilter(uint32_t filterId, uint32_t filterMask);

/******************************************************************************
 * @brief     : Read CAN FD error counters
 * @param[in] : None
 * @param[out]: errorCount --Pointer to structure to store error counters
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t CanFdReadErrorCount(CanFdErrorCount* errorCount);

/******************************************************************************
 * @brief     : Enable CAN FD RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void CanFdEnableRxInterrupt(void);

/******************************************************************************
 * @brief     : Enable CAN FD TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void CanFdEnableTxInterrupt(void);

/******************************************************************************
 * @brief     : Configure NVIC for CAN FD
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void CanFdConfigureNvic(void);

/******************************************************************************
 * @brief     : Disable CAN FD RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void CanFdDisableRxInterrupt(void);

/******************************************************************************
 * @brief     : Disable CAN FD TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void CanFdDisableTxInterrupt(void);

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void CanFdUpdateNvic(void);

#endif // CANFD_HAL_H

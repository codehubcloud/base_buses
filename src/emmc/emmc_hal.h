#ifndef EMMC_HAL_H
#define EMMC_HAL_H

#include <stdint.h>
#include "emmc.h"

/******************************************************************************
 * @brief     : Initialize eMMC hardware abstraction layer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalInit(void);

/******************************************************************************
 * @brief     : Deinitialize eMMC hardware abstraction layer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalDeinit(void);

/******************************************************************************
 * @brief     : Send command to eMMC device
 * @param[in] : cmd - Command index, arg - Command argument
 * @param[out]: response - Command response (can be NULL if not needed)
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalSendCommand(uint8_t cmd, uint32_t arg, uint32_t* response);

/******************************************************************************
 * @brief     : Read data from eMMC device
 * @param[in] : length - Number of bytes to read
 * @param[out]: data - Buffer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalReadData(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Write data to eMMC device
 * @param[in] : data - Data to write, length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalWriteData(const uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Set eMMC bus width at hardware level
 * @param[in] : busWidth - Bus width to set (1-bit, 4-bit, or 8-bit)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalSetBusWidth(EmmcBusWidth_E busWidth);

/******************************************************************************
 * @brief     : Set eMMC clock speed at hardware level
 * @param[in] : speedMode - Speed mode to set
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalSetClockSpeed(EmmcSpeedMode_E speedMode);

/******************************************************************************
 * @brief     : Delay for specified milliseconds
 * @param[in] : ms - Milliseconds to delay
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void EmmcHalDelay(uint32_t ms);

/******************************************************************************
 * @brief     : Enable eMMC clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalEnableClock(void);

/******************************************************************************
 * @brief     : Configure eMMC GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalConfigureGpio(void);

/******************************************************************************
 * @brief     : Wait for transfer complete
 * @param[in] : timeoutMs - Timeout in milliseconds
 * @param[out]: None
 * @return    : 0 if success, -1 if error or timeout
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EmmcHalWaitTransferComplete(uint32_t timeoutMs);

#endif // EMMC_HAL_H

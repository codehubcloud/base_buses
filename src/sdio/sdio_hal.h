/******************************************************************************
 * @file    : sdio_hal.h
 * @brief   : SDIO Hardware Abstraction Layer header file
 * @author  : Code Generator
 * @date    : 2025-10-01
 * @version : V1.0
 * @note    : Platform-specific SDIO hardware interface
 ******************************************************************************/

#ifndef SDIO_HAL_H
#define SDIO_HAL_H

#include <stdint.h>
#include "sdio.h"

/******************************************************************************
 * @brief     : Enable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SdioEnableClock(void);

/******************************************************************************
 * @brief     : Disable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SdioDisableClock(void);

/******************************************************************************
 * @brief     : Configure SDIO GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures CLK, CMD, and DAT pins
 *****************************************************************************/
int32_t SdioConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Activates SDIO controller
 *****************************************************************************/
void SdioEnable(void);

/******************************************************************************
 * @brief     : Disable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Deactivates SDIO controller
 *****************************************************************************/
void SdioDisable(void);

/******************************************************************************
 * @brief     : Send command via HAL layer
 * @param[in] : cmd - Command index, arg - Command argument, crc - CRC7 checksum
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Hardware-specific command transmission
 *****************************************************************************/
int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc);

/******************************************************************************
 * @brief     : Read response via HAL layer
 * @param[in] : responseType - Type of response expected
 * @param[out]: response - Buffer to store response data
 * @return    : 0 if success, -1 if error
 * @note      : Hardware-specific response reception
 *****************************************************************************/
int32_t SdioReadResponseHal(uint32_t* response, SdioResponseType_E responseType);

/******************************************************************************
 * @brief     : Read data via HAL layer
 * @param[in] : length - Number of bytes to read
 * @param[out]: data - Buffer to store received data
 * @return    : 0 if success, -1 if error
 * @note      : Hardware-specific data reception
 *****************************************************************************/
int32_t SdioReadDataHal(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Write data via HAL layer
 * @param[in] : data - Pointer to data buffer, length - Number of bytes, crc16 - CRC16 checksum
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Hardware-specific data transmission
 *****************************************************************************/
int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16);

/******************************************************************************
 * @brief     : Set bus width via HAL layer
 * @param[in] : width - Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures hardware for specified bus width
 *****************************************************************************/
int32_t SdioSetBusWidthHal(uint8_t width);

/******************************************************************************
 * @brief     : Set clock speed via HAL layer
 * @param[in] : speed - Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures hardware clock divider
 *****************************************************************************/
int32_t SdioSetClockSpeedHal(uint32_t speed);

/******************************************************************************
 * @brief     : Check if SDIO is busy
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if busy, 0 if idle
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SdioIsBusy(void);

/******************************************************************************
 * @brief     : Wait for SDIO command completion
 * @param[in] : timeout - Timeout in milliseconds
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SdioWaitCommandComplete(uint32_t timeout);

/******************************************************************************
 * @brief     : Wait for SDIO data transfer completion
 * @param[in] : timeout - Timeout in milliseconds
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SdioWaitDataComplete(uint32_t timeout);

#endif /* SDIO_HAL_H */

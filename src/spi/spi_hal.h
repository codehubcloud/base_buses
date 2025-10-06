#ifndef SPI_HAL_H
#define SPI_HAL_H

#include <stdint.h>
#include "spi.h"

/******************************************************************************
 * @brief     : Enable SPI clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SpiEnableClock(void);

/******************************************************************************
 * @brief     : Configure SPI GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SpiConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable SPI module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SpiEnable(void);

/******************************************************************************
 * @brief     : Check if SPI is ready
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SpiIsReady(void);

/******************************************************************************
 * @brief     : Check if SPI transfer is complete
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if complete, 0 if not complete
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SpiTransferComplete(void);

/******************************************************************************
 * @brief     : Write data to SPI
 * @param[in] : data --Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SpiWriteData(uint8_t data);

/******************************************************************************
 * @brief     : Read data from SPI
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from SPI
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t SpiReadData(void);

/******************************************************************************
 * @brief     : Configure SPI clock speed
 * @param[in] : clockSpeed --Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SpiConfigureClockSpeed(uint32_t clockSpeed);

/******************************************************************************
 * @brief     : Configure SPI mode
 * @param[in] : mode --SPI mode (MASTER or SLAVE)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SpiConfigureMode(SpiMode_E mode);

/******************************************************************************
 * @brief     : Set SPI data format
 * @param[in] : format --Data format (8-bit or 16-bit)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SpiSetDataFormat(SpiDataFormat_E format);

#endif // SPI_HAL_H
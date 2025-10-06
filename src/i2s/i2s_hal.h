#ifndef I2S_HAL_H
#define I2S_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable I2S clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sEnableClock(void);

/******************************************************************************
 * @brief     : Disable I2S clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I2sDisableClock(void);

/******************************************************************************
 * @brief     : Configure I2S GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable I2S module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I2sEnable(void);

/******************************************************************************
 * @brief     : Disable I2S module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I2sDisable(void);

/******************************************************************************
 * @brief     : Set I2S data format
 * @param[in] : dataFormat --Audio data format (16/24/32-bit)
 *              channelMode - Channel mode (stereo/mono)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sSetDataFormat(uint8_t dataFormat, uint8_t channelMode);

/******************************************************************************
 * @brief     : Configure I2S sample rate
 * @param[in] : sampleRate --Desired sample rate in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sConfigureSampleRate(uint32_t sampleRate);

/******************************************************************************
 * @brief     : Write data to I2S interface
 * @param[in] : data --Pointer to data buffer
 *              length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sWrite(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Read data from I2S interface
 * @param[in] : maxLength --Maximum number of bytes to read
 * @param[out]: buffer --Received data
 * @return    : Number of bytes actually read, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sRead(uint8_t* buffer, uint32_t maxLength);

/******************************************************************************
 * @brief     : Configure DMA for I2S
 * @param[in] : enable --1 to enable DMA, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2sConfigureDMA(uint8_t enable);

#endif // I2S_HAL_H

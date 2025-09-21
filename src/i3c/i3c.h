#ifndef I3C_H
#define I3C_H

#include <stdint.h>

#define I3C_DEFAULT_CLOCK_SPEED 12500000

/******************************************************************************
 * @brief     : Initialize I3C peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I3C clock speed and mode
 *****************************************************************************/
int32_t I3cInit(void);

/******************************************************************************
 * @brief     : Send data to I3C device
 * @param[in] : deviceAddr - I3C device address
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes data to I3C device
 *****************************************************************************/
int32_t I3cSendData(uint8_t deviceAddr, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Read data from I3C device
 * @param[in] : deviceAddr - I3C device address
 * @param[in] : buffer - Pointer to buffer to store received data
 * @param[in] : length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads data from I3C device
 *****************************************************************************/
int32_t I3cReadData(uint8_t deviceAddr, uint8_t* buffer, uint16_t length);

/******************************************************************************
 * @brief     : Set I3C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I3C clock prescaler
 *****************************************************************************/
int32_t I3cSetClockSpeed(uint32_t clockSpeed);

/******************************************************************************
 * @brief     : Check if I3C device is present
 * @param[in] : deviceAddr - I3C device address
 * @param[out]: None
 * @return    : 0 if device present, -1 if not present
 * @note      : Performs a simple address check
 *****************************************************************************/
int32_t I3cCheckDevice(uint8_t deviceAddr);

#endif // I3C_H
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define I2C_DEFAULT_CLOCK_SPEED 100000

/******************************************************************************
 * @brief     : Initialize I2C peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I2C clock speed and mode
 *****************************************************************************/
int32_t I2cInit(void);

/******************************************************************************
 * @brief     : Send data to I2C device
 * @param[in] : deviceAddr - I2C device address, regAddr - Register address (can be NULL for direct write), data - Pointer to data buffer to
 *send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes data to I2C device
 *****************************************************************************/
int32_t I2cWriteData(uint8_t deviceAddr, uint8_t* regAddr, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Read data from I2C device
 * @param[in] : deviceAddr - I2C device address, regAddr - Register address (can be NULL for direct read), length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads data from I2C device
 *****************************************************************************/
int32_t I2cReadData(uint8_t deviceAddr, uint8_t* regAddr, uint8_t* buffer, uint16_t length);

/******************************************************************************
 * @brief     : Set I2C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I2C clock prescaler
 *****************************************************************************/
int32_t I2cSetClockSpeed(uint32_t clockSpeed);

/******************************************************************************
 * @brief     : Check if I2C device is present
 * @param[in] : deviceAddr - I2C device address
 * @param[out]: None
 * @return    : 0 if device present, -1 if not present
 * @note      : Performs a simple address check
 *****************************************************************************/
int32_t I2cCheckDevice(uint8_t deviceAddr);

#endif // I2C_H
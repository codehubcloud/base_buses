#ifndef SMBUS_H
#define SMBUS_H

#include <stdint.h>

#define SMBUS_DEFAULT_CLOCK_SPEED 100000

/******************************************************************************
 * @brief     : Initialize SMBus peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SMBus clock speed and mode
 *****************************************************************************/
int32_t SmBusInit(void);

/******************************************************************************
 * @brief     : Send data to SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte, data - Pointer to data buffer to send, length - Number of bytes
 *to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes data to SMBus device
 *****************************************************************************/
int32_t SmBusSendData(uint8_t deviceAddr, uint8_t command, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Read data from SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte, length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads data from SMBus device
 *****************************************************************************/
int32_t SmBusReadData(uint8_t deviceAddr, uint8_t command, uint8_t* buffer, uint16_t length);

/******************************************************************************
 * @brief     : Read single byte from SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte
 * @param[out]: data - Pointer to store received byte
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads a single byte from SMBus device
 *****************************************************************************/
int32_t SmBusReadByte(uint8_t deviceAddr, uint8_t command, uint8_t* data);

/******************************************************************************
 * @brief     : Write single byte to SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte, data - Byte to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes a single byte to SMBus device
 *****************************************************************************/
int32_t SmBusWriteByte(uint8_t deviceAddr, uint8_t command, uint8_t data);

/******************************************************************************
 * @brief     : Set SMBus clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SMBus clock prescaler
 *****************************************************************************/
int32_t SmBusSetClockSpeed(uint32_t clockSpeed);

/******************************************************************************
 * @brief     : Check if SMBus device is present
 * @param[in] : deviceAddr - SMBus device address
 * @param[out]: None
 * @return    : 0 if device present, -1 if not present
 * @note      : Performs a simple address check
 *****************************************************************************/
int32_t SmBusCheckDevice(uint8_t deviceAddr);

#endif // SMBUS_H
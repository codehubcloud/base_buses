#ifndef PMBUS_HAL_H
#define PMBUS_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable PMBus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusEnableClock(void);

/******************************************************************************
 * @brief     : Configure PMBus GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable PMBus module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void PmBusEnable(void);

/******************************************************************************
 * @brief     : Disable PMBus module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void PmBusDisable(void);

/******************************************************************************
 * @brief     : Set PMBus clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusSetClockSpeed(uint32_t clockSpeed);

/******************************************************************************
 * @brief     : Generate PMBus START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusGenerateStart(void);

/******************************************************************************
 * @brief     : Generate PMBus STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusGenerateStop(void);

/******************************************************************************
 * @brief     : Send byte through PMBus
 * @param[in] : data - Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusSendByte(uint8_t data);

/******************************************************************************
 * @brief     : Read byte from PMBus
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from PMBus
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t PmBusReadByteHal(void);

/******************************************************************************
 * @brief     : Check PMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusCheckAck(void);

/******************************************************************************
 * @brief     : Send PMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void PmBusSendAck(void);

/******************************************************************************
 * @brief     : Send PMBus NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void PmBusSendNack(void);

/******************************************************************************
 * @brief     : Write data to PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte, data - Pointer to data buffer, length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusWriteData(uint8_t deviceAddr, uint8_t command, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Read data from PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte, length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusReadData(uint8_t deviceAddr, uint8_t command, uint8_t* buffer, uint16_t length);

/******************************************************************************
 * @brief     : Write single byte to PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte, data - Byte to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusWriteByte(uint8_t deviceAddr, uint8_t command, uint8_t data);

/******************************************************************************
 * @brief     : Read single byte from PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte
 * @param[out]: data - Pointer to store received byte
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusReadByte(uint8_t deviceAddr, uint8_t command, uint8_t* data);

/******************************************************************************
 * @brief     : Send PMBus command only (no data)
 * @param[in] : deviceAddr - PMBus device address, command - Command byte
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t PmBusSendCommand(uint8_t deviceAddr, uint8_t command);

#endif // PMBUS_HAL_H

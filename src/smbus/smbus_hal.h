#ifndef SMBUS_HAL_H
#define SMBUS_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable SMBus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusEnableClock(void);

/******************************************************************************
 * @brief     : Configure SMBus GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable SMBus module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SmBusEnable(void);

/******************************************************************************
 * @brief     : Generate SMBus START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusGenerateStart(void);

/******************************************************************************
 * @brief     : Generate SMBus STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusGenerateStop(void);

/******************************************************************************
 * @brief     : Send byte through SMBus
 * @param[in] : data --Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusSendByte(uint8_t data);

/******************************************************************************
 * @brief     : Read byte from SMBus
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from SMBus
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t SmBusReadByteHal(void);

/******************************************************************************
 * @brief     : Check SMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusCheckAck(void);

/******************************************************************************
 * @brief     : Send SMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SmBusSendAck(void);

/******************************************************************************
 * @brief     : Send SMBus NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SmBusSendNack(void);

/******************************************************************************
 * @brief     : Configure SMBus clock speed
 * @param[in] : clockSpeed --Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SmBusConfigureClockSpeed(uint32_t clockSpeed);

#endif // SMBUS_HAL_H
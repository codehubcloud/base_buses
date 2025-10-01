#ifndef I3C_HAL_H
#define I3C_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable I3C clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cEnableClock(void);

/******************************************************************************
 * @brief     : Configure I3C GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable I3C module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I3cEnable(void);

/******************************************************************************
 * @brief     : Generate I3C START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cGenerateStart(void);

/******************************************************************************
 * @brief     : Generate I3C STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cGenerateStop(void);

/******************************************************************************
 * @brief     : Send byte through I3C
 * @param[in] : data - Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cSendByte(uint8_t data);

/******************************************************************************
 * @brief     : Read byte from I3C
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from I3C
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t I3cReadByte(void);

/******************************************************************************
 * @brief     : Check I3C ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cCheckAck(void);

/******************************************************************************
 * @brief     : Send I3C ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I3cSendAck(void);

/******************************************************************************
 * @brief     : Send I3C NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I3cSendNack(void);

/******************************************************************************
 * @brief     : Configure I3C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I3cConfigureClockSpeed(uint32_t clockSpeed);

#endif // I3C_HAL_H
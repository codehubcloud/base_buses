#ifndef I2C_HAL_H
#define I2C_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable I2C clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cEnableClock(void);

/******************************************************************************
 * @brief     : Configure I2C GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable I2C module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I2cEnable(void);

/******************************************************************************
 * @brief     : Generate I2C START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cGenerateStart(void);

/******************************************************************************
 * @brief     : Generate I2C STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cGenerateStop(void);

/******************************************************************************
 * @brief     : Send byte through I2C
 * @param[in] : data - Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cSendByte(uint8_t data);

/******************************************************************************
 * @brief     : Read byte from I2C
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from I2C
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t I2cReadByte(void);

/******************************************************************************
 * @brief     : Check I2C ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cCheckAck(void);

/******************************************************************************
 * @brief     : Send I2C ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I2cSendAck(void);

/******************************************************************************
 * @brief     : Send I2C NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void I2cSendNack(void);

/******************************************************************************
 * @brief     : Configure I2C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t I2cConfigureClockSpeed(uint32_t clockSpeed);

#endif // I2C_HAL_H
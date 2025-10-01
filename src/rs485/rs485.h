#ifndef RS485_H
#define RS485_H

#include <stdint.h>

#define RS485_DEFAULT_BAUDRATE 9600

/******************************************************************************
 * @brief     : Initialize RS485 peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures RS485 parameters
 *****************************************************************************/
int32_t Rs485Init(void);

/******************************************************************************
 * @brief     : Send data through RS485 interface
 * @param[in] : data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t Rs485SendData(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from RS485 interface
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t Rs485ReceiveData(uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Set RS485 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t Rs485SetBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable RS485 transmitter
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RS485 transmitter and disables receiver
 *****************************************************************************/
int32_t Rs485EnableTransmit(void);

/******************************************************************************
 * @brief     : Enable RS485 receiver
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RS485 receiver and disables transmitter
 *****************************************************************************/
int32_t Rs485EnableReceive(void);

#endif // RS485_H
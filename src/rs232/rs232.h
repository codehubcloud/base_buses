#ifndef RS232_H
#define RS232_H

#include <stdint.h>

#define RS232_DEFAULT_BAUDRATE 9600

/******************************************************************************
 * @brief     : Initialize RS232 peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures RS232 parameters
 *****************************************************************************/
int32_t Rs232Init(void);

/******************************************************************************
 * @brief     : Send data through RS232 interface
 * @param[in] : data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t Rs232SendData(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from RS232 interface
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t Rs232ReceiveData(uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Set RS232 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t Rs232SetBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable RS232 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t Rs232EnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable RS232 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all RS232 interrupts
 *****************************************************************************/
int32_t Rs232DisableInterrupts(void);

#endif // RS232_H
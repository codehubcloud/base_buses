#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_DEFAULT_BAUDRATE 115200

/******************************************************************************
 * @brief     : Initialize UART peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures baud rate, data bits, stop bits and parity
 *****************************************************************************/
int32_t UartInit(void);

/******************************************************************************
 * @brief     : Send data through UART interface
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t UartSendData(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from UART interface
 * @param[in] : buffer - Pointer to buffer to store received data
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t UartReceiveData(uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Set UART baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t UartSetBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable UART interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t UartEnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable UART interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all UART interrupts
 *****************************************************************************/
int32_t UartDisableInterrupts(void);

#endif // UART_H
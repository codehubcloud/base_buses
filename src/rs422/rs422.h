#ifndef RS422_H
#define RS422_H

#include <stdint.h>

/* Standard baud rate definitions */
#define RS422_BAUDRATE_9600    9600
#define RS422_BAUDRATE_19200   19200
#define RS422_BAUDRATE_38400   38400
#define RS422_BAUDRATE_57600   57600
#define RS422_BAUDRATE_115200  115200
#define RS422_DEFAULT_BAUDRATE RS422_BAUDRATE_9600

/* Data bits configuration */
#define RS422_DATA_BITS_7 7
#define RS422_DATA_BITS_8 8

/* Parity configuration */
#define RS422_PARITY_NONE 0
#define RS422_PARITY_ODD  1
#define RS422_PARITY_EVEN 2

/* Stop bits configuration */
#define RS422_STOP_BITS_1 1
#define RS422_STOP_BITS_2 2

/******************************************************************************
 * @brief     : Initialize RS422 peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures RS422 in full-duplex mode with default parameters
 *****************************************************************************/
int32_t Rs422Init(void);

/******************************************************************************
 * @brief     : Deinitialize RS422 peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables RS422 peripheral and releases resources
 *****************************************************************************/
int32_t Rs422Deinit(void);

/******************************************************************************
 * @brief     : Send data through RS422 interface
 * @param[in] : data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function, full-duplex operation, no DE control needed
 *****************************************************************************/
int32_t Rs422SendData(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from RS422 interface
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function, returns immediately if no data available
 *****************************************************************************/
int32_t Rs422ReceiveData(uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Set RS422 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t Rs422SetBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Configure RS422 data format
 * @param[in] : dataBits - Data bits (7 or 8), parity - Parity mode, stopBits - Stop bits (1 or 2)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures frame format for RS422 communication
 *****************************************************************************/
int32_t Rs422ConfigureFormat(uint8_t dataBits, uint8_t parity, uint8_t stopBits);

/******************************************************************************
 * @brief     : Enable RS422 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t Rs422EnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable RS422 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all RS422 interrupts
 *****************************************************************************/
int32_t Rs422DisableInterrupts(void);

#endif // RS422_H

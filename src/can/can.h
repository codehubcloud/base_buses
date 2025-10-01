#ifndef CAN_H
#define CAN_H

#include <stdint.h>

#define CAN_DEFAULT_BAUDRATE 500000

/******************************************************************************
 * @brief     : Initialize CAN peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures CAN baud rate and mode
 *****************************************************************************/
int32_t CanInit(void);

/******************************************************************************
 * @brief     : Send data through CAN interface
 * @param[in] : id - CAN message ID, data - Pointer to data buffer to send, length - Number of bytes to send (0-8)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t CanSendData(uint32_t id, uint8_t* data, uint8_t length);

/******************************************************************************
 * @brief     : Receive data from CAN interface
 * @param[in] : maxLength - Maximum number of bytes to receive (should be 8)
 * @param[out]: id - Received message ID, buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t CanReceiveData(uint32_t* id, uint8_t* buffer, uint8_t maxLength);

/******************************************************************************
 * @brief     : Set CAN baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t CanSetBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable CAN interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t CanEnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable CAN interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all CAN interrupts
 *****************************************************************************/
int32_t CanDisableInterrupts(void);

#endif // CAN_H
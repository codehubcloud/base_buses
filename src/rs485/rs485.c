#include <string.h>
#include "rs485.h"
#include "rs485_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize RS485 peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures RS485 parameters
 *****************************************************************************/
int32_t Rs485Init(void)
{
    int32_t result = 0;

    result = Rs485EnableClock();
    if (result != 0) {
        return -1;
    }

    result = Rs485ConfigureGpio();
    if (result != 0) {
        return -1;
    }

    Rs485SetBaudRate(RS485_DEFAULT_BAUDRATE);
    Rs485Enable();
    Rs485EnableReceive();

    return 0;
}

/******************************************************************************
 * @brief     : Send data through RS485 interface
 * @param[in] : data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t Rs485SendData(uint8_t* data, uint16_t length)
{
    int32_t result = 0;

    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    // Set to transmit mode
    result = Rs485EnableTransmit();
    if (result != 0) {
        return -1;
    }

    // Send data
    for (uint16_t i = 0; i < length; i++) {
        while (Rs485TxBufferEmpty() == 0) {
        }
        Rs485WriteByte(data[i]);
    }

    // Wait for transmission to complete
    while (Rs485TxBufferEmpty() == 0) {
    }

    // Set back to receive mode
    result = Rs485EnableReceive();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive data from RS485 interface
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t Rs485ReceiveData(uint8_t* buffer, uint16_t maxLength)
{
    uint16_t receivedLength = 0;

    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    while (receivedLength < maxLength) {
        if (Rs485RxBufferHasData()) {
            buffer[receivedLength] = Rs485ReadByte();
            receivedLength++;
        } else {
            break;
        }
    }

    return (int32_t)receivedLength;
}

/******************************************************************************
 * @brief     : Set RS485 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t Rs485SetBaudRate(uint32_t baudRate)
{
    if (Rs485ConfigureBaudRate(baudRate) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Enable RS485 transmitter
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RS485 transmitter and disables receiver
 *****************************************************************************/
int32_t Rs485EnableTransmit(void)
{
    Rs485SetTxMode();
    return 0;
}

/******************************************************************************
 * @brief     : Enable RS485 receiver
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RS485 receiver and disables transmitter
 *****************************************************************************/
int32_t Rs485EnableReceive(void)
{
    Rs485SetRxMode();
    return 0;
}
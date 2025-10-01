#include <string.h>
#include "rs232.h"
#include "rs232_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize RS232 peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures RS232 parameters
 *****************************************************************************/
int32_t Rs232Init(void)
{
    int32_t result = 0;

    result = Rs232EnableClock();
    if (result != 0) {
        return -1;
    }

    result = Rs232ConfigureGpio();
    if (result != 0) {
        return -1;
    }

    Rs232SetBaudRate(RS232_DEFAULT_BAUDRATE);
    Rs232Enable();

    return 0;
}

/******************************************************************************
 * @brief     : Send data through RS232 interface
 * @param[in] : data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t Rs232SendData(uint8_t* data, uint16_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        while (Rs232TxBufferEmpty() == 0) {
        }
        Rs232WriteByte(data[i]);
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive data from RS232 interface
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t Rs232ReceiveData(uint8_t* buffer, uint16_t maxLength)
{
    uint16_t receivedLength = 0;

    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    while (receivedLength < maxLength) {
        if (Rs232RxBufferHasData()) {
            buffer[receivedLength] = Rs232ReadByte();
            receivedLength++;
        } else {
            break;
        }
    }

    return (int32_t)receivedLength;
}

/******************************************************************************
 * @brief     : Set RS232 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t Rs232SetBaudRate(uint32_t baudRate)
{
    if (Rs232ConfigureBaudRate(baudRate) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Enable RS232 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t Rs232EnableInterrupts(void)
{
    Rs232EnableRxInterrupt();
    Rs232EnableTxInterrupt();
    Rs232ConfigureNvic();
    return 0;
}

/******************************************************************************
 * @brief     : Disable RS232 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all RS232 interrupts
 *****************************************************************************/
int32_t Rs232DisableInterrupts(void)
{
    Rs232DisableRxInterrupt();
    Rs232DisableTxInterrupt();
    Rs232UpdateNvic();
    return 0;
}
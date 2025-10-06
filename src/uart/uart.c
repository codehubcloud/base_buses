#include <string.h>
#include "securec.h"
#include "uart.h"
#include "uart_hal.h"

/******************************************************************************
 * @brief     : Initialize UART peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures baud rate, data bits, stop bits and parity
 *****************************************************************************/
int32_t UartInit(void)
{
    int32_t result = 0;

    result = UartEnableClock();
    if (result != 0) {
        return -1;
    }

    result = UartConfigureGpio();
    if (result != 0) {
        return -1;
    }

    UartSetBaudRate(UART_DEFAULT_BAUDRATE);
    UartEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Send data through UART interface
 * @param[in] : data --Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t UartSendData(uint8_t* data, uint16_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        while (UartTxBufferEmpty() == 0) {
        }
        UartWriteByte(data[i]);
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive data from UART interface
 * @param[in] : maxLength --Maximum number of bytes to receive
 * @param[out]: buffer --Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t UartReceiveData(uint8_t* buffer, uint16_t maxLength)
{
    uint16_t receivedLength = 0;

    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    while (receivedLength < maxLength) {
        if (UartRxBufferHasData() != 0) {
            buffer[receivedLength] = UartReadByte();
            receivedLength++;
        } else {
            break;
        }
    }

    return (int32_t)receivedLength;
}

/******************************************************************************
 * @brief     : Set UART baud rate
 * @param[in] : baudRate --Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t UartSetBaudRate(uint32_t baudRate)
{
    if (UartConfigureBaudRate(baudRate) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Enable UART interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t UartEnableInterrupts(void)
{
    UartEnableRxInterrupt();
    UartEnableTxInterrupt();
    UartConfigureNvic();
    return 0;
}

/******************************************************************************
 * @brief     : Disable UART interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all UART interrupts
 *****************************************************************************/
int32_t UartDisableInterrupts(void)
{
    UartDisableRxInterrupt();
    UartDisableTxInterrupt();
    UartUpdateNvic();
    return 0;
}
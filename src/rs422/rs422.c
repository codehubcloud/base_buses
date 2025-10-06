#include <string.h>
#include "rs422.h"
#include "rs422_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize RS422 peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures RS422 in full-duplex mode with default parameters
 *****************************************************************************/
int32_t Rs422Init(void)
{
    int32_t result = 0;

    result = Rs422EnableClock();
    if (result != 0) {
        return -1;
    }

    result = Rs422ConfigureGpio();
    if (result != 0) {
        return -1;
    }

    Rs422SetBaudRate(RS422_DEFAULT_BAUDRATE);
    Rs422ConfigureFormat(RS422_DATA_BITS_8, RS422_PARITY_NONE, RS422_STOP_BITS_1);
    Rs422Enable();

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize RS422 peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables RS422 peripheral and releases resources
 *****************************************************************************/
int32_t Rs422Deinit(void)
{
    Rs422Disable();
    Rs422DisableClock();
    return 0;
}

/******************************************************************************
 * @brief     : Send data through RS422 interface
 * @param[in] : data --Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function, full-duplex operation, no DE control needed
 *****************************************************************************/
int32_t Rs422SendData(uint8_t* data, uint16_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        while (Rs422TxBufferEmpty() == 0) {
        }
        Rs422WriteByte(data[i]);
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive data from RS422 interface
 * @param[in] : maxLength --Maximum number of bytes to receive
 * @param[out]: buffer --Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function, returns immediately if no data available
 *****************************************************************************/
int32_t Rs422ReceiveData(uint8_t* buffer, uint16_t maxLength)
{
    uint16_t receivedLength = 0;

    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    while (receivedLength < maxLength) {
        if (Rs422RxBufferHasData()) {
            buffer[receivedLength] = Rs422ReadByte();
            receivedLength++;
        } else {
            break;
        }
    }

    return (int32_t)receivedLength;
}

/******************************************************************************
 * @brief     : Set RS422 baud rate
 * @param[in] : baudRate --Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the baud rate registers
 *****************************************************************************/
int32_t Rs422SetBaudRate(uint32_t baudRate)
{
    if (Rs422ConfigureBaudRate(baudRate) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Configure RS422 data format
 * @param[in] : dataBits --Data bits (7 or 8), parity - Parity mode, stopBits - Stop bits (1 or 2)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures frame format for RS422 communication
 *****************************************************************************/
int32_t Rs422ConfigureFormat(uint8_t dataBits, uint8_t parity, uint8_t stopBits)
{
    if ((dataBits != RS422_DATA_BITS_7) && (dataBits != RS422_DATA_BITS_8)) {
        return -1;
    }

    if ((parity != RS422_PARITY_NONE) && (parity != RS422_PARITY_ODD) && (parity != RS422_PARITY_EVEN)) {
        return -1;
    }

    if ((stopBits != RS422_STOP_BITS_1) && (stopBits != RS422_STOP_BITS_2)) {
        return -1;
    }

    if (Rs422ConfigureDataFormat(dataBits, parity, stopBits) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Enable RS422 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t Rs422EnableInterrupts(void)
{
    Rs422EnableRxInterrupt();
    Rs422EnableTxInterrupt();
    Rs422ConfigureNvic();
    return 0;
}

/******************************************************************************
 * @brief     : Disable RS422 interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all RS422 interrupts
 *****************************************************************************/
int32_t Rs422DisableInterrupts(void)
{
    Rs422DisableRxInterrupt();
    Rs422DisableTxInterrupt();
    Rs422UpdateNvic();
    return 0;
}

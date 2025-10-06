#include <string.h>
#include "securec.h"
#include "spi.h"
#include "spi_hal.h"

/******************************************************************************
 * @brief     : Initialize SPI peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SPI mode, clock speed and data format
 *****************************************************************************/
int32_t SpiInit(void)
{
    int32_t result = 0;

    result = SpiEnableClock();
    if (result != 0) {
        return -1;
    }

    result = SpiConfigureGpio();
    if (result != 0) {
        return -1;
    }

    SpiSetMode(SPI_MODE_MASTER);
    SpiSetClockSpeed(SPI_DEFAULT_CLOCK_SPEED);
    SpiSetDataFormat(SPI_DATA_FORMAT_8BIT);
    SpiEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Send and receive data through SPI interface
 * @param[in] : txData --Pointer to transmit data buffer, length - Number of bytes to transfer
 * @param[out]: rxData --Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that performs full-duplex transfer
 *****************************************************************************/
int32_t SpiTransfer(uint8_t* txData, uint8_t* rxData, uint16_t length)
{
    if ((txData == NULL) || (rxData == NULL) || (length == 0)) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        while (SpiIsReady() == 0) {
        }
        SpiWriteData(txData[i]);
        while (SpiTransferComplete() == 0) {
        }
        rxData[i] = SpiReadData();
    }

    return 0;
}

/******************************************************************************
 * @brief     : Send data through SPI interface
 * @param[in] : data --Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that sends data and ignores received data
 *****************************************************************************/
int32_t SpiSendData(uint8_t* data, uint16_t length)
{
    uint8_t dummyRx;

    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        while (SpiIsReady() == 0) {
        }
        SpiWriteData(data[i]);
        while (SpiTransferComplete() == 0) {
        }
        dummyRx = SpiReadData();
        (void)dummyRx;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive data from SPI interface
 * @param[in] : length --Number of bytes to receive
 * @param[out]: rxData --Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that sends dummy data and receives data
 *****************************************************************************/
int32_t SpiReceiveData(uint8_t* rxData, uint16_t length)
{
    if ((rxData == NULL) || (length == 0)) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        while (SpiIsReady() == 0) {
        }
        SpiWriteData(0xFF);
        while (SpiTransferComplete() == 0) {
        }
        rxData[i] = SpiReadData();
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set SPI clock speed
 * @param[in] : clockSpeed --Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SPI clock prescaler
 *****************************************************************************/
int32_t SpiSetClockSpeed(uint32_t clockSpeed)
{
    if (SpiConfigureClockSpeed(clockSpeed) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Set SPI mode
 * @param[in] : mode --SPI mode (MASTER or SLAVE)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SPI as master or slave
 *****************************************************************************/
int32_t SpiSetMode(SpiMode_E mode)
{
    if (SpiConfigureMode(mode) != 0) {
        return -1;
    }
    return 0;
}
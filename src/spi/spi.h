#ifndef SPI_H
#define SPI_H

#include <stdint.h>

typedef enum { SPI_MODE_MASTER = 0, SPI_MODE_SLAVE } SpiMode_E;

typedef enum { SPI_DATA_FORMAT_8BIT = 0, SPI_DATA_FORMAT_16BIT } SpiDataFormat_E;

#define SPI_DEFAULT_CLOCK_SPEED 1000000

/******************************************************************************
 * @brief     : Initialize SPI peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SPI mode, clock speed and data format
 *****************************************************************************/
int32_t SpiInit(void);

/******************************************************************************
 * @brief     : Send and receive data through SPI interface
 * @param[in] : txData - Pointer to transmit data buffer, length - Number of bytes to transfer
 * @param[out]: rxData - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that performs full-duplex transfer
 *****************************************************************************/
int32_t SpiTransfer(uint8_t* txData, uint8_t* rxData, uint16_t length);

/******************************************************************************
 * @brief     : Send data through SPI interface
 * @param[in] : data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that sends data and ignores received data
 *****************************************************************************/
int32_t SpiSendData(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from SPI interface
 * @param[in] : length - Number of bytes to receive
 * @param[out]: rxData - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that sends dummy data and receives data
 *****************************************************************************/
int32_t SpiReceiveData(uint8_t* rxData, uint16_t length);

/******************************************************************************
 * @brief     : Set SPI clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SPI clock prescaler
 *****************************************************************************/
int32_t SpiSetClockSpeed(uint32_t clockSpeed);

/******************************************************************************
 * @brief     : Set SPI mode
 * @param[in] : mode - SPI mode (MASTER or SLAVE)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SPI as master or slave
 *****************************************************************************/
int32_t SpiSetMode(SpiMode_E mode);

// Hardware abstraction functions

#endif // SPI_H
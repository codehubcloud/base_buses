#ifndef RS422_HAL_H
#define RS422_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable RS422 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs422EnableClock(void);

/******************************************************************************
 * @brief     : Disable RS422 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422DisableClock(void);

/******************************************************************************
 * @brief     : Configure RS422 GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs422ConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable RS422 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422Enable(void);

/******************************************************************************
 * @brief     : Disable RS422 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422Disable(void);

/******************************************************************************
 * @brief     : Check if RS422 TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs422TxBufferEmpty(void);

/******************************************************************************
 * @brief     : Write byte to RS422
 * @param[in] : data - Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422WriteByte(uint8_t data);

/******************************************************************************
 * @brief     : Check if RS422 RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs422RxBufferHasData(void);

/******************************************************************************
 * @brief     : Read byte from RS422
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from RS422
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t Rs422ReadByte(void);

/******************************************************************************
 * @brief     : Configure RS422 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs422ConfigureBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Configure RS422 data format
 * @param[in] : dataBits - Data bits (7 or 8), parity - Parity mode, stopBits - Stop bits (1 or 2)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs422ConfigureDataFormat(uint8_t dataBits, uint8_t parity, uint8_t stopBits);

/******************************************************************************
 * @brief     : Enable RS422 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422EnableRxInterrupt(void);

/******************************************************************************
 * @brief     : Enable RS422 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422EnableTxInterrupt(void);

/******************************************************************************
 * @brief     : Configure NVIC for RS422
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422ConfigureNvic(void);

/******************************************************************************
 * @brief     : Disable RS422 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422DisableRxInterrupt(void);

/******************************************************************************
 * @brief     : Disable RS422 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422DisableTxInterrupt(void);

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs422UpdateNvic(void);

#endif // RS422_HAL_H

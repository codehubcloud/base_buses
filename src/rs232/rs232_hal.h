#ifndef RS232_HAL_H
#define RS232_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable RS232 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs232EnableClock(void);

/******************************************************************************
 * @brief     : Configure RS232 GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs232ConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable RS232 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232Enable(void);

/******************************************************************************
 * @brief     : Check if RS232 TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs232TxBufferEmpty(void);

/******************************************************************************
 * @brief     : Write byte to RS232
 * @param[in] : data - Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232WriteByte(uint8_t data);

/******************************************************************************
 * @brief     : Check if RS232 RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs232RxBufferHasData(void);

/******************************************************************************
 * @brief     : Read byte from RS232
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from RS232
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t Rs232ReadByte(void);

/******************************************************************************
 * @brief     : Configure RS232 baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t Rs232ConfigureBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable RS232 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232EnableRxInterrupt(void);

/******************************************************************************
 * @brief     : Enable RS232 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232EnableTxInterrupt(void);

/******************************************************************************
 * @brief     : Configure NVIC for RS232
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232ConfigureNvic(void);

/******************************************************************************
 * @brief     : Disable RS232 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232DisableRxInterrupt(void);

/******************************************************************************
 * @brief     : Disable RS232 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232DisableTxInterrupt(void);

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void Rs232UpdateNvic(void);

#endif // RS232_HAL_H
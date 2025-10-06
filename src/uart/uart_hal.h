#ifndef UART_HAL_H
#define UART_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable UART clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UartEnableClock(void);

/******************************************************************************
 * @brief     : Configure UART GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UartConfigureGpio(void);

/******************************************************************************
 * @brief     : Enable UART module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartEnable(void);

/******************************************************************************
 * @brief     : Check if UART TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UartTxBufferEmpty(void);

/******************************************************************************
 * @brief     : Write byte to UART
 * @param[in] : data --Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartWriteByte(uint8_t data);

/******************************************************************************
 * @brief     : Check if UART RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UartRxBufferHasData(void);

/******************************************************************************
 * @brief     : Read byte from UART
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from UART
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t UartReadByte(void);

/******************************************************************************
 * @brief     : Configure UART baud rate
 * @param[in] : baudRate --Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UartConfigureBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable UART RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartEnableRxInterrupt(void);

/******************************************************************************
 * @brief     : Enable UART TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartEnableTxInterrupt(void);

/******************************************************************************
 * @brief     : Configure NVIC for UART
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartConfigureNvic(void);

/******************************************************************************
 * @brief     : Disable UART RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartDisableRxInterrupt(void);

/******************************************************************************
 * @brief     : Disable UART TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartDisableTxInterrupt(void);

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void UartUpdateNvic(void);

#endif // UART_HAL_H
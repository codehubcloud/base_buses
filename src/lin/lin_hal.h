#ifndef LIN_HAL_H
#define LIN_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable LIN UART clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinEnableClock(void);

/******************************************************************************
 * @brief     : Configure LIN GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinConfigureGpio(void);

/******************************************************************************
 * @brief     : Configure LIN UART peripheral
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinConfigureUart(uint32_t baudRate);

/******************************************************************************
 * @brief     : Enable LIN UART module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinEnable(void);

/******************************************************************************
 * @brief     : Disable LIN UART module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinDisable(void);

/******************************************************************************
 * @brief     : Send LIN break field (13-bit dominant)
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific break generation
 *****************************************************************************/
int32_t LinHalSendBreak(void);

/******************************************************************************
 * @brief     : Write byte to LIN UART
 * @param[in] : data - Byte to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinWriteByte(uint8_t data);

/******************************************************************************
 * @brief     : Read byte from LIN UART
 * @param[in] : None
 * @param[out]: data - Pointer to store read byte
 * @return    : 0 if success, -1 if error/timeout
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinReadByte(uint8_t* data);

/******************************************************************************
 * @brief     : Check if LIN TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinTxBufferEmpty(void);

/******************************************************************************
 * @brief     : Check if LIN RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t LinRxBufferHasData(void);

/******************************************************************************
 * @brief     : Flush LIN RX buffer
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinFlushRxBuffer(void);

/******************************************************************************
 * @brief     : Flush LIN TX buffer
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinFlushTxBuffer(void);

/******************************************************************************
 * @brief     : Set LIN break length (for platforms supporting variable break)
 * @param[in] : breakLength - Break length in bit times
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation, LIN uses 13-bit break
 *****************************************************************************/
int32_t LinSetBreakLength(uint8_t breakLength);

/******************************************************************************
 * @brief     : Enable LIN RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinEnableRxInterrupt(void);

/******************************************************************************
 * @brief     : Enable LIN TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinEnableTxInterrupt(void);

/******************************************************************************
 * @brief     : Disable LIN RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinDisableRxInterrupt(void);

/******************************************************************************
 * @brief     : Disable LIN TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinDisableTxInterrupt(void);

/******************************************************************************
 * @brief     : Configure NVIC for LIN UART
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void LinConfigureNvic(void);

/******************************************************************************
 * @brief     : Delay in microseconds
 * @param[in] : us - Microseconds to delay
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation for precise timing
 *****************************************************************************/
void LinDelayUs(uint32_t us);

#endif // LIN_HAL_H

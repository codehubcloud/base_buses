#ifndef SWD_HAL_H
#define SWD_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable SWD clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SwdEnableClock(void);

/******************************************************************************
 * @brief     : Configure SWD GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t SwdConfigureGpio(void);

/******************************************************************************
 * @brief     : Release SWD GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void SwdReleaseGpio(void);

/******************************************************************************
 * @brief     : Send JTAG-to-SWD switching sequence
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends 16-bit sequence 0xE79E to switch from JTAG to SWD
 *****************************************************************************/
int32_t SwdJtagToSwdSequence(void);

/******************************************************************************
 * @brief     : Perform SWD line reset
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Sends 50+ clock cycles with SWDIO high
 *****************************************************************************/
void SwdLineReset(void);

/******************************************************************************
 * @brief     : Send idle cycles
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Sends at least 8 clock cycles with SWDIO low
 *****************************************************************************/
void SwdIdle(void);

/******************************************************************************
 * @brief     : Perform turnaround (change SWDIO direction)
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : One clock cycle with SWDIO as input
 *****************************************************************************/
void SwdTurnaround(void);

/******************************************************************************
 * @brief     : Write SWD request header
 * @param[in] : request - 8-bit request value
 * @param[out]: None
 * @return    : None
 * @note      : Sends request LSB first
 *****************************************************************************/
void SwdWriteRequest(uint8_t request);

/******************************************************************************
 * @brief     : Read SWD ACK response
 * @param[in] : None
 * @param[out]: None
 * @return    : 3-bit ACK value
 * @note      : Reads ACK LSB first
 *****************************************************************************/
uint8_t SwdReadAck(void);

/******************************************************************************
 * @brief     : Write 32-bit data
 * @param[in] : data - Data to write
 * @param[out]: None
 * @return    : None
 * @note      : Sends data LSB first
 *****************************************************************************/
void SwdWriteData(uint32_t data);

/******************************************************************************
 * @brief     : Read 32-bit data
 * @param[in] : None
 * @param[out]: None
 * @return    : Data read
 * @note      : Reads data LSB first
 *****************************************************************************/
uint32_t SwdReadData(void);

/******************************************************************************
 * @brief     : Write parity bit
 * @param[in] : parity - Parity bit value (0 or 1)
 * @param[out]: None
 * @return    : None
 * @note      : Sends single parity bit
 *****************************************************************************/
void SwdWriteParity(uint8_t parity);

/******************************************************************************
 * @brief     : Read parity bit
 * @param[in] : None
 * @param[out]: None
 * @return    : Parity bit value (0 or 1)
 * @note      : Reads single parity bit
 *****************************************************************************/
uint8_t SwdReadParity(void);

#endif // SWD_HAL_H

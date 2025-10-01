#ifndef JTAG_HAL_H
#define JTAG_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Configure JTAG GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t JtagConfigureGpio(void);

/******************************************************************************
 * @brief     : Release JTAG GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t JtagReleaseGpio(void);

/******************************************************************************
 * @brief     : Enable JTAG clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t JtagEnableClock(void);

/******************************************************************************
 * @brief     : Set TCK (Test Clock) pin state
 * @param[in] : state - Pin state (0=low, 1=high)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void JtagSetTck(uint8_t state);

/******************************************************************************
 * @brief     : Set TMS (Test Mode Select) pin state
 * @param[in] : state - Pin state (0=low, 1=high)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void JtagSetTms(uint8_t state);

/******************************************************************************
 * @brief     : Set TDI (Test Data In) pin state
 * @param[in] : state - Pin state (0=low, 1=high)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void JtagSetTdi(uint8_t state);

/******************************************************************************
 * @brief     : Get TDO (Test Data Out) pin state
 * @param[in] : None
 * @param[out]: None
 * @return    : Pin state (0=low, 1=high)
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t JtagGetTdo(void);

/******************************************************************************
 * @brief     : Set TRST (Test Reset) pin state
 * @param[in] : state - Pin state (0=low/reset, 1=high/inactive)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required, optional pin
 *****************************************************************************/
void JtagSetTrst(uint8_t state);

/******************************************************************************
 * @brief     : Microsecond delay for JTAG timing
 * @param[in] : us - Delay in microseconds
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void JtagDelayUs(uint32_t us);

/******************************************************************************
 * @brief     : Configure JTAG clock speed
 * @param[in] : frequency - Clock frequency in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t JtagConfigureClockSpeed(uint32_t frequency);

#endif // JTAG_HAL_H

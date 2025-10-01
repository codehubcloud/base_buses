#ifndef APB_HAL_H
#define APB_HAL_H

#include <stdint.h>
#include "apb.h"

/******************************************************************************
 * @brief     : Enable APB bus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbEnableClock(void);

/******************************************************************************
 * @brief     : Configure APB bus parameters
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbConfigureBus(void);

/******************************************************************************
 * @brief     : Enable APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbBusEnable(void);

/******************************************************************************
 * @brief     : Disable APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbBusDisable(void);

/******************************************************************************
 * @brief     : Check if APB bus is ready
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbIsBusReady(void);

/******************************************************************************
 * @brief     : Check if APB transfer is complete
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if complete, 0 if not complete
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbIsTransferComplete(void);

/******************************************************************************
 * @brief     : Set APB peripheral address (PADDR)
 * @param[in] : address - Peripheral address
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetAddress(uint32_t address);

/******************************************************************************
 * @brief     : Set APB protection signals (PPROT)
 * @param[in] : prot - Protection signals
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetProtection(uint8_t prot);

/******************************************************************************
 * @brief     : Set APB transfer type (read/write)
 * @param[in] : type - Transfer type
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetTransferType(ApbTransferType_E type);

/******************************************************************************
 * @brief     : Set APB write data (PWDATA)
 * @param[in] : data - Data to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetWriteData(uint32_t data);

/******************************************************************************
 * @brief     : Get APB read data (PRDATA)
 * @param[in] : None
 * @param[out]: None
 * @return    : Read data value
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint32_t ApbGetReadData(void);

/******************************************************************************
 * @brief     : Assert PENABLE signal
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbAssertPenable(void);

/******************************************************************************
 * @brief     : Deassert PENABLE signal
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbDeassertPenable(void);

/******************************************************************************
 * @brief     : Set APB byte strobe (PSTRB) for APB4
 * @param[in] : strobe - Byte strobe mask
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation, APB4 only
 *****************************************************************************/
void ApbSetStrobe(uint8_t strobe);

/******************************************************************************
 * @brief     : Get APB slave error status (PSLVERR)
 * @param[in] : None
 * @param[out]: None
 * @return    : APB_PSLVERR_OK or APB_PSLVERR_ERROR
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbGetSlaveError(void);

/******************************************************************************
 * @brief     : Delay for specified microseconds
 * @param[in] : us - Microseconds to delay
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbDelayUs(uint32_t us);

#endif // APB_HAL_H

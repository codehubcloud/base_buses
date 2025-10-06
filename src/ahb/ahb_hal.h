#ifndef AHB_HAL_H
#define AHB_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : Enable AHB bus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t AhbEnableClock(void);

/******************************************************************************
 * @brief     : Disable AHB bus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbDisableClock(void);

/******************************************************************************
 * @brief     : Configure AHB bus matrix
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t AhbConfigureBusMatrix(void);

/******************************************************************************
 * @brief     : Reset AHB controller
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbResetController(void);

/******************************************************************************
 * @brief     : Enable AHB master interface
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbEnableMaster(void);

/******************************************************************************
 * @brief     : Disable AHB master interface
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbDisableMaster(void);

/******************************************************************************
 * @brief     : Set AHB transfer address
 * @param[in] : address --Target address
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbSetAddress(uint32_t address);

/******************************************************************************
 * @brief     : Set AHB transfer type
 * @param[in] : transferType --Transfer type (HTRANS)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbSetTransferType(uint8_t transferType);

/******************************************************************************
 * @brief     : Set AHB burst type
 * @param[in] : burstType --Burst type (HBURST)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbSetBurstType(uint8_t burstType);

/******************************************************************************
 * @brief     : Set AHB transfer size
 * @param[in] : transferSize --Transfer size (HSIZE)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbSetTransferSize(uint8_t transferSize);

/******************************************************************************
 * @brief     : Set AHB write direction
 * @param[in] : write --1 for write, 0 for read
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbSetWrite(uint8_t write);

/******************************************************************************
 * @brief     : Set AHB protection control
 * @param[in] : protection --Protection control signals (HPROT)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbSetProtection(uint8_t protection);

/******************************************************************************
 * @brief     : Write data to AHB bus
 * @param[in] : data --Data to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void AhbWriteData(uint32_t data);

/******************************************************************************
 * @brief     : Read data from AHB bus
 * @param[in] : None
 * @param[out]: None
 * @return    : Data read from bus
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint32_t AhbReadData(void);

/******************************************************************************
 * @brief     : Get AHB response signal
 * @param[in] : None
 * @param[out]: None
 * @return    : Response type (HRESP)
 * @note      : Platform-specific implementation required
 *****************************************************************************/
uint8_t AhbGetResponse(void);

/******************************************************************************
 * @brief     : Check if AHB bus is ready
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t AhbCheckReady(void);

/******************************************************************************
 * @brief     : Configure AHB master priority
 * @param[in] : priority --Priority level
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t AhbConfigurePriority(uint8_t priority);

#endif // AHB_HAL_H

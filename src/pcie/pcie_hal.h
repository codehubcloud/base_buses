/**
 * @file pcie_hal.h
 * * @brief PCIe Hardware Abstraction Layer interface
 * @details Defines hardware-specific interface for PCIe operations
 */

#ifndef PCIE_HAL_H
#define PCIE_HAL_H

#include "pcie.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * @brief      : Initialize PCIe hardware
 * @param[in]  : config --Pointer to PCIe configuration structure
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalInit(const PcieConfig* config);

/******************************************************************************
 * @brief      : Deinitialize PCIe hardware
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalDeinit(void);

/******************************************************************************
 * @brief      : Read from PCIe configuration space (HAL level)
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset size --Size of data to read
 * @param[out] : data --Pointer to store read data
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size);

/******************************************************************************
 * @brief      : Write to PCIe configuration space (HAL level)
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset data --Data to write size --Size of data to write
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size);

/******************************************************************************
 * @brief      : Send TLP packet (HAL level)
 * @param[in]  : packet --Pointer to TLP packet
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalSendTlp(const PcieTlpPacket* packet);

/******************************************************************************
 * @brief      : Receive TLP packet (HAL level)
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] : packet --Pointer to store received TLP packet
 * @return     : 0 on success, -1 on error or timeout
 * @note       :
 *****************************************************************************/
int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs);

/******************************************************************************
 * @brief      : Get PCIe link status (HAL level)
 * @param[in]  :
 * @param[out] : status --Pointer to link status structure
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalGetLinkStatus(PcieLinkStatus* status);

/******************************************************************************
 * @brief      : Read from PCIe memory-mapped region (HAL level)
 * @param[in]  : address --Physical address size --Number of bytes to read
 * @param[out] : data --Buffer to store read data
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size);

/******************************************************************************
 * @brief      : Write to PCIe memory-mapped region (HAL level)
 * @param[in]  : address --Physical address data --Data to write size --Number of bytes to write
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size);

/******************************************************************************
 * @brief      : Enable PCIe interrupts (HAL level)
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalEnableInterrupts(void);

/******************************************************************************
 * @brief      : Disable PCIe interrupts (HAL level)
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieHalDisableInterrupts(void);

#ifdef __cplusplus
}
#endif

#endif /* PCIE_HAL_H */

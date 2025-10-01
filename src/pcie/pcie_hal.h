/**
 * @file pcie_hal.h
 * @brief PCIe Hardware Abstraction Layer interface
 * @details Defines hardware-specific interface for PCIe operations
 */

#ifndef PCIE_HAL_H
#define PCIE_HAL_H

#include "pcie.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize PCIe hardware
 * @param config Pointer to PCIe configuration structure
 * @return 0 on success, -1 on error
 */
int PcieHalInit(const PcieConfig* config);

/**
 * @brief Deinitialize PCIe hardware
 * @return 0 on success, -1 on error
 */
int PcieHalDeinit(void);

/**
 * @brief Read from PCIe configuration space (HAL level)
 * @param bus Bus number
 * @param device Device number
 * @param function Function number
 * @param offset Register offset
 * @param data Pointer to store read data
 * @param size Size of data to read
 * @return 0 on success, -1 on error
 */
int PcieHalConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size);

/**
 * @brief Write to PCIe configuration space (HAL level)
 * @param bus Bus number
 * @param device Device number
 * @param function Function number
 * @param offset Register offset
 * @param data Data to write
 * @param size Size of data to write
 * @return 0 on success, -1 on error
 */
int PcieHalConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size);

/**
 * @brief Send TLP packet (HAL level)
 * @param packet Pointer to TLP packet
 * @return 0 on success, -1 on error
 */
int PcieHalSendTlp(const PcieTlpPacket* packet);

/**
 * @brief Receive TLP packet (HAL level)
 * @param packet Pointer to store received TLP packet
 * @param timeoutMs Timeout in milliseconds
 * @return 0 on success, -1 on error or timeout
 */
int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs);

/**
 * @brief Get PCIe link status (HAL level)
 * @param status Pointer to link status structure
 * @return 0 on success, -1 on error
 */
int PcieHalGetLinkStatus(PcieLinkStatus* status);

/**
 * @brief Read from PCIe memory-mapped region (HAL level)
 * @param address Physical address
 * @param data Buffer to store read data
 * @param size Number of bytes to read
 * @return 0 on success, -1 on error
 */
int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size);

/**
 * @brief Write to PCIe memory-mapped region (HAL level)
 * @param address Physical address
 * @param data Data to write
 * @param size Number of bytes to write
 * @return 0 on success, -1 on error
 */
int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size);

/**
 * @brief Enable PCIe interrupts (HAL level)
 * @return 0 on success, -1 on error
 */
int PcieHalEnableInterrupts(void);

/**
 * @brief Disable PCIe interrupts (HAL level)
 * @return 0 on success, -1 on error
 */
int PcieHalDisableInterrupts(void);

#ifdef __cplusplus
}
#endif

#endif /* PCIE_HAL_H */

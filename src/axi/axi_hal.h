/**
 * @file axi_hal.h
 * @brief AXI Hardware Abstraction Layer interface
 * @details Platform-specific hardware interface for AXI bus operations
 */

#ifndef AXI_HAL_H
#define AXI_HAL_H

#include <stdint.h>
#include <stddef.h>
#include "axi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup AXI_HAL_Functions AXI HAL Function Declarations
 * @{
 */

/**
 * @brief Initialize AXI hardware interface
 * @param[in] config Configuration structure pointer
 * @return 0 on success, -1 on failure
 */
int AxiHalInit(const AxiConfig *config);

/**
 * @brief Deinitialize AXI hardware interface
 * @return 0 on success, -1 on failure
 */
int AxiHalDeinit(void);

/**
 * @brief Write to AXI write address channel
 * @param[in] writeAddr Write address channel structure
 * @return 0 on success, -1 on failure
 */
int AxiHalWriteAddress(const AxiWriteAddress *writeAddr);

/**
 * @brief Write to AXI write data channel
 * @param[in] writeData Write data channel structure
 * @param[in] dataWidth Data width in bytes
 * @return 0 on success, -1 on failure
 */
int AxiHalWriteData(const AxiWriteData *writeData, uint8_t dataWidth);

/**
 * @brief Read from AXI write response channel
 * @param[out] writeResp Write response channel structure
 * @return 0 on success, -1 on failure
 */
int AxiHalReadWriteResponse(AxiWriteResponse *writeResp);

/**
 * @brief Write to AXI read address channel
 * @param[in] readAddr Read address channel structure
 * @return 0 on success, -1 on failure
 */
int AxiHalReadAddress(const AxiReadAddress *readAddr);

/**
 * @brief Read from AXI read data channel
 * @param[out] readData Read data channel structure
 * @param[in] dataWidth Data width in bytes
 * @return 0 on success, -1 on failure
 */
int AxiHalReadData(AxiReadData *readData, uint8_t dataWidth);

/**
 * @brief Send AXI4-Stream data
 * @param[in] streamData Stream data structure
 * @return 0 on success, -1 on failure
 */
int AxiHalStreamSend(const AxiStreamData *streamData);

/**
 * @brief Receive AXI4-Stream data
 * @param[out] streamData Stream data structure
 * @param[in] maxLength Maximum length to receive
 * @return Number of bytes received on success, -1 on failure
 */
int AxiHalStreamReceive(AxiStreamData *streamData, size_t maxLength);

/**
 * @brief Check if AXI bus is busy
 * @return 1 if busy, 0 if idle, -1 on error
 */
int AxiHalIsBusy(void);

/**
 * @brief Wait for AXI transaction completion
 * @param[in] timeoutMs Timeout in milliseconds
 * @return 0 on success, -1 on timeout or error
 */
int AxiHalWaitComplete(uint32_t timeoutMs);

/**
 * @brief Set AXI Quality of Service register
 * @param[in] qosValue QoS value (0-15)
 * @return 0 on success, -1 on failure
 */
int AxiHalSetQoS(uint8_t qosValue);

/**
 * @brief Reset AXI interface
 * @return 0 on success, -1 on failure
 */
int AxiHalReset(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AXI_HAL_H */

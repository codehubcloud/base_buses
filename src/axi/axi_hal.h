/**
 * @file axi_hal.h
 * * @brief AXI Hardware Abstraction Layer interface
 * @details Platform-specific hardware interface for AXI bus operations
 */

#ifndef AXI_HAL_H
#define AXI_HAL_H

#include <stddef.h>
#include <stdint.h>
#include "axi.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup AXI_HAL_Functions AXI HAL Function Declarations
 * @{
 */

/******************************************************************************
 * @brief      : Initialize AXI hardware interface
 * @param[in]  : config --Configuration structure pointer
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalInit(const AxiConfig* config);

/******************************************************************************
 * @brief      : Deinitialize AXI hardware interface
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalDeinit(void);

/******************************************************************************
 * @brief      : Write to AXI write address channel
 * @param[in]  : writeAddr --Write address channel structure
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalWriteAddress(const AxiWriteAddress* writeAddr);

/******************************************************************************
 * @brief      : Write to AXI write data channel
 * @param[in]  : writeData --Write data channel structure dataWidth --Data width in bytes
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalWriteData(const AxiWriteData* writeData, uint8_t dataWidth);

/******************************************************************************
 * @brief      : Read from AXI write response channel
 * @param[in]  :
 * @param[out] : writeResp --Write response channel structure
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalReadWriteResponse(AxiWriteResponse* writeResp);

/******************************************************************************
 * @brief      : Write to AXI read address channel
 * @param[in]  : readAddr --Read address channel structure
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalReadAddress(const AxiReadAddress* readAddr);

/******************************************************************************
 * @brief      : Read from AXI read data channel
 * @param[in]  : dataWidth --Data width in bytes
 * @param[out] : readData --Read data channel structure
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalReadData(AxiReadData* readData, uint8_t dataWidth);

/******************************************************************************
 * @brief      : Send AXI4-Stream data
 * @param[in]  : streamData --Stream data structure
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalStreamSend(const AxiStreamData* streamData);

/******************************************************************************
 * @brief      : Receive AXI4-Stream data
 * @param[in]  : maxLength --Maximum length to receive
 * @param[out] : streamData --Stream data structure
 * @return     : Number of bytes received on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalStreamReceive(AxiStreamData* streamData, size_t maxLength);

/******************************************************************************
 * @brief      : Check if AXI bus is busy
 * @param[in]  :
 * @param[out] :
 * @return     : 1 if busy, 0 if idle, -1 on error
 * @note       :
 *****************************************************************************/
int AxiHalIsBusy(void);

/******************************************************************************
 * @brief      : Wait for AXI transaction completion
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] :
 * @return     : 0 on success, -1 on timeout or error
 * @note       :
 *****************************************************************************/
int AxiHalWaitComplete(uint32_t timeoutMs);

/******************************************************************************
 * @brief      : Set AXI Quality of Service register
 * @param[in]  : qosValue --QoS value (0-15)
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalSetQoS(uint8_t qosValue);

/******************************************************************************
 * @brief      : Reset AXI interface
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       :
 *****************************************************************************/
int AxiHalReset(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AXI_HAL_H */

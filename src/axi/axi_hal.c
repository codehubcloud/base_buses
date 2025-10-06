/**
 * @file axi_hal.c
 * @brief AXI Hardware Abstraction Layer implementation
 * @details Multi-platform HAL for AXI bus operations supporting:
 *          - STM32F4 (memory-mapped AXI via pointer access)
 *          - STM32F1 (not supported - no AXI hardware)
 *          - ESP32 (not supported - no AXI hardware)
 *          - Linux (simulated with memory mapping)
 */

#include <string.h>
#include "../platform_config.h"
#include "axi.h"
#include "axi_hal.h"


#if defined(PLATFORM_STM32F4)
/* STM32F4 Platform - Memory-mapped AXI peripheral access */

#include "stm32f4xx.h"

/** Base address for AXI interconnect (example: external memory controller) */
#define AXI_BASE_ADDRESS 0xA0000000

/** AXI register offsets */
#define AXI_AWADDR_OFFSET 0x00
#define AXI_AWLEN_OFFSET 0x04
#define AXI_AWSIZE_OFFSET 0x08
#define AXI_WDATA_OFFSET 0x20
#define AXI_WSTRB_OFFSET 0x24
#define AXI_BRESP_OFFSET 0x30
#define AXI_ARADDR_OFFSET 0x40
#define AXI_ARLEN_OFFSET 0x44
#define AXI_ARSIZE_OFFSET 0x48
#define AXI_RDATA_OFFSET 0x60
#define AXI_RRESP_OFFSET 0x64
#define AXI_STATUS_OFFSET 0x80
#define AXI_QOS_OFFSET 0x84

/** Status register bits */
#define AXI_STATUS_BUSY (1 << 0)
#define AXI_STATUS_ERROR (1 << 1)

static volatile uint32_t* g_axiBase = NULL;
static AxiConfig g_axiHalConfig = {0};

/******************************************************************************
 * @brief      : Initialize AXI HAL layer for STM32F4 platform
 * @param[in]  : config --Pointer to AXI configuration structure
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Enables DMA clocks and initializes AXI base address
 *****************************************************************************/
int AxiHalInit(const AxiConfig* config)
{
    if (config == NULL) {
        return -1;
    }

    g_axiBase = (volatile uint32_t*)AXI_BASE_ADDRESS;
    if (memcpy_s(&g_axiHalConfig, sizeof(AxiConfig), config, sizeof(AxiConfig)) != EOK) {
        return -1;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

    g_axiBase[AXI_STATUS_OFFSET / 4] = 0;

    return 0;
}

/******************************************************************************
 * @brief      : Deinitialize AXI HAL layer for STM32F4 platform
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Clears AXI status register and releases base address pointer
 *****************************************************************************/
int AxiHalDeinit(void)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    g_axiBase[AXI_STATUS_OFFSET / 4] = 0;
    g_axiBase = NULL;

    return 0;
}

/******************************************************************************
 * @brief      : Write to AXI write address channel
 * @param[in]  : writeAddr --Write address channel structure containing address, burst length and size
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Configures AXI write address registers (AWADDR, AWLEN, AWSIZE)
 *****************************************************************************/
int AxiHalWriteAddress(const AxiWriteAddress* writeAddr)
{
    if (g_axiBase == NULL || writeAddr == NULL) {
        return -1;
    }

    g_axiBase[AXI_AWADDR_OFFSET / 4] = writeAddr->address;
    g_axiBase[AXI_AWLEN_OFFSET / 4] = writeAddr->burstLength;
    g_axiBase[AXI_AWSIZE_OFFSET / 4] = writeAddr->burstSize;

    return 0;
}

/******************************************************************************
 * @brief      : Write to AXI write data channel
 * @param[in]  : writeData --Write data channel structure containing data and strobe signals
                dataWidth --Data width in bytes
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Writes data to WDATA registers, sets WSTRB to 0xFFFFFFFF if strobe is NULL
 *****************************************************************************/
int AxiHalWriteData(const AxiWriteData* writeData, uint8_t dataWidth)
{
    if (g_axiBase == NULL || writeData == NULL || writeData->data == NULL) {
        return -1;
    }

    for (uint8_t i = 0; i < dataWidth / 4; i++) {
        g_axiBase[(AXI_WDATA_OFFSET / 4) + i] = ((uint32_t*)writeData->data)[i];
    }

    if (writeData->strobe != NULL) {
        g_axiBase[AXI_WSTRB_OFFSET / 4] = *(uint32_t*)writeData->strobe;
    } else {
        g_axiBase[AXI_WSTRB_OFFSET / 4] = 0xFFFFFFFF;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Read from AXI write response channel
 * @param[in]  :
 * @param[out] : writeResp --Write response channel structure to store response and ID
 * @return     : 0 on success, -1 on failure or timeout
 * @note       : Waits for transaction completion, reads BRESP register for response type and ID
 *****************************************************************************/
int AxiHalReadWriteResponse(AxiWriteResponse* writeResp)
{
    if (g_axiBase == NULL || writeResp == NULL) {
        return -1;
    }

    uint32_t timeout = g_axiHalConfig.timeout * 1000;
    while ((g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) && timeout--) {
        for (volatile int i = 0; i < 100; i++) {
        }
    }

    if (timeout == 0) {
        return -1;
    }

    writeResp->response = (AxiResponseType)(g_axiBase[AXI_BRESP_OFFSET / 4] & 0x03);
    writeResp->id = (uint16_t)((g_axiBase[AXI_BRESP_OFFSET / 4] >> 16) & 0xFFFF);

    return 0;
}

/******************************************************************************
 * @brief      : Write to AXI read address channel
 * @param[in]  : readAddr --Read address channel structure containing address, burst length and size
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Configures AXI read address registers (ARADDR, ARLEN, ARSIZE)
 *****************************************************************************/
int AxiHalReadAddress(const AxiReadAddress* readAddr)
{
    if (g_axiBase == NULL || readAddr == NULL) {
        return -1;
    }

    g_axiBase[AXI_ARADDR_OFFSET / 4] = readAddr->address;
    g_axiBase[AXI_ARLEN_OFFSET / 4] = readAddr->burstLength;
    g_axiBase[AXI_ARSIZE_OFFSET / 4] = readAddr->burstSize;

    return 0;
}

/******************************************************************************
 * @brief      : Read from AXI read data channel
 * @param[in]  : dataWidth --Data width in bytes
 * @param[out] : readData --Read data channel structure to store data, response and ID
 * @return     : 0 on success, -1 on failure or timeout
 * @note       : Waits for transaction completion, reads RDATA registers and RRESP for response
 *****************************************************************************/
int AxiHalReadData(AxiReadData* readData, uint8_t dataWidth)
{
    if (g_axiBase == NULL || readData == NULL || readData->data == NULL) {
        return -1;
    }

    uint32_t timeout = g_axiHalConfig.timeout * 1000;
    while ((g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) && timeout--) {
        for (volatile int i = 0; i < 100; i++) {
        }
    }

    if (timeout == 0) {
        return -1;
    }

    for (uint8_t i = 0; i < dataWidth / 4; i++) {
        ((uint32_t*)readData->data)[i] = g_axiBase[(AXI_RDATA_OFFSET / 4) + i];
    }

    readData->response = (AxiResponseType)(g_axiBase[AXI_RRESP_OFFSET / 4] & 0x03);
    readData->id = (uint16_t)((g_axiBase[AXI_RRESP_OFFSET / 4] >> 16) & 0xFFFF);

    return 0;
}

/******************************************************************************
 * @brief      : Send AXI4-Stream data
 * @param[in]  : streamData --Stream data structure
 * @param[out] :
 * @return     : -1 (not implemented on STM32F4)
 * @note       : AXI4-Stream interface not implemented for this platform
 *****************************************************************************/
int AxiHalStreamSend(const AxiStreamData* streamData)
{
    if (g_axiBase == NULL || streamData == NULL) {
        return -1;
    }

    return -1;
}

/******************************************************************************
 * @brief      : Receive AXI4-Stream data
 * @param[in]  : maxLength --Maximum length to receive
 * @param[out] : streamData --Stream data structure
 * @return     : -1 (not implemented on STM32F4)
 * @note       : AXI4-Stream interface not implemented for this platform
 *****************************************************************************/
int AxiHalStreamReceive(AxiStreamData* streamData, size_t maxLength)
{
    if (g_axiBase == NULL || streamData == NULL) {
        return -1;
    }

    return -1;
}

/******************************************************************************
 * @brief      : Check if AXI bus is busy
 * @param[in]  :
 * @param[out] :
 * @return     : 1 if busy, 0 if idle, -1 on error
 * @note       : Reads AXI_STATUS_BUSY bit from status register
 *****************************************************************************/
int AxiHalIsBusy(void)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    return (g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) ? 1 : 0;
}

/******************************************************************************
 * @brief      : Wait for AXI transaction completion
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] :
 * @return     : 0 on success, -1 on timeout or error
 * @note       : Polls status register until busy flag clears or timeout expires
 *****************************************************************************/
int AxiHalWaitComplete(uint32_t timeoutMs)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    uint32_t timeout = timeoutMs * 1000;
    while ((g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) && timeout--) {
        for (volatile int i = 0; i < 100; i++) {
        }
    }

    return (timeout == 0) ? -1 : 0;
}

/******************************************************************************
 * @brief      : Set AXI Quality of Service register
 * @param[in]  : qosValue --QoS value (0-15)
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Writes lower 4 bits of qosValue to AXI QoS register
 *****************************************************************************/
int AxiHalSetQoS(uint8_t qosValue)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    g_axiBase[AXI_QOS_OFFSET / 4] = qosValue & 0x0F;
    return 0;
}

/******************************************************************************
 * @brief      : Reset AXI interface
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Clears AXI status register to reset interface state
 *****************************************************************************/
int AxiHalReset(void)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    g_axiBase[AXI_STATUS_OFFSET / 4] = 0;
    return 0;
}

#elif defined(PLATFORM_STM32F1)
/* STM32F1 Platform - No AXI hardware support */

/******************************************************************************
 * @brief      : Initialize AXI HAL layer for STM32F1 platform
 * @param[in]  : config --Pointer to AXI configuration structure
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalInit(const AxiConfig* config)
{
    (void)config;
    return -1;
}

/******************************************************************************
 * @brief      : Deinitialize AXI HAL layer for STM32F1 platform
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalDeinit(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Write to AXI write address channel
 * @param[in]  : writeAddr --Write address channel structure
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalWriteAddress(const AxiWriteAddress* writeAddr)
{
    (void)writeAddr;
    return -1;
}

/******************************************************************************
 * @brief      : Write to AXI write data channel
 * @param[in]  : writeData --Write data channel structure dataWidth --Data width in bytes
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalWriteData(const AxiWriteData* writeData, uint8_t dataWidth)
{
    (void)writeData;
    (void)dataWidth;
    return -1;
}

/******************************************************************************
 * @brief      : Read from AXI write response channel
 * @param[in]  :
 * @param[out] : writeResp --Write response channel structure
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReadWriteResponse(AxiWriteResponse* writeResp)
{
    (void)writeResp;
    return -1;
}

/******************************************************************************
 * @brief      : Write to AXI read address channel
 * @param[in]  : readAddr --Read address channel structure
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReadAddress(const AxiReadAddress* readAddr)
{
    (void)readAddr;
    return -1;
}

/******************************************************************************
 * @brief      : Read from AXI read data channel
 * @param[in]  : dataWidth --Data width in bytes
 * @param[out] : readData --Read data channel structure
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReadData(AxiReadData* readData, uint8_t dataWidth)
{
    (void)readData;
    (void)dataWidth;
    return -1;
}

/******************************************************************************
 * @brief      : Send AXI4-Stream data
 * @param[in]  : streamData --Stream data structure
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalStreamSend(const AxiStreamData* streamData)
{
    (void)streamData;
    return -1;
}

/******************************************************************************
 * @brief      : Receive AXI4-Stream data
 * @param[in]  : maxLength --Maximum length to receive
 * @param[out] : streamData --Stream data structure
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalStreamReceive(AxiStreamData* streamData, size_t maxLength)
{
    (void)streamData;
    (void)maxLength;
    return -1;
}

/******************************************************************************
 * @brief      : Check if AXI bus is busy
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalIsBusy(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Wait for AXI transaction completion
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalWaitComplete(uint32_t timeoutMs)
{
    (void)timeoutMs;
    return -1;
}

/******************************************************************************
 * @brief      : Set AXI Quality of Service register
 * @param[in]  : qosValue --QoS value (0-15)
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalSetQoS(uint8_t qosValue)
{
    (void)qosValue;
    return -1;
}

/******************************************************************************
 * @brief      : Reset AXI interface
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (not supported on STM32F1)
 * @note       : STM32F1 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReset(void)
{
    return -1;
}

#elif defined(PLATFORM_ESP32)
/* ESP32 Platform - No AXI hardware support */

/******************************************************************************
 * @brief      : Initialize AXI HAL layer for ESP32 platform
 * @param[in]  : config --Pointer to AXI configuration structure
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalInit(const AxiConfig* config)
{
    (void)config;
    return -1;
}

/******************************************************************************
 * @brief      : Deinitialize AXI HAL layer for ESP32 platform
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalDeinit(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Write to AXI write address channel
 * @param[in]  : writeAddr --Write address channel structure
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalWriteAddress(const AxiWriteAddress* writeAddr)
{
    (void)writeAddr;
    return -1;
}

/******************************************************************************
 * @brief      : Write to AXI write data channel
 * @param[in]  : writeData --Write data channel structure dataWidth --Data width in bytes
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalWriteData(const AxiWriteData* writeData, uint8_t dataWidth)
{
    (void)writeData;
    (void)dataWidth;
    return -1;
}

/******************************************************************************
 * @brief      : Read from AXI write response channel
 * @param[in]  :
 * @param[out] : writeResp --Write response channel structure
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReadWriteResponse(AxiWriteResponse* writeResp)
{
    (void)writeResp;
    return -1;
}

/******************************************************************************
 * @brief      : Write to AXI read address channel
 * @param[in]  : readAddr --Read address channel structure
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReadAddress(const AxiReadAddress* readAddr)
{
    (void)readAddr;
    return -1;
}

/******************************************************************************
 * @brief      : Read from AXI read data channel
 * @param[in]  : dataWidth --Data width in bytes
 * @param[out] : readData --Read data channel structure
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReadData(AxiReadData* readData, uint8_t dataWidth)
{
    (void)readData;
    (void)dataWidth;
    return -1;
}

/******************************************************************************
 * @brief      : Send AXI4-Stream data
 * @param[in]  : streamData --Stream data structure
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalStreamSend(const AxiStreamData* streamData)
{
    (void)streamData;
    return -1;
}

/******************************************************************************
 * @brief      : Receive AXI4-Stream data
 * @param[in]  : maxLength --Maximum length to receive
 * @param[out] : streamData --Stream data structure
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalStreamReceive(AxiStreamData* streamData, size_t maxLength)
{
    (void)streamData;
    (void)maxLength;
    return -1;
}

/******************************************************************************
 * @brief      : Check if AXI bus is busy
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalIsBusy(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Wait for AXI transaction completion
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalWaitComplete(uint32_t timeoutMs)
{
    (void)timeoutMs;
    return -1;
}

/******************************************************************************
 * @brief      : Set AXI Quality of Service register
 * @param[in]  : qosValue --QoS value (0-15)
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalSetQoS(uint8_t qosValue)
{
    (void)qosValue;
    return -1;
}

/******************************************************************************
 * @brief      : Reset AXI interface
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (not supported on ESP32)
 * @note       : ESP32 does not support AXI hardware interface
 *****************************************************************************/
int AxiHalReset(void)
{
    return -1;
}

#elif defined(PLATFORM_LINUX)
/* Linux Platform - Simulated AXI with memory mapping */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

/** Simulated AXI register structure */
typedef struct {
    uint32_t awAddr;
    uint32_t awLen;
    uint32_t awSize;
    uint32_t wData[32];
    uint32_t wStrb;
    uint32_t bResp;
    uint32_t arAddr;
    uint32_t arLen;
    uint32_t arSize;
    uint32_t rData[32];
    uint32_t rResp;
    uint32_t status;
    uint32_t qos;
} AxiSimulatedRegs;

static AxiSimulatedRegs* g_axiRegs = NULL;
static uint8_t* g_simulatedMemory = NULL;
static size_t g_memorySize = 1024 * 1024;
static AxiConfig g_axiHalConfig = {0};

/******************************************************************************
 * @brief      : Initialize AXI HAL layer for Linux platform
 * @param[in]  : config --Pointer to AXI configuration structure
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Allocates simulated AXI registers and 1MB memory buffer
 *****************************************************************************/
int AxiHalInit(const AxiConfig* config)
{
    if (config == NULL) {
        return -1;
    }

    g_axiRegs = (AxiSimulatedRegs*)malloc(sizeof(AxiSimulatedRegs));
    if (g_axiRegs == NULL) {
        return -1;
    }

    g_simulatedMemory = (uint8_t*)malloc(g_memorySize);
    if (g_simulatedMemory == NULL) {
        free(g_axiRegs);
        g_axiRegs = NULL;
        return -1;
    }

    if (memset_s(g_axiRegs, sizeof(AxiSimulatedRegs), 0, sizeof(AxiSimulatedRegs)) != EOK) {
        free(g_simulatedMemory);
        free(g_axiRegs);
        g_axiRegs = NULL;
        g_simulatedMemory = NULL;
        return -1;
    }
    if (memset_s(g_simulatedMemory, g_memorySize, 0, g_memorySize) != EOK) {
        free(g_simulatedMemory);
        free(g_axiRegs);
        g_axiRegs = NULL;
        g_simulatedMemory = NULL;
        return -1;
    }
    if (memcpy_s(&g_axiHalConfig, sizeof(AxiConfig), config, sizeof(AxiConfig)) != EOK) {
        free(g_simulatedMemory);
        free(g_axiRegs);
        g_axiRegs = NULL;
        g_simulatedMemory = NULL;
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Deinitialize AXI HAL layer for Linux platform
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success
 * @note       : Frees allocated simulated registers and memory buffer
 *****************************************************************************/
int AxiHalDeinit(void)
{
    if (g_axiRegs != NULL) {
        free(g_axiRegs);
        g_axiRegs = NULL;
    }

    if (g_simulatedMemory != NULL) {
        free(g_simulatedMemory);
        g_simulatedMemory = NULL;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Write to AXI write address channel
 * @param[in]  : writeAddr --Write address channel structure containing address, burst length and size
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Updates simulated AWADDR, AWLEN, AWSIZE registers and sets busy status
 *****************************************************************************/
int AxiHalWriteAddress(const AxiWriteAddress* writeAddr)
{
    if (g_axiRegs == NULL || writeAddr == NULL) {
        return -1;
    }

    g_axiRegs->awAddr = writeAddr->address;
    g_axiRegs->awLen = writeAddr->burstLength;
    g_axiRegs->awSize = writeAddr->burstSize;
    g_axiRegs->status |= 0x01;

    return 0;
}

/******************************************************************************
 * @brief      : Write to AXI write data channel
 * @param[in]  : writeData --Write data channel structure containing data and strobe signals
                dataWidth --Data width in bytes
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Writes data to simulated memory, validates address bounds and sets write response
 *****************************************************************************/
int AxiHalWriteData(const AxiWriteData* writeData, uint8_t dataWidth)
{
    if (g_axiRegs == NULL || writeData == NULL || writeData->data == NULL) {
        return -1;
    }

    if (g_axiRegs->awAddr >= g_memorySize) {
        g_axiRegs->bResp = AXI_RESP_DECERR;
        return -1;
    }

    if (memcpy_s(g_axiRegs->wData, sizeof(g_axiRegs->wData), writeData->data, dataWidth) != EOK) {
        g_axiRegs->bResp = AXI_RESP_SLVERR;
        return -1;
    }

    size_t writeSize = (1U << g_axiRegs->awSize);
    if (g_axiRegs->awAddr + writeSize <= g_memorySize) {
        if (memcpy_s(&g_simulatedMemory[g_axiRegs->awAddr], g_memorySize - g_axiRegs->awAddr, writeData->data, writeSize) != EOK) {
            g_axiRegs->bResp = AXI_RESP_SLVERR;
            return -1;
        }
        g_axiRegs->bResp = AXI_RESP_OKAY;
    } else {
        g_axiRegs->bResp = AXI_RESP_SLVERR;
        return -1;
    }

    g_axiRegs->status &= ~0x01;
    return 0;
}

/******************************************************************************
 * @brief      : Read from AXI write response channel
 * @param[in]  :
 * @param[out] : writeResp --Write response channel structure to store response, ID and user data
 * @return     : 0 on success, -1 on failure
 * @note       : Retrieves write response from simulated BRESP register
 *****************************************************************************/
int AxiHalReadWriteResponse(AxiWriteResponse* writeResp)
{
    if (g_axiRegs == NULL || writeResp == NULL) {
        return -1;
    }

    writeResp->response = (AxiResponseType)g_axiRegs->bResp;
    writeResp->id = 0;
    writeResp->user = 0;

    return 0;
}

/******************************************************************************
 * @brief      : Write to AXI read address channel
 * @param[in]  : readAddr --Read address channel structure containing address, burst length and size
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Updates simulated ARADDR, ARLEN, ARSIZE registers and sets busy status
 *****************************************************************************/
int AxiHalReadAddress(const AxiReadAddress* readAddr)
{
    if (g_axiRegs == NULL || readAddr == NULL) {
        return -1;
    }

    g_axiRegs->arAddr = readAddr->address;
    g_axiRegs->arLen = readAddr->burstLength;
    g_axiRegs->arSize = readAddr->burstSize;
    g_axiRegs->status |= 0x01;

    return 0;
}

/******************************************************************************
 * @brief      : Read from AXI read data channel
 * @param[in]  : dataWidth --Data width in bytes
 * @param[out] : readData --Read data channel structure to store data, response, ID and flags
 * @return     : 0 on success, -1 on failure
 * @note       : Reads data from simulated memory, validates address bounds and sets read response
 *****************************************************************************/
int AxiHalReadData(AxiReadData* readData, uint8_t dataWidth)
{
    if (g_axiRegs == NULL || readData == NULL || readData->data == NULL) {
        return -1;
    }

    if (g_axiRegs->arAddr >= g_memorySize) {
        g_axiRegs->rResp = AXI_RESP_DECERR;
        readData->response = AXI_RESP_DECERR;
        return -1;
    }

    size_t readSize = (1U << g_axiRegs->arSize);
    if (g_axiRegs->arAddr + readSize <= g_memorySize) {
        if (memcpy_s(readData->data, dataWidth, &g_simulatedMemory[g_axiRegs->arAddr], readSize) != EOK) {
            g_axiRegs->rResp = AXI_RESP_SLVERR;
            readData->response = AXI_RESP_SLVERR;
            return -1;
        }
        if (memcpy_s(g_axiRegs->rData, sizeof(g_axiRegs->rData), readData->data, dataWidth) != EOK) {
            g_axiRegs->rResp = AXI_RESP_SLVERR;
            readData->response = AXI_RESP_SLVERR;
            return -1;
        }
        g_axiRegs->rResp = AXI_RESP_OKAY;
        readData->response = AXI_RESP_OKAY;
    } else {
        g_axiRegs->rResp = AXI_RESP_SLVERR;
        readData->response = AXI_RESP_SLVERR;
        return -1;
    }

    readData->id = 0;
    readData->last = 1;
    readData->user = 0;
    g_axiRegs->status &= ~0x01;

    return 0;
}

/******************************************************************************
 * @brief      : Send AXI4-Stream data
 * @param[in]  : streamData --Stream data structure
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : AXI4-Stream interface simulated (stub implementation)
 *****************************************************************************/
int AxiHalStreamSend(const AxiStreamData* streamData)
{
    if (g_axiRegs == NULL || streamData == NULL) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Receive AXI4-Stream data
 * @param[in]  : maxLength --Maximum length to receive
 * @param[out] : streamData --Stream data structure
 * @return     : 0 on success, -1 on failure
 * @note       : AXI4-Stream interface simulated (stub implementation)
 *****************************************************************************/
int AxiHalStreamReceive(AxiStreamData* streamData, size_t maxLength)
{
    if (g_axiRegs == NULL || streamData == NULL) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Check if AXI bus is busy
 * @param[in]  :
 * @param[out] :
 * @return     : 1 if busy, 0 if idle, -1 on error
 * @note       : Reads simulated status register busy bit
 *****************************************************************************/
int AxiHalIsBusy(void)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    return (g_axiRegs->status & 0x01) ? 1 : 0;
}

/******************************************************************************
 * @brief      : Wait for AXI transaction completion
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] :
 * @return     : 0 on success, -1 on timeout or error
 * @note       : Polls simulated status register with 1ms sleep intervals
 *****************************************************************************/
int AxiHalWaitComplete(uint32_t timeoutMs)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    uint32_t elapsed = 0;
    while ((g_axiRegs->status & 0x01) && elapsed < timeoutMs) {
        usleep(1000);
        elapsed++;
    }

    return (elapsed >= timeoutMs) ? -1 : 0;
}

/******************************************************************************
 * @brief      : Set AXI Quality of Service register
 * @param[in]  : qosValue --QoS value (0-15)
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Writes lower 4 bits of qosValue to simulated QoS register
 *****************************************************************************/
int AxiHalSetQoS(uint8_t qosValue)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    g_axiRegs->qos = qosValue & 0x0F;
    return 0;
}

/******************************************************************************
 * @brief      : Reset AXI interface
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on failure
 * @note       : Clears all simulated AXI registers to reset interface state
 *****************************************************************************/
int AxiHalReset(void)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    if (memset_s(g_axiRegs, sizeof(AxiSimulatedRegs), 0, sizeof(AxiSimulatedRegs)) != EOK) {
        return -1;
    }
    return 0;
}

#else
/* Unsupported platform */
#error "Unsupported platform for AXI HAL"
#endif

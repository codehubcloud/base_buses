/**
 * @file axi_hal.c
 * @brief AXI Hardware Abstraction Layer implementation
 * @details Multi-platform HAL for AXI bus operations supporting:
 *          - STM32F4 (memory-mapped AXI via pointer access)
 *          - STM32F1 (not supported - no AXI hardware)
 *          - ESP32 (not supported - no AXI hardware)
 *          - Linux (simulated with memory mapping)
 */

#include "axi_hal.h"
#include "axi.h"
#include "../platform_config.h"
#include <string.h>

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

static volatile uint32_t *g_axiBase = NULL;
static AxiConfig g_axiHalConfig = {0};

int AxiHalInit(const AxiConfig *config)
{
    if (config == NULL) {
        return -1;
    }

    g_axiBase = (volatile uint32_t *)AXI_BASE_ADDRESS;
    memcpy(&g_axiHalConfig, config, sizeof(AxiConfig));

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

    g_axiBase[AXI_STATUS_OFFSET / 4] = 0;

    return 0;
}

int AxiHalDeinit(void)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    g_axiBase[AXI_STATUS_OFFSET / 4] = 0;
    g_axiBase = NULL;

    return 0;
}

int AxiHalWriteAddress(const AxiWriteAddress *writeAddr)
{
    if (g_axiBase == NULL || writeAddr == NULL) {
        return -1;
    }

    g_axiBase[AXI_AWADDR_OFFSET / 4] = writeAddr->address;
    g_axiBase[AXI_AWLEN_OFFSET / 4] = writeAddr->burstLength;
    g_axiBase[AXI_AWSIZE_OFFSET / 4] = writeAddr->burstSize;

    return 0;
}

int AxiHalWriteData(const AxiWriteData *writeData, uint8_t dataWidth)
{
    if (g_axiBase == NULL || writeData == NULL || writeData->data == NULL) {
        return -1;
    }

    for (uint8_t i = 0; i < dataWidth / 4; i++) {
        g_axiBase[(AXI_WDATA_OFFSET / 4) + i] =
            ((uint32_t *)writeData->data)[i];
    }

    if (writeData->strobe != NULL) {
        g_axiBase[AXI_WSTRB_OFFSET / 4] = *(uint32_t *)writeData->strobe;
    } else {
        g_axiBase[AXI_WSTRB_OFFSET / 4] = 0xFFFFFFFF;
    }

    return 0;
}

int AxiHalReadWriteResponse(AxiWriteResponse *writeResp)
{
    if (g_axiBase == NULL || writeResp == NULL) {
        return -1;
    }

    uint32_t timeout = g_axiHalConfig.timeout * 1000;
    while ((g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) && timeout--) {
        for (volatile int i = 0; i < 100; i++) {}
    }

    if (timeout == 0) {
        return -1;
    }

    writeResp->response =
        (AxiResponseType)(g_axiBase[AXI_BRESP_OFFSET / 4] & 0x03);
    writeResp->id = (uint16_t)((g_axiBase[AXI_BRESP_OFFSET / 4] >> 16) & 0xFFFF);

    return 0;
}

int AxiHalReadAddress(const AxiReadAddress *readAddr)
{
    if (g_axiBase == NULL || readAddr == NULL) {
        return -1;
    }

    g_axiBase[AXI_ARADDR_OFFSET / 4] = readAddr->address;
    g_axiBase[AXI_ARLEN_OFFSET / 4] = readAddr->burstLength;
    g_axiBase[AXI_ARSIZE_OFFSET / 4] = readAddr->burstSize;

    return 0;
}

int AxiHalReadData(AxiReadData *readData, uint8_t dataWidth)
{
    if (g_axiBase == NULL || readData == NULL || readData->data == NULL) {
        return -1;
    }

    uint32_t timeout = g_axiHalConfig.timeout * 1000;
    while ((g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) && timeout--) {
        for (volatile int i = 0; i < 100; i++) {}
    }

    if (timeout == 0) {
        return -1;
    }

    for (uint8_t i = 0; i < dataWidth / 4; i++) {
        ((uint32_t *)readData->data)[i] = g_axiBase[(AXI_RDATA_OFFSET / 4) + i];
    }

    readData->response =
        (AxiResponseType)(g_axiBase[AXI_RRESP_OFFSET / 4] & 0x03);
    readData->id = (uint16_t)((g_axiBase[AXI_RRESP_OFFSET / 4] >> 16) & 0xFFFF);

    return 0;
}

int AxiHalStreamSend(const AxiStreamData *streamData)
{
    if (g_axiBase == NULL || streamData == NULL) {
        return -1;
    }

    return -1;
}

int AxiHalStreamReceive(AxiStreamData *streamData, size_t maxLength)
{
    if (g_axiBase == NULL || streamData == NULL) {
        return -1;
    }

    return -1;
}

int AxiHalIsBusy(void)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    return (g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) ? 1 : 0;
}

int AxiHalWaitComplete(uint32_t timeoutMs)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    uint32_t timeout = timeoutMs * 1000;
    while ((g_axiBase[AXI_STATUS_OFFSET / 4] & AXI_STATUS_BUSY) && timeout--) {
        for (volatile int i = 0; i < 100; i++) {}
    }

    return (timeout == 0) ? -1 : 0;
}

int AxiHalSetQoS(uint8_t qosValue)
{
    if (g_axiBase == NULL) {
        return -1;
    }

    g_axiBase[AXI_QOS_OFFSET / 4] = qosValue & 0x0F;
    return 0;
}

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

int AxiHalInit(const AxiConfig *config)
{
    (void)config;
    return -1;
}

int AxiHalDeinit(void)
{
    return -1;
}

int AxiHalWriteAddress(const AxiWriteAddress *writeAddr)
{
    (void)writeAddr;
    return -1;
}

int AxiHalWriteData(const AxiWriteData *writeData, uint8_t dataWidth)
{
    (void)writeData;
    (void)dataWidth;
    return -1;
}

int AxiHalReadWriteResponse(AxiWriteResponse *writeResp)
{
    (void)writeResp;
    return -1;
}

int AxiHalReadAddress(const AxiReadAddress *readAddr)
{
    (void)readAddr;
    return -1;
}

int AxiHalReadData(AxiReadData *readData, uint8_t dataWidth)
{
    (void)readData;
    (void)dataWidth;
    return -1;
}

int AxiHalStreamSend(const AxiStreamData *streamData)
{
    (void)streamData;
    return -1;
}

int AxiHalStreamReceive(AxiStreamData *streamData, size_t maxLength)
{
    (void)streamData;
    (void)maxLength;
    return -1;
}

int AxiHalIsBusy(void)
{
    return -1;
}

int AxiHalWaitComplete(uint32_t timeoutMs)
{
    (void)timeoutMs;
    return -1;
}

int AxiHalSetQoS(uint8_t qosValue)
{
    (void)qosValue;
    return -1;
}

int AxiHalReset(void)
{
    return -1;
}

#elif defined(PLATFORM_ESP32)
/* ESP32 Platform - No AXI hardware support */

int AxiHalInit(const AxiConfig *config)
{
    (void)config;
    return -1;
}

int AxiHalDeinit(void)
{
    return -1;
}

int AxiHalWriteAddress(const AxiWriteAddress *writeAddr)
{
    (void)writeAddr;
    return -1;
}

int AxiHalWriteData(const AxiWriteData *writeData, uint8_t dataWidth)
{
    (void)writeData;
    (void)dataWidth;
    return -1;
}

int AxiHalReadWriteResponse(AxiWriteResponse *writeResp)
{
    (void)writeResp;
    return -1;
}

int AxiHalReadAddress(const AxiReadAddress *readAddr)
{
    (void)readAddr;
    return -1;
}

int AxiHalReadData(AxiReadData *readData, uint8_t dataWidth)
{
    (void)readData;
    (void)dataWidth;
    return -1;
}

int AxiHalStreamSend(const AxiStreamData *streamData)
{
    (void)streamData;
    return -1;
}

int AxiHalStreamReceive(AxiStreamData *streamData, size_t maxLength)
{
    (void)streamData;
    (void)maxLength;
    return -1;
}

int AxiHalIsBusy(void)
{
    return -1;
}

int AxiHalWaitComplete(uint32_t timeoutMs)
{
    (void)timeoutMs;
    return -1;
}

int AxiHalSetQoS(uint8_t qosValue)
{
    (void)qosValue;
    return -1;
}

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

static AxiSimulatedRegs *g_axiRegs = NULL;
static uint8_t *g_simulatedMemory = NULL;
static size_t g_memorySize = 1024 * 1024;
static AxiConfig g_axiHalConfig = {0};

int AxiHalInit(const AxiConfig *config)
{
    if (config == NULL) {
        return -1;
    }

    g_axiRegs = (AxiSimulatedRegs *)malloc(sizeof(AxiSimulatedRegs));
    if (g_axiRegs == NULL) {
        return -1;
    }

    g_simulatedMemory = (uint8_t *)malloc(g_memorySize);
    if (g_simulatedMemory == NULL) {
        free(g_axiRegs);
        g_axiRegs = NULL;
        return -1;
    }

    memset(g_axiRegs, 0, sizeof(AxiSimulatedRegs));
    memset(g_simulatedMemory, 0, g_memorySize);
    memcpy(&g_axiHalConfig, config, sizeof(AxiConfig));

    return 0;
}

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

int AxiHalWriteAddress(const AxiWriteAddress *writeAddr)
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

int AxiHalWriteData(const AxiWriteData *writeData, uint8_t dataWidth)
{
    if (g_axiRegs == NULL || writeData == NULL || writeData->data == NULL) {
        return -1;
    }

    if (g_axiRegs->awAddr >= g_memorySize) {
        g_axiRegs->bResp = AXI_RESP_DECERR;
        return -1;
    }

    memcpy(g_axiRegs->wData, writeData->data, dataWidth);

    size_t writeSize = (1U << g_axiRegs->awSize);
    if (g_axiRegs->awAddr + writeSize <= g_memorySize) {
        memcpy(&g_simulatedMemory[g_axiRegs->awAddr], writeData->data,
               writeSize);
        g_axiRegs->bResp = AXI_RESP_OKAY;
    } else {
        g_axiRegs->bResp = AXI_RESP_SLVERR;
        return -1;
    }

    g_axiRegs->status &= ~0x01;
    return 0;
}

int AxiHalReadWriteResponse(AxiWriteResponse *writeResp)
{
    if (g_axiRegs == NULL || writeResp == NULL) {
        return -1;
    }

    writeResp->response = (AxiResponseType)g_axiRegs->bResp;
    writeResp->id = 0;
    writeResp->user = 0;

    return 0;
}

int AxiHalReadAddress(const AxiReadAddress *readAddr)
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

int AxiHalReadData(AxiReadData *readData, uint8_t dataWidth)
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
        memcpy(readData->data, &g_simulatedMemory[g_axiRegs->arAddr],
               readSize);
        memcpy(g_axiRegs->rData, readData->data, dataWidth);
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

int AxiHalStreamSend(const AxiStreamData *streamData)
{
    if (g_axiRegs == NULL || streamData == NULL) {
        return -1;
    }

    return 0;
}

int AxiHalStreamReceive(AxiStreamData *streamData, size_t maxLength)
{
    if (g_axiRegs == NULL || streamData == NULL) {
        return -1;
    }

    return 0;
}

int AxiHalIsBusy(void)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    return (g_axiRegs->status & 0x01) ? 1 : 0;
}

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

int AxiHalSetQoS(uint8_t qosValue)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    g_axiRegs->qos = qosValue & 0x0F;
    return 0;
}

int AxiHalReset(void)
{
    if (g_axiRegs == NULL) {
        return -1;
    }

    memset(g_axiRegs, 0, sizeof(AxiSimulatedRegs));
    return 0;
}

#else
/* Unsupported platform */
#error "Unsupported platform for AXI HAL"
#endif

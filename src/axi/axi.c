/**
 * @file axi.c
 * @brief AXI (Advanced eXtensible Interface) bus protocol implementation
 * @details Protocol-agnostic implementation of AXI4, AXI4-Lite, and AXI4-Stream
 */

#include <string.h>
#include "axi.h"
#include "axi_hal.h"
#include "securec.h"


/** Current AXI configuration */
static AxiConfig g_axiConfig = {0};

/** Current AXI status */
static AxiStatus g_axiStatus = {0};

/** Initialization flag */
static uint8_t g_axiInitialized = 0;

/**
 * @brief Validate AXI configuration parameters
 * @param[in] config Configuration structure pointer
 * @return 0 on success, -1 on failure
 */
static int ValidateConfig(const AxiConfig* config)
{
    if (config == NULL) {
        return -1;
    }

    if (config->dataWidth != 32 && config->dataWidth != 64 && config->dataWidth != 128 && config->dataWidth != 256
        && config->dataWidth != 512 && config->dataWidth != 1024) {
        return -1;
    }

    if (config->addressWidth != 32 && config->addressWidth != 64) {
        return -1;
    }

    if (config->maxBurstLength > AXI_MAX_BURST_LEN) {
        return -1;
    }

    if (config->isLite && config->maxBurstLength > AXI_LITE_MAX_BURST_LEN) {
        return -1;
    }

    return 0;
}

/**
 * @brief Validate write address channel parameters
 * @param[in] writeAddr Write address channel structure
 * @return 0 on success, -1 on failure
 */
static int ValidateWriteAddress(const AxiWriteAddress* writeAddr)
{
    if (writeAddr == NULL) {
        return -1;
    }

    if (writeAddr->burstLength > g_axiConfig.maxBurstLength) {
        return -1;
    }

    if (writeAddr->burstType == AXI_BURST_RESERVED) {
        return -1;
    }

    if (writeAddr->qos > 15) {
        return -1;
    }

    return 0;
}

/**
 * @brief Validate read address channel parameters
 * @param[in] readAddr Read address channel structure
 * @return 0 on success, -1 on failure
 */
static int ValidateReadAddress(const AxiReadAddress* readAddr)
{
    if (readAddr == NULL) {
        return -1;
    }

    if (readAddr->burstLength > g_axiConfig.maxBurstLength) {
        return -1;
    }

    if (readAddr->burstType == AXI_BURST_RESERVED) {
        return -1;
    }

    if (readAddr->qos > 15) {
        return -1;
    }

    return 0;
}

/**
 * @brief Initialize AXI bus interface
 * @param[in] config Configuration structure pointer
 * @return 0 on success, -1 on failure
 */
int AxiInit(const AxiConfig* config)
{
    if (g_axiInitialized) {
        return -1;
    }

    if (ValidateConfig(config) != 0) {
        return -1;
    }

    if (memcpy_s(&g_axiConfig, sizeof(AxiConfig), config, sizeof(AxiConfig)) != EOK) {
        return -1;
    }

    if (AxiHalInit(config) != 0) {
        return -1;
    }

    if (memset_s(&g_axiStatus, sizeof(AxiStatus), 0, sizeof(AxiStatus)) != EOK) {
        return -1;
    }
    g_axiStatus.isInitialized = 1;
    g_axiInitialized = 1;

    return 0;
}

/**
 * @brief Deinitialize AXI bus interface
 * @return 0 on success, -1 on failure
 */
int AxiDeinit(void)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (AxiHalDeinit() != 0) {
        return -1;
    }

    if (memset_s(&g_axiConfig, sizeof(AxiConfig), 0, sizeof(AxiConfig)) != EOK) {
        return -1;
    }
    if (memset_s(&g_axiStatus, sizeof(AxiStatus), 0, sizeof(AxiStatus)) != EOK) {
        return -1;
    }
    g_axiInitialized = 0;

    return 0;
}

/**
 * @brief Perform single AXI write transaction
 * @param[in] writeAddr Write address channel structure
 * @param[in] writeData Write data channel structure
 * @param[out] writeResp Write response channel structure
 * @return 0 on success, -1 on failure
 */
int AxiWriteTransaction(const AxiWriteAddress* writeAddr, const AxiWriteData* writeData, AxiWriteResponse* writeResp)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (ValidateWriteAddress(writeAddr) != 0) {
        return -1;
    }

    if (writeData == NULL || writeData->data == NULL) {
        return -1;
    }

    if (writeResp == NULL) {
        return -1;
    }

    if (AxiHalIsBusy() == 1) {
        g_axiStatus.isBusy = 1;
        return -1;
    }

    if (AxiHalWriteAddress(writeAddr) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    if (AxiHalWriteData(writeData, g_axiConfig.dataWidth / 8) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    if (AxiHalReadWriteResponse(writeResp) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    if (writeResp->response != AXI_RESP_OKAY && writeResp->response != AXI_RESP_EXOKAY) {
        g_axiStatus.errorCount++;
        g_axiStatus.lastError = writeResp->response;
        return -1;
    }

    g_axiStatus.writeCount++;
    return 0;
}

/**
 * @brief Perform single AXI read transaction
 * @param[in] readAddr Read address channel structure
 * @param[out] readData Read data channel structure
 * @return 0 on success, -1 on failure
 */
int AxiReadTransaction(const AxiReadAddress* readAddr, AxiReadData* readData)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (ValidateReadAddress(readAddr) != 0) {
        return -1;
    }

    if (readData == NULL || readData->data == NULL) {
        return -1;
    }

    if (AxiHalIsBusy() == 1) {
        g_axiStatus.isBusy = 1;
        return -1;
    }

    if (AxiHalReadAddress(readAddr) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    if (AxiHalReadData(readData, g_axiConfig.dataWidth / 8) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    if (readData->response != AXI_RESP_OKAY && readData->response != AXI_RESP_EXOKAY) {
        g_axiStatus.errorCount++;
        g_axiStatus.lastError = readData->response;
        return -1;
    }

    g_axiStatus.readCount++;
    return 0;
}

/**
 * @brief Perform AXI burst write operation
 * @param[in] writeAddr Write address channel structure
 * @param[in] writeData Array of write data structures
 * @param[in] burstLength Number of transfers in burst
 * @param[out] writeResp Write response channel structure
 * @return 0 on success, -1 on failure
 */
int AxiWriteBurst(const AxiWriteAddress* writeAddr, const AxiWriteData* writeData, uint8_t burstLength, AxiWriteResponse* writeResp)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (ValidateWriteAddress(writeAddr) != 0) {
        return -1;
    }

    if (writeData == NULL || writeResp == NULL) {
        return -1;
    }

    if (burstLength == 0 || burstLength > g_axiConfig.maxBurstLength) {
        return -1;
    }

    if (writeAddr->burstLength != burstLength - 1) {
        return -1;
    }

    if (AxiHalWriteAddress(writeAddr) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    for (uint8_t i = 0; i < burstLength; i++) {
        if (writeData[i].data == NULL) {
            g_axiStatus.errorCount++;
            return -1;
        }

        if (AxiHalWriteData(&writeData[i], g_axiConfig.dataWidth / 8) != 0) {
            g_axiStatus.errorCount++;
            return -1;
        }
    }

    if (AxiHalReadWriteResponse(writeResp) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    if (writeResp->response != AXI_RESP_OKAY && writeResp->response != AXI_RESP_EXOKAY) {
        g_axiStatus.errorCount++;
        g_axiStatus.lastError = writeResp->response;
        return -1;
    }

    g_axiStatus.writeCount++;
    return 0;
}

/**
 * @brief Perform AXI burst read operation
 * @param[in] readAddr Read address channel structure
 * @param[out] readData Array of read data structures
 * @param[in] burstLength Number of transfers in burst
 * @return 0 on success, -1 on failure
 */
int AxiReadBurst(const AxiReadAddress* readAddr, AxiReadData* readData, uint8_t burstLength)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (ValidateReadAddress(readAddr) != 0) {
        return -1;
    }

    if (readData == NULL) {
        return -1;
    }

    if (burstLength == 0 || burstLength > g_axiConfig.maxBurstLength) {
        return -1;
    }

    if (readAddr->burstLength != burstLength - 1) {
        return -1;
    }

    if (AxiHalReadAddress(readAddr) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    for (uint8_t i = 0; i < burstLength; i++) {
        if (readData[i].data == NULL) {
            g_axiStatus.errorCount++;
            return -1;
        }

        if (AxiHalReadData(&readData[i], g_axiConfig.dataWidth / 8) != 0) {
            g_axiStatus.errorCount++;
            return -1;
        }

        if (readData[i].response != AXI_RESP_OKAY && readData[i].response != AXI_RESP_EXOKAY) {
            g_axiStatus.errorCount++;
            g_axiStatus.lastError = readData[i].response;
            return -1;
        }
    }

    g_axiStatus.readCount++;
    return 0;
}

/**
 * @brief Configure AXI Quality of Service settings
 * @param[in] qosValue QoS value (0-15)
 * @return 0 on success, -1 on failure
 */
int AxiSetQoS(uint8_t qosValue)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (qosValue > 15) {
        return -1;
    }

    if (AxiHalSetQoS(qosValue) != 0) {
        return -1;
    }

    return 0;
}

/**
 * @brief Get current AXI bus status
 * @param[out] status Status structure pointer
 * @return 0 on success, -1 on failure
 */
int AxiGetStatus(AxiStatus* status)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (status == NULL) {
        return -1;
    }

    int busyStatus = AxiHalIsBusy();
    if (busyStatus >= 0) {
        g_axiStatus.isBusy = (uint8_t)busyStatus;
    }

    if (memcpy_s(status, sizeof(AxiStatus), &g_axiStatus, sizeof(AxiStatus)) != EOK) {
        return -1;
    }

    return 0;
}

/**
 * @brief Send data via AXI4-Stream interface
 * @param[in] streamData Stream data structure
 * @return 0 on success, -1 on failure
 */
int AxiStreamSend(const AxiStreamData* streamData)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (streamData == NULL || streamData->data == NULL) {
        return -1;
    }

    if (streamData->length == 0) {
        return -1;
    }

    if (AxiHalStreamSend(streamData) != 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    return 0;
}

/**
 * @brief Receive data via AXI4-Stream interface
 * @param[out] streamData Stream data structure
 * @param[in] maxLength Maximum length to receive
 * @return Number of bytes received on success, -1 on failure
 */
int AxiStreamReceive(AxiStreamData* streamData, size_t maxLength)
{
    if (!g_axiInitialized) {
        return -1;
    }

    if (streamData == NULL || streamData->data == NULL) {
        return -1;
    }

    if (maxLength == 0) {
        return -1;
    }

    int received = AxiHalStreamReceive(streamData, maxLength);
    if (received < 0) {
        g_axiStatus.errorCount++;
        return -1;
    }

    return received;
}

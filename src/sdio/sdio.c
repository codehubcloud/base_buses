/******************************************************************************
 * @file    : sdio.c
 * @brief   : SDIO protocol implementation
 * @author  : Code Generator
 * @date    : 2025-10-01
 * @version : V1.0
 * @note    : SD/SDIO interface protocol layer
 ******************************************************************************/

#include <string.h>
#include "sdio.h"
#include "sdio_hal.h"
#include "securec.h"


/******************************************************************************
 * Static Variables
 ******************************************************************************/
static uint32_t currentBlockSize = SDIO_DEFAULT_BLOCK_SIZE;
static SdioBusWidth_E currentBusWidth = SDIO_BUS_WIDTH_1BIT;

/******************************************************************************
 * @brief     : Initialize SDIO peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SDIO with 1-bit mode and default 25MHz clock
 *****************************************************************************/
int32_t SdioInit(void)
{
    int32_t result = 0;

    result = SdioEnableClock();
    if (result != 0) {
        return -1;
    }

    result = SdioConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = SdioSetClockSpeed(SDIO_CLOCK_DEFAULT);
    if (result != 0) {
        return -1;
    }

    result = SdioSetBusWidth(SDIO_BUS_WIDTH_1BIT);
    if (result != 0) {
        return -1;
    }

    SdioEnable();
    currentBlockSize = SDIO_DEFAULT_BLOCK_SIZE;
    currentBusWidth = SDIO_BUS_WIDTH_1BIT;

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables SDIO peripheral and releases resources
 *****************************************************************************/
int32_t SdioDeinit(void)
{
    SdioDisable();
    SdioDisableClock();
    return 0;
}

/******************************************************************************
 * @brief     : Send SDIO command
 * @param[in] : cmd - Command index (0-63), arg - Command argument
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends command with CRC7 checksum
 *****************************************************************************/
int32_t SdioSendCommand(uint8_t cmd, uint32_t arg)
{
    if (cmd > 63) {
        return -1;
    }

    uint8_t cmdBuffer[6];
    cmdBuffer[0] = 0x40 | cmd;
    cmdBuffer[1] = (arg >> 24) & 0xFF;
    cmdBuffer[2] = (arg >> 16) & 0xFF;
    cmdBuffer[3] = (arg >> 8) & 0xFF;
    cmdBuffer[4] = arg & 0xFF;
    cmdBuffer[5] = (SdioCalculateCRC7(cmdBuffer, 5) << 1) | 0x01;

    return SdioSendCommandHal(cmd, arg, cmdBuffer[5]);
}

/******************************************************************************
 * @brief     : Read SDIO response
 * @param[in] : responseType - Type of response expected (NONE/SHORT/LONG)
 * @param[out]: response - Pointer to response buffer (up to 4 words)
 * @return    : 0 if success, -1 if error
 * @note      : Reads and validates response with CRC7 check
 *****************************************************************************/
int32_t SdioReadResponse(uint32_t* response, SdioResponseType_E responseType)
{
    if (response == NULL && responseType != SDIO_RESPONSE_NONE) {
        return -1;
    }

    if (responseType == SDIO_RESPONSE_NONE) {
        return 0;
    }

    return SdioReadResponseHal(response, responseType);
}

/******************************************************************************
 * @brief     : Read data from SDIO
 * @param[in] : length - Number of bytes to read
 * @param[out]: data - Buffer to store received data
 * @return    : 0 if success, -1 if error
 * @note      : Reads data with CRC16 validation
 *****************************************************************************/
int32_t SdioReadData(uint8_t* data, uint32_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    int32_t result = SdioReadDataHal(data, length);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write data to SDIO
 * @param[in] : data - Pointer to data buffer, length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes data with CRC16 checksum
 *****************************************************************************/
int32_t SdioWriteData(uint8_t* data, uint32_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    uint16_t crc16 = SdioCalculateCRC16(data, length);
    int32_t result = SdioWriteDataHal(data, length, crc16);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set SDIO bus width
 * @param[in] : width - Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures data bus width for parallel transfer
 *****************************************************************************/
int32_t SdioSetBusWidth(uint8_t width)
{
    if ((width != 1) && (width != 4) && (width != 8)) {
        return -1;
    }

    int32_t result = SdioSetBusWidthHal(width);
    if (result == 0) {
        currentBusWidth = (SdioBusWidth_E)width;
    }

    return result;
}

/******************************************************************************
 * @brief     : Set SDIO clock speed
 * @param[in] : speed - Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SDIO clock frequency
 *****************************************************************************/
int32_t SdioSetClockSpeed(uint32_t speed)
{
    if (speed == 0) {
        return -1;
    }

    return SdioSetClockSpeedHal(speed);
}

/******************************************************************************
 * @brief     : Calculate CRC7 checksum for SDIO commands
 * @param[in] : data - Pointer to data buffer, length - Number of bytes
 * @param[out]: None
 * @return    : CRC7 checksum value (7 bits)
 * @note      : Used for command and response validation
 *****************************************************************************/
uint8_t SdioCalculateCRC7(uint8_t* data, uint8_t length)
{
    uint8_t crc = 0;
    uint8_t poly = 0x89;

    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc >> 1;
}

/******************************************************************************
 * @brief     : Calculate CRC16 checksum for SDIO data
 * @param[in] : data - Pointer to data buffer, length - Number of bytes
 * @param[out]: None
 * @return    : CRC16 checksum value
 * @note      : Used for data block validation
 *****************************************************************************/
uint16_t SdioCalculateCRC16(uint8_t* data, uint32_t length)
{
    uint16_t crc = 0;
    uint16_t poly = 0x1021;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/******************************************************************************
 * @brief     : Read single block from SDIO device
 * @param[in] : blockAddr - Block address, blockSize - Size of block in bytes
 * @param[out]: data - Buffer to store block data
 * @return    : 0 if success, -1 if error
 * @note      : Reads a single data block with CMD17
 *****************************************************************************/
int32_t SdioReadBlock(uint32_t blockAddr, uint8_t* data, uint32_t blockSize)
{
    if ((data == NULL) || (blockSize == 0)) {
        return -1;
    }

    int32_t result = SdioSendCommand(SDIO_CMD17_READ_SINGLE_BLOCK, blockAddr);
    if (result != 0) {
        return -1;
    }

    uint32_t response[4];
    result = SdioReadResponse(response, SDIO_RESPONSE_SHORT);
    if (result != 0) {
        return -1;
    }

    result = SdioReadData(data, blockSize);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write single block to SDIO device
 * @param[in] : blockAddr - Block address, data - Data to write, blockSize - Size of block
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes a single data block with CMD24
 *****************************************************************************/
int32_t SdioWriteBlock(uint32_t blockAddr, uint8_t* data, uint32_t blockSize)
{
    if ((data == NULL) || (blockSize == 0)) {
        return -1;
    }

    int32_t result = SdioSendCommand(SDIO_CMD24_WRITE_BLOCK, blockAddr);
    if (result != 0) {
        return -1;
    }

    uint32_t response[4];
    result = SdioReadResponse(response, SDIO_RESPONSE_SHORT);
    if (result != 0) {
        return -1;
    }

    result = SdioWriteData(data, blockSize);
    if (result != 0) {
        return -1;
    }

    return 0;
}

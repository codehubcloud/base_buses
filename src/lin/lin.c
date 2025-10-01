#include <string.h>
#include "lin.h"
#include "lin_hal.h"
#include "securec.h"

/* Global LIN configuration */
static LinConfig_t g_linConfig = {.baudRate = LIN_DEFAULT_BAUDRATE, .version = LIN_VERSION_2_0, .defaultChecksumType = LIN_CHECKSUM_ENHANCED};

/* Current frame being processed */
static uint8_t g_currentId = 0;

/******************************************************************************
 * @brief     : Initialize LIN peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures LIN with 19200 baud rate and LIN 2.0 protocol
 *****************************************************************************/
int32_t LinInit(void)
{
    int32_t result = 0;

    result = LinEnableClock();
    if (result != 0) {
        return -1;
    }

    result = LinConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = LinConfigureUart(g_linConfig.baudRate);
    if (result != 0) {
        return -1;
    }

    LinEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Initialize LIN peripheral with custom configuration
 * @param[in] : config - Pointer to LIN configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Allows custom baud rate and protocol version
 *****************************************************************************/
int32_t LinInitWithConfig(const LinConfig_t* config)
{
    if (config == NULL) {
        return -1;
    }

    if (memcpy_s(&g_linConfig, sizeof(LinConfig_t), config, sizeof(LinConfig_t)) != EOK) {
        return -1;
    }

    return LinInit();
}

/******************************************************************************
 * @brief     : Deinitialize LIN peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables LIN and releases resources
 *****************************************************************************/
int32_t LinDeinit(void)
{
    LinDisable();
    return 0;
}

/******************************************************************************
 * @brief     : Send LIN break field
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends 13-bit dominant (low) break signal
 *****************************************************************************/
int32_t LinSendBreak(void)
{
    return LinHalSendBreak();
}

/******************************************************************************
 * @brief     : Calculate protected identifier (ID with parity bits)
 * @param[in] : id - LIN identifier (0-63)
 * @param[out]: None
 * @return    : Protected ID with parity bits
 * @note      : Adds P0 and P1 parity bits to 6-bit ID
 *****************************************************************************/
uint8_t LinCalculateProtectedId(uint8_t id)
{
    uint8_t protectedId = id & 0x3F; /* Ensure only 6 bits */
    uint8_t p0, p1;

    /* P0 = ID0 XOR ID1 XOR ID2 XOR ID4 */
    p0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ ((id >> 2) & 1) ^ ((id >> 4) & 1);

    /* P1 = NOT(ID1 XOR ID3 XOR ID4 XOR ID5) */
    p1 = ~(((id >> 1) & 1) ^ ((id >> 3) & 1) ^ ((id >> 4) & 1) ^ ((id >> 5) & 1));
    p1 &= 1; /* Ensure single bit */

    /* Combine: [P1][P0][ID5][ID4][ID3][ID2][ID1][ID0] */
    protectedId |= (p0 << 6) | (p1 << 7);

    return protectedId;
}

/******************************************************************************
 * @brief     : Validate protected identifier parity
 * @param[in] : protectedId - Protected ID byte with parity
 * @param[out]: None
 * @return    : 1 if valid, 0 if invalid
 * @note      : Checks P0 and P1 parity bits
 *****************************************************************************/
int32_t LinValidateProtectedId(uint8_t protectedId)
{
    uint8_t id = protectedId & 0x3F;
    uint8_t p0 = (protectedId >> 6) & 1;
    uint8_t p1 = (protectedId >> 7) & 1;
    uint8_t expectedP0, expectedP1;

    /* Calculate expected parity bits */
    expectedP0 = ((id >> 0) & 1) ^ ((id >> 1) & 1) ^ ((id >> 2) & 1) ^ ((id >> 4) & 1);
    expectedP1 = ~(((id >> 1) & 1) ^ ((id >> 3) & 1) ^ ((id >> 4) & 1) ^ ((id >> 5) & 1));
    expectedP1 &= 1;

    return ((p0 == expectedP0) && (p1 == expectedP1)) ? 1 : 0;
}

/******************************************************************************
 * @brief     : Send LIN frame header (break + sync + ID)
 * @param[in] : id - LIN identifier (0-63)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Master function - sends break, sync byte, and protected ID
 *****************************************************************************/
int32_t LinSendHeader(uint8_t id)
{
    uint8_t protectedId;
    int32_t result;

    if (id > 0x3F) { /* ID must be 6 bits (0-63) */
        return -1;
    }

    /* Store current ID for checksum calculation */
    g_currentId = id;

    /* Send break field (13-bit dominant) */
    result = LinSendBreak();
    if (result != 0) {
        return -1;
    }

    /* Send sync byte (0x55) */
    result = LinWriteByte(LIN_SYNC_BYTE);
    if (result != 0) {
        return -1;
    }

    /* Calculate and send protected ID */
    protectedId = LinCalculateProtectedId(id);
    result = LinWriteByte(protectedId);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Calculate LIN checksum
 * @param[in] : id - LIN identifier, data - Pointer to data buffer,
 *              length - Number of bytes, enhanced - Use enhanced checksum
 * @param[out]: None
 * @return    : Calculated checksum byte
 * @note      : Classic checksum uses data only, enhanced includes ID
 *****************************************************************************/
uint8_t LinCalculateChecksum(uint8_t id, const uint8_t* data, uint8_t length, bool enhanced)
{
    uint16_t checksum = 0;
    uint8_t i;

    if (data == NULL || length == 0 || length > LIN_MAX_DATA_LENGTH) {
        return 0xFF;
    }

    /* Enhanced checksum includes protected ID */
    if (enhanced) {
        uint8_t protectedId = LinCalculateProtectedId(id);
        checksum = protectedId;
    }

    /* Add all data bytes */
    for (i = 0; i < length; i++) {
        checksum += data[i];
        /* Add carry */
        if (checksum > 0xFF) {
            checksum = (checksum & 0xFF) + 1;
        }
    }

    /* Invert checksum */
    checksum = ~checksum & 0xFF;

    return (uint8_t)checksum;
}

/******************************************************************************
 * @brief     : Send LIN response (data + checksum)
 * @param[in] : data - Pointer to data buffer, length - Number of bytes (1-8)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Can be used by master or slave to respond to header
 *****************************************************************************/
int32_t LinSendResponse(const uint8_t* data, uint8_t length)
{
    uint8_t checksum;
    uint8_t i;
    int32_t result;

    if (data == NULL || length == 0 || length > LIN_MAX_DATA_LENGTH) {
        return -1;
    }

    /* Send data bytes */
    for (i = 0; i < length; i++) {
        result = LinWriteByte(data[i]);
        if (result != 0) {
            return -1;
        }
    }

    /* Calculate and send checksum */
    checksum = LinCalculateChecksum(g_currentId, data, length, g_linConfig.defaultChecksumType == LIN_CHECKSUM_ENHANCED);
    result = LinWriteByte(checksum);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive LIN response (data + checksum)
 * @param[in] : length - Expected number of data bytes (1-8)
 * @param[out]: data - Buffer to store received data
 * @return    : Number of bytes received, -1 if error
 * @note      : Validates checksum and returns data if valid
 *****************************************************************************/
int32_t LinReceiveResponse(uint8_t* data, uint8_t length)
{
    uint8_t receivedChecksum;
    uint8_t calculatedChecksum;
    uint8_t i;
    int32_t result;
    uint32_t timeout;

    if (data == NULL || length == 0 || length > LIN_MAX_DATA_LENGTH) {
        return -1;
    }

    /* Receive data bytes with timeout */
    for (i = 0; i < length; i++) {
        timeout = 10000; /* Timeout counter */
        while (LinRxBufferHasData() == 0 && timeout > 0) {
            LinDelayUs(10);
            timeout--;
        }

        if (timeout == 0) {
            return -1; /* Timeout */
        }

        result = LinReadByte(&data[i]);
        if (result != 0) {
            return -1;
        }
    }

    /* Receive checksum byte */
    timeout = 10000;
    while (LinRxBufferHasData() == 0 && timeout > 0) {
        LinDelayUs(10);
        timeout--;
    }

    if (timeout == 0) {
        return -1; /* Timeout */
    }

    result = LinReadByte(&receivedChecksum);
    if (result != 0) {
        return -1;
    }

    /* Validate checksum */
    calculatedChecksum = LinCalculateChecksum(g_currentId, data, length, g_linConfig.defaultChecksumType == LIN_CHECKSUM_ENHANCED);

    if (receivedChecksum != calculatedChecksum) {
        return -1; /* Checksum mismatch */
    }

    return (int32_t)length;
}

/******************************************************************************
 * @brief     : Send complete LIN frame (master mode)
 * @param[in] : frame - Pointer to LIN frame structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends header and response as master
 *****************************************************************************/
int32_t LinSendFrame(const LinFrame_t* frame)
{
    int32_t result;

    if (frame == NULL) {
        return -1;
    }

    /* Send header */
    result = LinSendHeader(frame->id);
    if (result != 0) {
        return -1;
    }

    /* Small delay between header and response */
    LinDelayUs(100);

    /* Send response */
    result = LinSendResponse(frame->data, frame->length);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive complete LIN frame
 * @param[in] : expectedLength - Expected data length
 * @param[out]: frame - Pointer to store received frame
 * @return    : 0 if success, -1 if error
 * @note      : Receives header and response, validates checksum
 *****************************************************************************/
int32_t LinReceiveFrame(LinFrame_t* frame, uint8_t expectedLength)
{
    uint8_t syncByte;
    uint8_t protectedId;
    int32_t result;
    uint32_t timeout;

    if (frame == NULL || expectedLength == 0 || expectedLength > LIN_MAX_DATA_LENGTH) {
        return -1;
    }

    /* Wait for sync byte */
    timeout = 100000;
    while (LinRxBufferHasData() == 0 && timeout > 0) {
        LinDelayUs(10);
        timeout--;
    }

    if (timeout == 0) {
        return -1; /* Timeout */
    }

    result = LinReadByte(&syncByte);
    if (result != 0 || syncByte != LIN_SYNC_BYTE) {
        return -1;
    }

    /* Receive protected ID */
    timeout = 10000;
    while (LinRxBufferHasData() == 0 && timeout > 0) {
        LinDelayUs(10);
        timeout--;
    }

    if (timeout == 0) {
        return -1; /* Timeout */
    }

    result = LinReadByte(&protectedId);
    if (result != 0) {
        return -1;
    }

    /* Validate protected ID */
    if (LinValidateProtectedId(protectedId) == 0) {
        return -1; /* Invalid parity */
    }

    frame->id = protectedId & 0x3F;
    g_currentId = frame->id;

    /* Receive response */
    frame->length = expectedLength;
    result = LinReceiveResponse(frame->data, expectedLength);
    if (result < 0) {
        return -1;
    }

    frame->checksumType = g_linConfig.defaultChecksumType;

    return 0;
}

/******************************************************************************
 * @brief     : Set LIN baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Common rates: 9600, 19200 (default), 20000
 *****************************************************************************/
int32_t LinSetBaudRate(uint32_t baudRate)
{
    int32_t result;

    result = LinConfigureUart(baudRate);
    if (result != 0) {
        return -1;
    }

    g_linConfig.baudRate = baudRate;
    return 0;
}

/******************************************************************************
 * @brief     : Set LIN protocol version
 * @param[in] : version - LIN protocol version
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Affects checksum type and frame handling
 *****************************************************************************/
int32_t LinSetVersion(LinVersion_t version)
{
    g_linConfig.version = version;

    /* LIN 1.3 uses classic checksum by default */
    if (version == LIN_VERSION_1_3) {
        g_linConfig.defaultChecksumType = LIN_CHECKSUM_CLASSIC;
    } else {
        /* LIN 2.0 and 2.1 use enhanced checksum */
        g_linConfig.defaultChecksumType = LIN_CHECKSUM_ENHANCED;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set default checksum type
 * @param[in] : checksumType - Checksum type to use
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : LIN 2.x uses enhanced checksum by default
 *****************************************************************************/
int32_t LinSetChecksumType(LinChecksumType_t checksumType)
{
    g_linConfig.defaultChecksumType = checksumType;
    return 0;
}

/******************************************************************************
 * @brief     : Get current LIN configuration
 * @param[in] : None
 * @param[out]: config - Pointer to store current configuration
 * @return    : 0 if success, -1 if error
 * @note      : Returns current LIN settings
 *****************************************************************************/
int32_t LinGetConfig(LinConfig_t* config)
{
    if (config == NULL) {
        return -1;
    }

    if (memcpy_s(config, sizeof(LinConfig_t), &g_linConfig, sizeof(LinConfig_t)) != EOK) {
        return -1;
    }
    return 0;
}

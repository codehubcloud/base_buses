#ifndef LIN_H
#define LIN_H

#include <stdbool.h>
#include <stdint.h>


#define LIN_DEFAULT_BAUDRATE 19200
#define LIN_MAX_DATA_LENGTH 8
#define LIN_SYNC_BYTE 0x55

/* LIN protocol versions */
typedef enum { LIN_VERSION_1_3 = 0, LIN_VERSION_2_0, LIN_VERSION_2_1 } LinVersion_t;

/* LIN checksum types */
typedef enum {
    LIN_CHECKSUM_CLASSIC = 0, /* Classic checksum (data only) */
    LIN_CHECKSUM_ENHANCED     /* Enhanced checksum (ID + data) */
} LinChecksumType_t;

/* LIN frame structure */
typedef struct {
    uint8_t id;     /* Protected ID (6-bit ID + 2-bit parity) */
    uint8_t data[LIN_MAX_DATA_LENGTH];
    uint8_t length; /* Data length (1-8 bytes) */
    uint8_t checksum;
    LinChecksumType_t checksumType;
} LinFrame_t;

/* LIN configuration structure */
typedef struct {
    uint32_t baudRate;
    LinVersion_t version;
    LinChecksumType_t defaultChecksumType;
} LinConfig_t;

/******************************************************************************
 * @brief     : Initialize LIN peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures LIN with 19200 baud rate and LIN 2.0 protocol
 *****************************************************************************/
int32_t LinInit(void);

/******************************************************************************
 * @brief     : Initialize LIN peripheral with custom configuration
 * @param[in] : config - Pointer to LIN configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Allows custom baud rate and protocol version
 *****************************************************************************/
int32_t LinInitWithConfig(const LinConfig_t* config);

/******************************************************************************
 * @brief     : Deinitialize LIN peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables LIN and releases resources
 *****************************************************************************/
int32_t LinDeinit(void);

/******************************************************************************
 * @brief     : Send LIN break field
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends 13-bit dominant (low) break signal
 *****************************************************************************/
int32_t LinSendBreak(void);

/******************************************************************************
 * @brief     : Send LIN frame header (break + sync + ID)
 * @param[in] : id - LIN identifier (0-63)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Master function - sends break, sync byte, and protected ID
 *****************************************************************************/
int32_t LinSendHeader(uint8_t id);

/******************************************************************************
 * @brief     : Send LIN response (data + checksum)
 * @param[in] : data - Pointer to data buffer, length - Number of bytes (1-8)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Can be used by master or slave to respond to header
 *****************************************************************************/
int32_t LinSendResponse(const uint8_t* data, uint8_t length);

/******************************************************************************
 * @brief     : Receive LIN response (data + checksum)
 * @param[in] : length - Expected number of data bytes (1-8)
 * @param[out]: data - Buffer to store received data
 * @return    : Number of bytes received, -1 if error
 * @note      : Validates checksum and returns data if valid
 *****************************************************************************/
int32_t LinReceiveResponse(uint8_t* data, uint8_t length);

/******************************************************************************
 * @brief     : Send complete LIN frame (master mode)
 * @param[in] : frame - Pointer to LIN frame structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends header and response as master
 *****************************************************************************/
int32_t LinSendFrame(const LinFrame_t* frame);

/******************************************************************************
 * @brief     : Receive complete LIN frame
 * @param[in] : expectedLength - Expected data length
 * @param[out]: frame - Pointer to store received frame
 * @return    : 0 if success, -1 if error
 * @note      : Receives header and response, validates checksum
 *****************************************************************************/
int32_t LinReceiveFrame(LinFrame_t* frame, uint8_t expectedLength);

/******************************************************************************
 * @brief     : Calculate LIN checksum
 * @param[in] : id - LIN identifier, data - Pointer to data buffer,
 *              length - Number of bytes, enhanced - Use enhanced checksum
 * @param[out]: None
 * @return    : Calculated checksum byte
 * @note      : Classic checksum uses data only, enhanced includes ID
 *****************************************************************************/
uint8_t LinCalculateChecksum(uint8_t id, const uint8_t* data, uint8_t length, bool enhanced);

/******************************************************************************
 * @brief     : Calculate protected identifier (ID with parity bits)
 * @param[in] : id - LIN identifier (0-63)
 * @param[out]: None
 * @return    : Protected ID with parity bits
 * @note      : Adds P0 and P1 parity bits to 6-bit ID
 *****************************************************************************/
uint8_t LinCalculateProtectedId(uint8_t id);

/******************************************************************************
 * @brief     : Validate protected identifier parity
 * @param[in] : protectedId - Protected ID byte with parity
 * @param[out]: None
 * @return    : 1 if valid, 0 if invalid
 * @note      : Checks P0 and P1 parity bits
 *****************************************************************************/
int32_t LinValidateProtectedId(uint8_t protectedId);

/******************************************************************************
 * @brief     : Set LIN baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Common rates: 9600, 19200 (default), 20000
 *****************************************************************************/
int32_t LinSetBaudRate(uint32_t baudRate);

/******************************************************************************
 * @brief     : Set LIN protocol version
 * @param[in] : version - LIN protocol version
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Affects checksum type and frame handling
 *****************************************************************************/
int32_t LinSetVersion(LinVersion_t version);

/******************************************************************************
 * @brief     : Set default checksum type
 * @param[in] : checksumType - Checksum type to use
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : LIN 2.x uses enhanced checksum by default
 *****************************************************************************/
int32_t LinSetChecksumType(LinChecksumType_t checksumType);

/******************************************************************************
 * @brief     : Get current LIN configuration
 * @param[in] : None
 * @param[out]: config - Pointer to store current configuration
 * @return    : 0 if success, -1 if error
 * @note      : Returns current LIN settings
 *****************************************************************************/
int32_t LinGetConfig(LinConfig_t* config);

#endif // LIN_H

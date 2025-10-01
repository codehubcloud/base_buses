#ifndef SDIO_H
#define SDIO_H

#include <stdint.h>

/* SDIO Bus Width */
typedef enum { SDIO_BUS_WIDTH_1BIT = 1, SDIO_BUS_WIDTH_4BIT = 4, SDIO_BUS_WIDTH_8BIT = 8 } SdioBusWidth_E;

/* SDIO Clock Speeds */
#define SDIO_CLOCK_DEFAULT 25000000    /* 25 MHz */
#define SDIO_CLOCK_HIGH_SPEED 50000000 /* 50 MHz */

/* SDIO Response Types */
typedef enum {
    SDIO_RESPONSE_NONE = 0,  /* No response */
    SDIO_RESPONSE_SHORT = 1, /* R1, R3, R4, R5, R6, R7 (48 bits) */
    SDIO_RESPONSE_LONG = 2   /* R2 (136 bits) */
} SdioResponseType_E;

/* SDIO Command Structure */
typedef struct {
    uint8_t cmdIndex;                /* Command index (0-63) */
    uint32_t argument;               /* Command argument */
    SdioResponseType_E responseType; /* Expected response type */
    uint8_t dataPresentFlag;         /* Data transfer expected */
} SdioCommand_T;

/* SDIO Response Structure */
typedef struct {
    uint32_t response[4]; /* Response data (up to 136 bits) */
    uint8_t crc7;         /* CRC7 checksum */
} SdioResponse_T;

/* Common SDIO Commands */
#define SDIO_CMD0_GO_IDLE_STATE 0
#define SDIO_CMD2_ALL_SEND_CID 2
#define SDIO_CMD3_SEND_RELATIVE_ADDR 3
#define SDIO_CMD5_IO_SEND_OP_COND 5
#define SDIO_CMD6_SWITCH_FUNC 6
#define SDIO_CMD7_SELECT_CARD 7
#define SDIO_CMD8_SEND_IF_COND 8
#define SDIO_CMD9_SEND_CSD 9
#define SDIO_CMD12_STOP_TRANSMISSION 12
#define SDIO_CMD13_SEND_STATUS 13
#define SDIO_CMD16_SET_BLOCKLEN 16
#define SDIO_CMD17_READ_SINGLE_BLOCK 17
#define SDIO_CMD18_READ_MULTIPLE_BLOCK 18
#define SDIO_CMD24_WRITE_BLOCK 24
#define SDIO_CMD25_WRITE_MULTIPLE_BLOCK 25
#define SDIO_CMD52_IO_RW_DIRECT 52
#define SDIO_CMD53_IO_RW_EXTENDED 53
#define SDIO_CMD55_APP_CMD 55

/* SDIO Block Size */
#define SDIO_DEFAULT_BLOCK_SIZE 512

/******************************************************************************
 * @brief     : Initialize SDIO peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SDIO with 1-bit mode and default 25MHz clock
 *****************************************************************************/
int32_t SdioInit(void);

/******************************************************************************
 * @brief     : Deinitialize SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables SDIO peripheral and releases resources
 *****************************************************************************/
int32_t SdioDeinit(void);

/******************************************************************************
 * @brief     : Send SDIO command
 * @param[in] : cmd - Command index (0-63), arg - Command argument
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends command with CRC7 checksum
 *****************************************************************************/
int32_t SdioSendCommand(uint8_t cmd, uint32_t arg);

/******************************************************************************
 * @brief     : Read SDIO response
 * @param[in] : responseType - Type of response expected (NONE/SHORT/LONG)
 * @param[out]: response - Pointer to response buffer (up to 4 words)
 * @return    : 0 if success, -1 if error
 * @note      : Reads and validates response with CRC7 check
 *****************************************************************************/
int32_t SdioReadResponse(uint32_t* response, SdioResponseType_E responseType);

/******************************************************************************
 * @brief     : Read data from SDIO
 * @param[in] : length - Number of bytes to read
 * @param[out]: data - Buffer to store received data
 * @return    : 0 if success, -1 if error
 * @note      : Reads data with CRC16 validation
 *****************************************************************************/
int32_t SdioReadData(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Write data to SDIO
 * @param[in] : data - Pointer to data buffer, length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes data with CRC16 checksum
 *****************************************************************************/
int32_t SdioWriteData(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Set SDIO bus width
 * @param[in] : width - Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures data bus width for parallel transfer
 *****************************************************************************/
int32_t SdioSetBusWidth(uint8_t width);

/******************************************************************************
 * @brief     : Set SDIO clock speed
 * @param[in] : speed - Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SDIO clock frequency
 *****************************************************************************/
int32_t SdioSetClockSpeed(uint32_t speed);

/******************************************************************************
 * @brief     : Calculate CRC7 checksum for SDIO commands
 * @param[in] : data - Pointer to data buffer, length - Number of bytes
 * @param[out]: None
 * @return    : CRC7 checksum value (7 bits)
 * @note      : Used for command and response validation
 *****************************************************************************/
uint8_t SdioCalculateCRC7(uint8_t* data, uint8_t length);

/******************************************************************************
 * @brief     : Calculate CRC16 checksum for SDIO data
 * @param[in] : data - Pointer to data buffer, length - Number of bytes
 * @param[out]: None
 * @return    : CRC16 checksum value
 * @note      : Used for data block validation
 *****************************************************************************/
uint16_t SdioCalculateCRC16(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Read single block from SDIO device
 * @param[in] : blockAddr - Block address, blockSize - Size of block in bytes
 * @param[out]: data - Buffer to store block data
 * @return    : 0 if success, -1 if error
 * @note      : Reads a single data block with CMD17
 *****************************************************************************/
int32_t SdioReadBlock(uint32_t blockAddr, uint8_t* data, uint32_t blockSize);

/******************************************************************************
 * @brief     : Write single block to SDIO device
 * @param[in] : blockAddr - Block address, data - Data to write, blockSize - Size of block
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes a single data block with CMD24
 *****************************************************************************/
int32_t SdioWriteBlock(uint32_t blockAddr, uint8_t* data, uint32_t blockSize);

#endif // SDIO_H

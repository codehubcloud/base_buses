#ifndef CANFD_H
#define CANFD_H

#include <stdint.h>

/* CAN FD Default Bit Rates */
#define CANFD_DEFAULT_NOMINAL_BITRATE 500000 /* Nominal bit rate: 500 kbps */
#define CANFD_DEFAULT_DATA_BITRATE 2000000   /* Data phase bit rate: 2 Mbps */

/* CAN FD Maximum Data Length */
#define CANFD_MAX_DATA_LENGTH 64

/* CAN FD Frame Type Flags */
#define CANFD_FRAME_TYPE_CLASSIC 0x00 /* Classic CAN frame (up to 8 bytes) */
#define CANFD_FRAME_TYPE_FD 0x01      /* CAN FD frame (up to 64 bytes) */

/* CAN ID Type Flags */
#define CANFD_ID_TYPE_STANDARD 0x00 /* Standard 11-bit ID */
#define CANFD_ID_TYPE_EXTENDED 0x01 /* Extended 29-bit ID */

/* CAN FD BRS Flag (Bit Rate Switch) */
#define CANFD_BRS_DISABLED 0x00 /* No bit rate switching */
#define CANFD_BRS_ENABLED 0x01  /* Enable bit rate switching */

/* CAN FD ESI Flag (Error State Indicator) */
#define CANFD_ESI_ERROR_ACTIVE 0x00  /* Transmitter is error active */
#define CANFD_ESI_ERROR_PASSIVE 0x01 /* Transmitter is error passive */

/******************************************************************************
 * CAN FD Frame Structure
 *****************************************************************************/
typedef struct {
    uint32_t id;                         /* CAN message ID (11-bit or 29-bit) */
    uint8_t data[CANFD_MAX_DATA_LENGTH]; /* Data payload */
    uint8_t dataLength;                  /* Number of data bytes (0-64) */
    uint8_t frameType;                   /* CANFD_FRAME_TYPE_CLASSIC or CANFD_FRAME_TYPE_FD */
    uint8_t idType;                      /* CANFD_ID_TYPE_STANDARD or CANFD_ID_TYPE_EXTENDED */
    uint8_t brsFlag;                     /* CANFD_BRS_DISABLED or CANFD_BRS_ENABLED */
    uint8_t esiFlag;                     /* CANFD_ESI_ERROR_ACTIVE or CANFD_ESI_ERROR_PASSIVE */
} CanFdFrame;

/******************************************************************************
 * CAN FD Error Counter Structure
 *****************************************************************************/
typedef struct {
    uint8_t txErrorCount; /* Transmit error counter */
    uint8_t rxErrorCount; /* Receive error counter */
} CanFdErrorCount;

/******************************************************************************
 * @brief     : Initialize CAN FD peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures CAN FD with default nominal and data bit rates
 *****************************************************************************/
int32_t CanFdInit(void);

/******************************************************************************
 * @brief     : Deinitialize CAN FD peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Stops CAN FD operation and releases resources
 *****************************************************************************/
int32_t CanFdDeinit(void);

/******************************************************************************
 * @brief     : Send CAN FD frame
 * @param[in] : frame - Pointer to CAN FD frame structure to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t CanFdSendFrame(const CanFdFrame* frame);

/******************************************************************************
 * @brief     : Receive CAN FD frame
 * @param[in] : None
 * @param[out]: frame - Pointer to CAN FD frame structure to store received data
 * @return    : 0 if success, -1 if error or no data available
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t CanFdReceiveFrame(CanFdFrame* frame);

/******************************************************************************
 * @brief     : Set CAN FD bit rates
 * @param[in] : nominalBitRate - Nominal phase bit rate in bps
 * @param[in] : dataBitRate - Data phase bit rate in bps
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the bit rate registers for both phases
 *****************************************************************************/
int32_t CanFdSetBitRate(uint32_t nominalBitRate, uint32_t dataBitRate);

/******************************************************************************
 * @brief     : Configure CAN FD acceptance filter
 * @param[in] : filterId - Filter ID value
 * @param[in] : filterMask - Filter mask value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures hardware filter to accept specific message IDs
 *****************************************************************************/
int32_t CanFdSetFilter(uint32_t filterId, uint32_t filterMask);

/******************************************************************************
 * @brief     : Get CAN FD error counters
 * @param[in] : None
 * @param[out]: errorCount - Pointer to structure to store error counters
 * @return    : 0 if success, -1 if error
 * @note      : Reads current TX and RX error counter values
 *****************************************************************************/
int32_t CanFdGetErrorCount(CanFdErrorCount* errorCount);

/******************************************************************************
 * @brief     : Enable CAN FD interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t CanFdEnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable CAN FD interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all CAN FD interrupts
 *****************************************************************************/
int32_t CanFdDisableInterrupts(void);

#endif // CANFD_H

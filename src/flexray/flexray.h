#ifndef FLEXRAY_H
#define FLEXRAY_H

#include <stdint.h>

/* FlexRay Protocol Constants */
#define FLEXRAY_MAX_PAYLOAD_LENGTH      254
#define FLEXRAY_HEADER_LENGTH           5
#define FLEXRAY_CRC_LENGTH              3
#define FLEXRAY_MAX_FRAME_LENGTH        (FLEXRAY_HEADER_LENGTH + FLEXRAY_MAX_PAYLOAD_LENGTH + FLEXRAY_CRC_LENGTH)
#define FLEXRAY_MAX_SLOTS               1023
#define FLEXRAY_DEFAULT_BITRATE         10000000  /* 10 Mbps */

/* FlexRay Channel Selection */
#define FLEXRAY_CHANNEL_A               0x01
#define FLEXRAY_CHANNEL_B               0x02
#define FLEXRAY_CHANNEL_AB              0x03

/* FlexRay Communication Status */
#define FLEXRAY_STATUS_UNINITIALIZED    0x00
#define FLEXRAY_STATUS_READY            0x01
#define FLEXRAY_STATUS_ACTIVE           0x02
#define FLEXRAY_STATUS_HALT             0x03
#define FLEXRAY_STATUS_ERROR            0x04

/* FlexRay Segment Types */
#define FLEXRAY_SEGMENT_STATIC          0x00
#define FLEXRAY_SEGMENT_DYNAMIC         0x01

/* FlexRay Frame Structure */
typedef struct {
    uint16_t slotId;                    /* Slot ID (0-1023) */
    uint8_t channel;                    /* Channel selection (A, B, or AB) */
    uint8_t payloadLength;              /* Payload length in words (0-127) */
    uint16_t headerCrc;                 /* Header CRC */
    uint8_t cycleCount;                 /* Cycle counter */
    uint8_t payload[FLEXRAY_MAX_PAYLOAD_LENGTH];
    uint32_t frameCrc;                  /* 24-bit Frame CRC */
} FlexRayFrame_t;

/* FlexRay Slot Configuration */
typedef struct {
    uint16_t slotId;                    /* Slot ID */
    uint8_t channel;                    /* Channel selection */
    uint16_t payloadLength;             /* Payload length in bytes */
    uint8_t segmentType;                /* Static or dynamic segment */
    uint8_t cycleRepetition;            /* Cycle repetition (1, 2, 4, 8, 16, 32, 64) */
} FlexRaySlotConfig_t;

/* FlexRay Communication Status Structure */
typedef struct {
    uint8_t state;                      /* Communication state */
    uint8_t syncState;                  /* Synchronization state */
    uint16_t slotCounter;               /* Current slot counter */
    uint8_t cycleCounter;               /* Current cycle counter */
    uint32_t errorFlags;                /* Error flags */
    uint16_t txSlotCount;               /* Number of transmitted slots */
    uint16_t rxSlotCount;               /* Number of received slots */
} FlexRayStatus_t;

/******************************************************************************
 * @brief     : Initialize FlexRay controller with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures FlexRay controller, timing parameters, and buffers
 *****************************************************************************/
int32_t FlexRayInit(void);

/******************************************************************************
 * @brief     : Deinitialize FlexRay controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Stops communication and releases resources
 *****************************************************************************/
int32_t FlexRayDeinit(void);

/******************************************************************************
 * @brief     : Configure a FlexRay slot
 * @param[in] : slotId - Slot identifier (0-1023)
 * @param[in] : channel - Channel selection (FLEXRAY_CHANNEL_A/B/AB)
 * @param[in] : payloadLength - Payload length in bytes (0-254)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before starting communication
 *****************************************************************************/
int32_t FlexRayConfigureSlot(uint16_t slotId, uint8_t channel, uint16_t payloadLength);

/******************************************************************************
 * @brief     : Send a FlexRay frame in configured slot
 * @param[in] : slotId - Slot identifier (0-1023)
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send (0-254)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Frame is transmitted in the specified slot during next cycle
 *****************************************************************************/
int32_t FlexRaySendFrame(uint16_t slotId, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive a FlexRay frame from configured slot
 * @param[in] : slotId - Slot identifier (0-1023)
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: data - Received data buffer
 * @return    : Number of bytes actually received, -1 if error, 0 if no data
 * @note      : Non-blocking function, returns immediately if no data
 *****************************************************************************/
int32_t FlexRayReceiveFrame(uint16_t slotId, uint8_t* data, uint16_t maxLength);

/******************************************************************************
 * @brief     : Start FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Initiates startup procedure and begins communication
 *****************************************************************************/
int32_t FlexRayStartCommunication(void);

/******************************************************************************
 * @brief     : Stop FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Halts communication, slots remain configured
 *****************************************************************************/
int32_t FlexRayStopCommunication(void);

/******************************************************************************
 * @brief     : Get FlexRay communication status
 * @param[in] : None
 * @param[out]: status - Pointer to status structure to fill
 * @return    : 0 if success, -1 if error
 * @note      : Returns current communication state and statistics
 *****************************************************************************/
int32_t FlexRayGetStatus(FlexRayStatus_t* status);

/******************************************************************************
 * @brief     : Set FlexRay bit rate
 * @param[in] : bitRate - Desired bit rate in bps (typically 10000000)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Should be called before FlexRayInit or after FlexRayDeinit
 *****************************************************************************/
int32_t FlexRaySetBitRate(uint32_t bitRate);

/******************************************************************************
 * @brief     : Configure FlexRay timing parameters
 * @param[in] : cycleLength - Communication cycle length in macroticks
 * @param[in] : staticSlots - Number of static slots
 * @param[in] : dynamicSlots - Number of dynamic slots
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Advanced configuration, use with caution
 *****************************************************************************/
int32_t FlexRayConfigureTiming(uint16_t cycleLength, uint16_t staticSlots, uint16_t dynamicSlots);

/******************************************************************************
 * @brief     : Enable FlexRay interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables slot and error interrupts
 *****************************************************************************/
int32_t FlexRayEnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable FlexRay interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all FlexRay interrupts
 *****************************************************************************/
int32_t FlexRayDisableInterrupts(void);

#endif // FLEXRAY_H

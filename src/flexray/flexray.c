#include <string.h>
#include "flexray.h"
#include "flexray_hal.h"
#include "securec.h"

/* Internal state variables */
static uint8_t g_flexrayInitialized = 0;
static FlexRayStatus_t g_flexrayStatus = {0};
static FlexRaySlotConfig_t g_slotConfigs[FLEXRAY_MAX_SLOTS] = {0};

/* Internal function prototypes */
static uint16_t FlexRayCalculateHeaderCrc(FlexRayFrame_t* frame);
static uint32_t FlexRayCalculateFrameCrc(FlexRayFrame_t* frame);
static int32_t FlexRayValidateSlotConfig(uint16_t slotId, uint8_t channel, uint16_t payloadLength);

/******************************************************************************
 * @brief     : Initialize FlexRay controller with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures FlexRay controller, timing parameters, and buffers
 *****************************************************************************/
int32_t FlexRayInit(void)
{
    int32_t result = 0;

    if (g_flexrayInitialized) {
        return 0;  /* Already initialized */
    }

    /* Enable clock and configure GPIO */
    result = FlexRayEnableClock();
    if (result != 0) {
        return -1;
    }

    result = FlexRayConfigureGpio();
    if (result != 0) {
        return -1;
    }

    /* Initialize controller with default parameters */
    result = FlexRayHalInit();
    if (result != 0) {
        return -1;
    }

    /* Set default bit rate */
    result = FlexRaySetBitRate(FLEXRAY_DEFAULT_BITRATE);
    if (result != 0) {
        return -1;
    }

    /* Initialize status structure */
    if (memset_s(&g_flexrayStatus, sizeof(g_flexrayStatus), 0, sizeof(g_flexrayStatus)) != EOK) {
        return -1;
    }
    g_flexrayStatus.state = FLEXRAY_STATUS_READY;

    /* Clear slot configurations */
    if (memset_s(g_slotConfigs, sizeof(g_slotConfigs), 0, sizeof(g_slotConfigs)) != EOK) {
        return -1;
    }

    g_flexrayInitialized = 1;

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize FlexRay controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Stops communication and releases resources
 *****************************************************************************/
int32_t FlexRayDeinit(void)
{
    if (!g_flexrayInitialized) {
        return 0;
    }

    /* Stop communication if active */
    if (g_flexrayStatus.state == FLEXRAY_STATUS_ACTIVE) {
        FlexRayStopCommunication();
    }

    /* Deinitialize HAL */
    FlexRayHalDeinit();

    g_flexrayInitialized = 0;
    g_flexrayStatus.state = FLEXRAY_STATUS_UNINITIALIZED;

    return 0;
}

/******************************************************************************
 * @brief     : Configure a FlexRay slot
 * @param[in] : slotId - Slot identifier (0-1023)
 * @param[in] : channel - Channel selection (FLEXRAY_CHANNEL_A/B/AB)
 * @param[in] : payloadLength - Payload length in bytes (0-254)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before starting communication
 *****************************************************************************/
int32_t FlexRayConfigureSlot(uint16_t slotId, uint8_t channel, uint16_t payloadLength)
{
    if (!g_flexrayInitialized) {
        return -1;
    }

    /* Validate parameters */
    if (FlexRayValidateSlotConfig(slotId, channel, payloadLength) != 0) {
        return -1;
    }

    /* Cannot configure slots while communication is active */
    if (g_flexrayStatus.state == FLEXRAY_STATUS_ACTIVE) {
        return -1;
    }

    /* Store slot configuration */
    g_slotConfigs[slotId].slotId = slotId;
    g_slotConfigs[slotId].channel = channel;
    g_slotConfigs[slotId].payloadLength = payloadLength;
    g_slotConfigs[slotId].segmentType = FLEXRAY_SEGMENT_STATIC;  /* Default to static */
    g_slotConfigs[slotId].cycleRepetition = 1;  /* Transmit every cycle */

    /* Configure slot in hardware */
    return FlexRayHalConfigureSlot(slotId, channel, payloadLength);
}

/******************************************************************************
 * @brief     : Send a FlexRay frame in configured slot
 * @param[in] : slotId - Slot identifier (0-1023)
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send (0-254)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Frame is transmitted in the specified slot during next cycle
 *****************************************************************************/
int32_t FlexRaySendFrame(uint16_t slotId, uint8_t* data, uint16_t length)
{
    FlexRayFrame_t frame;

    if (!g_flexrayInitialized || data == NULL) {
        return -1;
    }

    /* Validate slot ID and length */
    if (slotId >= FLEXRAY_MAX_SLOTS || length > FLEXRAY_MAX_PAYLOAD_LENGTH) {
        return -1;
    }

    /* Check if slot is configured */
    if (g_slotConfigs[slotId].slotId != slotId) {
        return -1;  /* Slot not configured */
    }

    /* Verify length matches configured payload length */
    if (length > g_slotConfigs[slotId].payloadLength) {
        return -1;
    }

    /* Prepare frame */
    if (memset_s(&frame, sizeof(frame), 0, sizeof(frame)) != EOK) {
        return -1;
    }
    frame.slotId = slotId;
    frame.channel = g_slotConfigs[slotId].channel;
    frame.payloadLength = (uint8_t)(length / 2);  /* Convert bytes to words */
    frame.cycleCount = g_flexrayStatus.cycleCounter;

    /* Copy payload data */
    if (length > 0) {
        if (memcpy_s(frame.payload, sizeof(frame.payload), data, length) != EOK) {
            return -1;
        }
    }

    /* Calculate CRCs */
    frame.headerCrc = FlexRayCalculateHeaderCrc(&frame);
    frame.frameCrc = FlexRayCalculateFrameCrc(&frame);

    /* Send frame via HAL */
    if (FlexRayHalSendFrame(slotId, &frame) == 0) {
        g_flexrayStatus.txSlotCount++;
        return 0;
    }

    return -1;
}

/******************************************************************************
 * @brief     : Receive a FlexRay frame from configured slot
 * @param[in] : slotId - Slot identifier (0-1023)
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: data - Received data buffer
 * @return    : Number of bytes actually received, -1 if error, 0 if no data
 * @note      : Non-blocking function, returns immediately if no data
 *****************************************************************************/
int32_t FlexRayReceiveFrame(uint16_t slotId, uint8_t* data, uint16_t maxLength)
{
    FlexRayFrame_t frame;
    int32_t result;

    if (!g_flexrayInitialized || data == NULL) {
        return -1;
    }

    /* Validate slot ID */
    if (slotId >= FLEXRAY_MAX_SLOTS || maxLength > FLEXRAY_MAX_PAYLOAD_LENGTH) {
        return -1;
    }

    /* Check if slot is configured */
    if (g_slotConfigs[slotId].slotId != slotId) {
        return -1;  /* Slot not configured */
    }

    /* Check if data is available */
    if (FlexRayHalRxSlotHasData(slotId) == 0) {
        return 0;  /* No data available */
    }

    /* Receive frame from HAL */
    if (memset_s(&frame, sizeof(frame), 0, sizeof(frame)) != EOK) {
        return -1;
    }
    result = FlexRayHalReceiveFrame(slotId, &frame);
    if (result < 0) {
        return -1;
    }

    /* Verify CRCs */
    if (frame.headerCrc != FlexRayCalculateHeaderCrc(&frame)) {
        g_flexrayStatus.errorFlags |= 0x01;  /* Header CRC error */
        return -1;
    }

    if (frame.frameCrc != FlexRayCalculateFrameCrc(&frame)) {
        g_flexrayStatus.errorFlags |= 0x02;  /* Frame CRC error */
        return -1;
    }

    /* Copy payload to output buffer */
    uint16_t payloadBytes = frame.payloadLength * 2;  /* Convert words to bytes */
    if (payloadBytes > maxLength) {
        payloadBytes = maxLength;
    }

    if (payloadBytes > 0) {
        if (memcpy_s(data, maxLength, frame.payload, payloadBytes) != EOK) {
            return -1;
        }
    }

    g_flexrayStatus.rxSlotCount++;
    return (int32_t)payloadBytes;
}

/******************************************************************************
 * @brief     : Start FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Initiates startup procedure and begins communication
 *****************************************************************************/
int32_t FlexRayStartCommunication(void)
{
    if (!g_flexrayInitialized) {
        return -1;
    }

    if (g_flexrayStatus.state == FLEXRAY_STATUS_ACTIVE) {
        return 0;  /* Already active */
    }

    /* Start communication via HAL */
    if (FlexRayHalStartCommunication() != 0) {
        g_flexrayStatus.state = FLEXRAY_STATUS_ERROR;
        return -1;
    }

    g_flexrayStatus.state = FLEXRAY_STATUS_ACTIVE;
    g_flexrayStatus.syncState = 1;  /* Synchronized */
    g_flexrayStatus.slotCounter = 0;
    g_flexrayStatus.cycleCounter = 0;
    g_flexrayStatus.errorFlags = 0;

    return 0;
}

/******************************************************************************
 * @brief     : Stop FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Halts communication, slots remain configured
 *****************************************************************************/
int32_t FlexRayStopCommunication(void)
{
    if (!g_flexrayInitialized) {
        return -1;
    }

    if (g_flexrayStatus.state != FLEXRAY_STATUS_ACTIVE) {
        return 0;  /* Not active */
    }

    /* Stop communication via HAL */
    if (FlexRayHalStopCommunication() != 0) {
        return -1;
    }

    g_flexrayStatus.state = FLEXRAY_STATUS_HALT;
    g_flexrayStatus.syncState = 0;

    return 0;
}

/******************************************************************************
 * @brief     : Get FlexRay communication status
 * @param[in] : None
 * @param[out]: status - Pointer to status structure to fill
 * @return    : 0 if success, -1 if error
 * @note      : Returns current communication state and statistics
 *****************************************************************************/
int32_t FlexRayGetStatus(FlexRayStatus_t* status)
{
    if (!g_flexrayInitialized || status == NULL) {
        return -1;
    }

    /* Update status from HAL */
    FlexRayHalUpdateStatus(&g_flexrayStatus);

    /* Copy status to output */
    if (memcpy_s(status, sizeof(FlexRayStatus_t), &g_flexrayStatus, sizeof(FlexRayStatus_t)) != EOK) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set FlexRay bit rate
 * @param[in] : bitRate - Desired bit rate in bps (typically 10000000)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Should be called before FlexRayInit or after FlexRayDeinit
 *****************************************************************************/
int32_t FlexRaySetBitRate(uint32_t bitRate)
{
    /* Validate bit rate (FlexRay standard is 10 Mbps) */
    if (bitRate != FLEXRAY_DEFAULT_BITRATE) {
        return -1;  /* FlexRay only supports 10 Mbps */
    }

    return FlexRayHalConfigureBitRate(bitRate);
}

/******************************************************************************
 * @brief     : Configure FlexRay timing parameters
 * @param[in] : cycleLength - Communication cycle length in macroticks
 * @param[in] : staticSlots - Number of static slots
 * @param[in] : dynamicSlots - Number of dynamic slots
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Advanced configuration, use with caution
 *****************************************************************************/
int32_t FlexRayConfigureTiming(uint16_t cycleLength, uint16_t staticSlots, uint16_t dynamicSlots)
{
    if (!g_flexrayInitialized) {
        return -1;
    }

    /* Cannot configure timing while communication is active */
    if (g_flexrayStatus.state == FLEXRAY_STATUS_ACTIVE) {
        return -1;
    }

    /* Validate parameters */
    if (cycleLength == 0 || staticSlots + dynamicSlots > FLEXRAY_MAX_SLOTS) {
        return -1;
    }

    return FlexRayHalConfigureTiming(cycleLength, staticSlots, dynamicSlots);
}

/******************************************************************************
 * @brief     : Enable FlexRay interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables slot and error interrupts
 *****************************************************************************/
int32_t FlexRayEnableInterrupts(void)
{
    if (!g_flexrayInitialized) {
        return -1;
    }

    FlexRayHalEnableSlotInterrupt();
    FlexRayHalEnableErrorInterrupt();
    FlexRayHalConfigureNvic();

    return 0;
}

/******************************************************************************
 * @brief     : Disable FlexRay interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all FlexRay interrupts
 *****************************************************************************/
int32_t FlexRayDisableInterrupts(void)
{
    if (!g_flexrayInitialized) {
        return -1;
    }

    FlexRayHalDisableSlotInterrupt();
    FlexRayHalDisableErrorInterrupt();
    FlexRayHalUpdateNvic();

    return 0;
}

/******************************************************************************
 * @brief     : Calculate FlexRay header CRC
 * @param[in] : frame - Pointer to FlexRay frame
 * @param[out]: None
 * @return    : Calculated header CRC
 * @note      : Simplified CRC calculation (placeholder for actual algorithm)
 *****************************************************************************/
static uint16_t FlexRayCalculateHeaderCrc(FlexRayFrame_t* frame)
{
    /* Simplified CRC-11 calculation */
    /* In production, use FlexRay specification CRC-11 polynomial: 0x5A5 */
    uint16_t crc = 0x01A;  /* Initial value */
    uint16_t data = 0;

    data = (frame->slotId << 5) | (frame->payloadLength & 0x1F);
    crc ^= data;
    crc ^= (frame->channel << 8);

    return crc & 0x7FF;  /* 11-bit CRC */
}

/******************************************************************************
 * @brief     : Calculate FlexRay frame CRC
 * @param[in] : frame - Pointer to FlexRay frame
 * @param[out]: None
 * @return    : Calculated frame CRC
 * @note      : Simplified CRC calculation (placeholder for actual algorithm)
 *****************************************************************************/
static uint32_t FlexRayCalculateFrameCrc(FlexRayFrame_t* frame)
{
    /* Simplified CRC-24 calculation */
    /* In production, use FlexRay specification CRC-24 polynomial: 0x5D6DCB */
    uint32_t crc = 0xFEDCBA;  /* Initial value */
    uint16_t payloadBytes = frame->payloadLength * 2;

    for (uint16_t i = 0; i < payloadBytes; i++) {
        crc ^= (uint32_t)frame->payload[i] << (16 - (i % 3) * 8);
    }

    return crc & 0xFFFFFF;  /* 24-bit CRC */
}

/******************************************************************************
 * @brief     : Validate slot configuration parameters
 * @param[in] : slotId - Slot identifier
 * @param[in] : channel - Channel selection
 * @param[in] : payloadLength - Payload length
 * @param[out]: None
 * @return    : 0 if valid, -1 if invalid
 * @note      : Internal validation function
 *****************************************************************************/
static int32_t FlexRayValidateSlotConfig(uint16_t slotId, uint8_t channel, uint16_t payloadLength)
{
    /* Validate slot ID */
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return -1;
    }

    /* Validate channel */
    if (channel != FLEXRAY_CHANNEL_A &&
        channel != FLEXRAY_CHANNEL_B &&
        channel != FLEXRAY_CHANNEL_AB) {
        return -1;
    }

    /* Validate payload length */
    if (payloadLength > FLEXRAY_MAX_PAYLOAD_LENGTH) {
        return -1;
    }

    return 0;
}

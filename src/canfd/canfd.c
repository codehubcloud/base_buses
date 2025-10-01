#include <string.h>
#include "canfd.h"
#include "canfd_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize CAN FD peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures CAN FD with default nominal and data bit rates
 *****************************************************************************/
int32_t CanFdInit(void)
{
    int32_t result = 0;

    result = CanFdEnableClock();
    if (result != 0) {
        return -1;
    }

    result = CanFdConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = CanFdSetBitRate(CANFD_DEFAULT_NOMINAL_BITRATE, CANFD_DEFAULT_DATA_BITRATE);
    if (result != 0) {
        return -1;
    }

    result = CanFdEnable();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize CAN FD peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Stops CAN FD operation and releases resources
 *****************************************************************************/
int32_t CanFdDeinit(void)
{
    int32_t result = 0;

    result = CanFdDisable();
    if (result != 0) {
        return -1;
    }

    result = CanFdDisableClock();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Send CAN FD frame
 * @param[in] : frame - Pointer to CAN FD frame structure to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer to be empty
 *****************************************************************************/
int32_t CanFdSendFrame(const CanFdFrame* frame)
{
    if (frame == NULL) {
        return -1;
    }

    if (frame->dataLength > CANFD_MAX_DATA_LENGTH) {
        return -1;
    }

    if ((frame->frameType == CANFD_FRAME_TYPE_CLASSIC) && (frame->dataLength > 8)) {
        return -1;
    }

    while (CanFdTxBufferEmpty() == 0) {
        /* Wait for TX buffer to be available */
    }

    if (CanFdWriteFrame(frame) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Receive CAN FD frame
 * @param[in] : None
 * @param[out]: frame - Pointer to CAN FD frame structure to store received data
 * @return    : 0 if success, -1 if error or no data available
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t CanFdReceiveFrame(CanFdFrame* frame)
{
    if (frame == NULL) {
        return -1;
    }

    if (CanFdRxBufferHasData() == 0) {
        return -1;
    }

    if (CanFdReadFrame(frame) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set CAN FD bit rates
 * @param[in] : nominalBitRate - Nominal phase bit rate in bps
 * @param[in] : dataBitRate - Data phase bit rate in bps
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Recalculates and sets the bit rate registers for both phases
 *****************************************************************************/
int32_t CanFdSetBitRate(uint32_t nominalBitRate, uint32_t dataBitRate)
{
    if (CanFdConfigureBitRate(nominalBitRate, dataBitRate) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Configure CAN FD acceptance filter
 * @param[in] : filterId - Filter ID value
 * @param[in] : filterMask - Filter mask value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures hardware filter to accept specific message IDs
 *****************************************************************************/
int32_t CanFdSetFilter(uint32_t filterId, uint32_t filterMask)
{
    if (CanFdConfigureFilter(filterId, filterMask) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Get CAN FD error counters
 * @param[in] : None
 * @param[out]: errorCount - Pointer to structure to store error counters
 * @return    : 0 if success, -1 if error
 * @note      : Reads current TX and RX error counter values
 *****************************************************************************/
int32_t CanFdGetErrorCount(CanFdErrorCount* errorCount)
{
    if (errorCount == NULL) {
        return -1;
    }

    if (CanFdReadErrorCount(errorCount) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Enable CAN FD interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables RX and TX interrupts and configures NVIC
 *****************************************************************************/
int32_t CanFdEnableInterrupts(void)
{
    CanFdEnableRxInterrupt();
    CanFdEnableTxInterrupt();
    CanFdConfigureNvic();
    return 0;
}

/******************************************************************************
 * @brief     : Disable CAN FD interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables all CAN FD interrupts
 *****************************************************************************/
int32_t CanFdDisableInterrupts(void)
{
    CanFdDisableRxInterrupt();
    CanFdDisableTxInterrupt();
    CanFdUpdateNvic();
    return 0;
}

#include <string.h>
#include "i3c.h"
#include "i3c_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize I3C peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I3C clock speed and mode
 *****************************************************************************/
int32_t I3cInit(void)
{
    int32_t result = 0;

    result = I3cEnableClock();
    if (result != 0) {
        return -1;
    }

    result = I3cConfigureGpio();
    if (result != 0) {
        return -1;
    }

    I3cSetClockSpeed(I3C_DEFAULT_CLOCK_SPEED);
    I3cEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Send data to I3C device
 * @param[in] : deviceAddr - I3C device address
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes data to I3C device
 *****************************************************************************/
int32_t I3cSendData(uint8_t deviceAddr, uint8_t* data, uint16_t length)
{
    int32_t result = 0;

    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    result = I3cGenerateStart();
    if (result != 0) {
        return -1;
    }

    result = I3cSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        I3cGenerateStop();
        return -1;
    }

    if (!I3cCheckAck()) {
        I3cGenerateStop();
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        result = I3cSendByte(data[i]);
        if (result != 0) {
            I3cGenerateStop();
            return -1;
        }

        if (!I3cCheckAck()) {
            I3cGenerateStop();
            return -1;
        }
    }

    I3cGenerateStop();
    return 0;
}

/******************************************************************************
 * @brief     : Read data from I3C device
 * @param[in] : deviceAddr - I3C device address
 * @param[in] : buffer - Pointer to buffer to store received data
 * @param[in] : length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads data from I3C device
 *****************************************************************************/
int32_t I3cReadData(uint8_t deviceAddr, uint8_t* buffer, uint16_t length)
{
    int32_t result = 0;

    if ((buffer == NULL) || (length == 0)) {
        return -1;
    }

    result = I3cGenerateStart();
    if (result != 0) {
        return -1;
    }

    result = I3cSendByte((deviceAddr << 1) | 0x01);
    if (result != 0) {
        I3cGenerateStop();
        return -1;
    }

    if (!I3cCheckAck()) {
        I3cGenerateStop();
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = I3cReadByte();

        if (i < (length - 1)) {
            I3cSendAck();
        } else {
            I3cSendNack();
        }
    }

    I3cGenerateStop();
    return 0;
}

/******************************************************************************
 * @brief     : Set I3C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I3C clock prescaler
 *****************************************************************************/
int32_t I3cSetClockSpeed(uint32_t clockSpeed)
{
    if (I3cConfigureClockSpeed(clockSpeed) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Check if I3C device is present
 * @param[in] : deviceAddr - I3C device address
 * @param[out]: None
 * @return    : 0 if device present, -1 if not present
 * @note      : Performs a simple address check
 *****************************************************************************/
int32_t I3cCheckDevice(uint8_t deviceAddr)
{
    int32_t result = 0;

    result = I3cGenerateStart();
    if (result != 0) {
        return -1;
    }

    result = I3cSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        I3cGenerateStop();
        return -1;
    }

    if (!I3cCheckAck()) {
        I3cGenerateStop();
        return -1;
    }

    I3cGenerateStop();
    return 0;
}
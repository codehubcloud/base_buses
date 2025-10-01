#include <string.h>
#include "i2c.h"
#include "i2c_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize I2C peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I2C clock speed and mode
 *****************************************************************************/
int32_t I2cInit(void)
{
    int32_t result = 0;

    result = I2cEnableClock();
    if (result != 0) {
        return -1;
    }

    result = I2cConfigureGpio();
    if (result != 0) {
        return -1;
    }

    I2cSetClockSpeed(I2C_DEFAULT_CLOCK_SPEED);
    I2cEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Send data to I2C device
 * @param[in] : deviceAddr - I2C device address, regAddr - Register address (can be NULL for direct write), data - Pointer to data buffer to
 *send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes data to I2C device
 *****************************************************************************/
int32_t I2cWriteData(uint8_t deviceAddr, uint8_t* regAddr, uint8_t* data, uint16_t length)
{
    int32_t result = 0;

    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    result = I2cGenerateStart();
    if (result != 0) {
        return -1;
    }

    result = I2cSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        I2cGenerateStop();
        return -1;
    }

    if (!I2cCheckAck()) {
        I2cGenerateStop();
        return -1;
    }

    if (regAddr != NULL) {
        result = I2cSendByte(*regAddr);
        if (result != 0) {
            I2cGenerateStop();
            return -1;
        }

        if (I2cCheckAck() == 0) {
            I2cGenerateStop();
            return -1;
        }
    }

    for (uint16_t i = 0; i < length; i++) {
        result = I2cSendByte(data[i]);
        if (result != 0) {
            I2cGenerateStop();
            return -1;
        }

        if (I2cCheckAck() == 0) {
            I2cGenerateStop();
            return -1;
        }
    }

    I2cGenerateStop();
    return 0;
}

/******************************************************************************
 * @brief     : Read data from I2C device
 * @param[in] : deviceAddr - I2C device address, regAddr - Register address (can be NULL for direct read), length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads data from I2C device
 *****************************************************************************/
int32_t I2cReadData(uint8_t deviceAddr, uint8_t* regAddr, uint8_t* buffer, uint16_t length)
{
    int32_t result = 0;

    if ((buffer == NULL) || (length == 0)) {
        return -1;
    }

    if (regAddr != NULL) {
        result = I2cGenerateStart();
        if (result != 0) {
            return -1;
        }

        result = I2cSendByte((deviceAddr << 1) | 0x00);
        if (result != 0) {
            I2cGenerateStop();
            return -1;
        }

        if (I2cCheckAck() == 0) {
            I2cGenerateStop();
            return -1;
        }

        result = I2cSendByte(*regAddr);
        if (result != 0) {
            I2cGenerateStop();
            return -1;
        }

        if (I2cCheckAck() == 0) {
            I2cGenerateStop();
            return -1;
        }

        result = I2cGenerateStart();
        if (result != 0) {
            I2cGenerateStop();
            return -1;
        }
    } else {
        result = I2cGenerateStart();
        if (result != 0) {
            return -1;
        }
    }

    result = I2cSendByte((deviceAddr << 1) | 0x01);
    if (result != 0) {
        I2cGenerateStop();
        return -1;
    }

    if (I2cCheckAck() == 0) {
        I2cGenerateStop();
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = I2cReadByte();

        if (i < (length - 1)) {
            I2cSendAck();
        } else {
            I2cSendNack();
        }
    }

    I2cGenerateStop();
    return 0;
}

/******************************************************************************
 * @brief     : Set I2C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I2C clock prescaler
 *****************************************************************************/
int32_t I2cSetClockSpeed(uint32_t clockSpeed)
{
    if (I2cConfigureClockSpeed(clockSpeed) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Check if I2C device is present
 * @param[in] : deviceAddr - I2C device address
 * @param[out]: None
 * @return    : 0 if device present, -1 if not present
 * @note      : Performs a simple address check
 *****************************************************************************/
int32_t I2cCheckDevice(uint8_t deviceAddr)
{
    int32_t result = 0;

    result = I2cGenerateStart();
    if (result != 0) {
        return -1;
    }

    result = I2cSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        I2cGenerateStop();
        return -1;
    }

    if (I2cCheckAck() == 0) {
        I2cGenerateStop();
        return -1;
    }

    I2cGenerateStop();
    return 0;
}
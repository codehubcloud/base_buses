#include <string.h>
#include "securec.h"
#include "smbus.h"
#include "smbus_hal.h"

/******************************************************************************
 * @brief     : Initialize SMBus peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SMBus clock speed and mode
 *****************************************************************************/
int32_t SmBusInit(void)
{
    int32_t result = 0;

    result = SmBusEnableClock();
    if (result != 0) {
        return -1;
    }

    result = SmBusConfigureGpio();
    if (result != 0) {
        return -1;
    }

    SmBusSetClockSpeed(SMBUS_DEFAULT_CLOCK_SPEED);
    SmBusEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Send data to SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte, data - Pointer to data buffer to send, length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes data to SMBus device
 *****************************************************************************/
int32_t SmBusSendData(uint8_t deviceAddr, uint8_t command, uint8_t* data, uint16_t length)
{
    int32_t result = 0;

    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    result = SmBusGenerateStart();
    if (result != 0) {
        return -1;
    }

    // Send device address with write bit
    result = SmBusSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    if (SmBusCheckAck() == 0) {
        SmBusGenerateStop();
        return -1;
    }

    // Send command
    result = SmBusSendByte(command);
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    if (SmBusCheckAck() == 0) {
        SmBusGenerateStop();
        return -1;
    }

    // Send data
    for (uint16_t i = 0; i < length; i++) {
        result = SmBusSendByte(data[i]);
        if (result != 0) {
            SmBusGenerateStop();
            return -1;
        }

        if (SmBusCheckAck() == 0) {
            SmBusGenerateStop();
            return -1;
        }
    }

    SmBusGenerateStop();
    return 0;
}

/******************************************************************************
 * @brief     : Read data from SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte, length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads data from SMBus device
 *****************************************************************************/
int32_t SmBusReadData(uint8_t deviceAddr, uint8_t command, uint8_t* buffer, uint16_t length)
{
    int32_t result = 0;

    if ((buffer == NULL) || (length == 0)) {
        return -1;
    }

    // First write the command
    result = SmBusGenerateStart();
    if (result != 0) {
        return -1;
    }

    // Send device address with write bit
    result = SmBusSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    if (SmBusCheckAck() == 0) {
        SmBusGenerateStop();
        return -1;
    }

    // Send command
    result = SmBusSendByte(command);
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    if (SmBusCheckAck() == 0) {
        SmBusGenerateStop();
        return -1;
    }

    // Generate repeated start
    result = SmBusGenerateStart();
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    // Send device address with read bit
    result = SmBusSendByte((deviceAddr << 1) | 0x01);
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    if (SmBusCheckAck() == 0) {
        SmBusGenerateStop();
        return -1;
    }

    // Read data
    for (uint16_t i = 0; i < length; i++) {
        buffer[i] = SmBusReadByteHal();

        if (i < (length - 1)) {
            SmBusSendAck();
        } else {
            SmBusSendNack();
        }
    }

    SmBusGenerateStop();
    return 0;
}

/******************************************************************************
 * @brief     : Read single byte from SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte
 * @param[out]: data - Pointer to store received byte
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that reads a single byte from SMBus device
 *****************************************************************************/
int32_t SmBusReadByte(uint8_t deviceAddr, uint8_t command, uint8_t* data)
{
    int32_t result = 0;

    if (data == NULL) {
        return -1;
    }

    result = SmBusReadData(deviceAddr, command, data, 1);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write single byte to SMBus device
 * @param[in] : deviceAddr - SMBus device address, command - Command byte, data - Byte to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that writes a single byte to SMBus device
 *****************************************************************************/
int32_t SmBusWriteByte(uint8_t deviceAddr, uint8_t command, uint8_t data)
{
    int32_t result = 0;

    result = SmBusSendData(deviceAddr, command, &data, 1);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set SMBus clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures SMBus clock prescaler
 *****************************************************************************/
int32_t SmBusSetClockSpeed(uint32_t clockSpeed)
{
    if (SmBusConfigureClockSpeed(clockSpeed) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Check if SMBus device is present
 * @param[in] : deviceAddr - SMBus device address
 * @param[out]: None
 * @return    : 0 if device present, -1 if not present
 * @note      : Performs a simple address check
 *****************************************************************************/
int32_t SmBusCheckDevice(uint8_t deviceAddr)
{
    int32_t result = 0;

    result = SmBusGenerateStart();
    if (result != 0) {
        return -1;
    }

    result = SmBusSendByte((deviceAddr << 1) | 0x00);
    if (result != 0) {
        SmBusGenerateStop();
        return -1;
    }

    if (SmBusCheckAck() == 0) {
        SmBusGenerateStop();
        return -1;
    }

    SmBusGenerateStop();
    return 0;
}
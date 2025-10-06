#include <math.h>
#include <string.h>
#include "pmbus.h"
#include "pmbus_hal.h"
#include "securec.h"


/******************************************************************************
 * @brief     : Initialize PMBus peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures PMBus based on SMBus/I2C
 *****************************************************************************/
int32_t PmBusInit(void)
{
    int32_t result = 0;

    result = PmBusEnableClock();
    if (result != 0) {
        return -1;
    }

    result = PmBusConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = PmBusSetClockSpeed(PMBUS_DEFAULT_CLOCK_SPEED);
    if (result != 0) {
        return -1;
    }

    PmBusEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize PMBus peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Releases PMBus resources
 *****************************************************************************/
int32_t PmBusDeinit(void)
{
    PmBusDisable();
    return 0;
}

/******************************************************************************
 * @brief     : Set PMBus page for multi-rail devices
 * @param[in] : deviceAddr --PMBus device address, page - Page number (0-15)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Selects the target rail/page for subsequent commands
 *****************************************************************************/
int32_t PmBusSetPage(uint8_t deviceAddr, uint8_t page)
{
    if (page >= PMBUS_MAX_PAGES) {
        return -1;
    }

    return PmBusWriteByte(deviceAddr, PMBUS_CMD_PAGE, page);
}

/******************************************************************************
 * @brief     : Convert LINEAR11 format to float
 * @param[in] : linear11 --16-bit LINEAR11 value (5-bit exponent, 11-bit mantissa)
 * @param[out]: None
 * @return    : Floating point value
 * @note      : LINEAR11 format: Y = mantissa * 2^exponent
 *****************************************************************************/
float PmBusLinear11ToFloat(uint16_t linear11)
{
    int16_t exponent = 0;
    int16_t mantissa = 0;
    float result = 0.0f;

    // Extract exponent (bits 15:11)
    exponent = (int16_t)((linear11 >> 11) & 0x1F);
    // Sign extend if negative
    if (exponent > 15) {
        exponent |= 0xFFE0;
    }

    // Extract mantissa (bits 10:0)
    mantissa = (int16_t)(linear11 & 0x7FF);
    // Sign extend if negative
    if (mantissa > 1023) {
        mantissa |= 0xF800;
    }

    // Calculate: Y = mantissa * 2^exponent
    result = (float)mantissa * powf(2.0f, (float)exponent);

    return result;
}

/******************************************************************************
 * @brief     : Convert LINEAR16 format to float
 * @param[in] : linear16 --16-bit LINEAR16 mantissa, exponent - 5-bit exponent from VOUT_MODE
 * @param[out]: None
 * @return    : Floating point value
 * @note      : LINEAR16 format: Y = mantissa * 2^exponent
 *****************************************************************************/
float PmBusLinear16ToFloat(uint16_t linear16, int8_t exponent)
{
    int16_t mantissa = 0;
    float result = 0.0f;

    // Sign extend exponent if necessary
    if (exponent > 15) {
        exponent |= 0xE0;
    }

    // Mantissa is the full 16-bit value (signed)
    mantissa = (int16_t)linear16;

    // Calculate: Y = mantissa * 2^exponent
    result = (float)mantissa * powf(2.0f, (float)exponent);

    return result;
}

/******************************************************************************
 * @brief     : Convert float to LINEAR11 format
 * @param[in] : value --Floating point value to convert
 * @param[out]: None
 * @return    : 16-bit LINEAR11 value
 * @note      : LINEAR11 format: Y = mantissa * 2^exponent
 *****************************************************************************/
uint16_t PmBusFloatToLinear11(float value)
{
    int16_t exponent = 0;
    int16_t mantissa = 0;
    uint16_t result = 0;
    float scaledValue = 0.0f;

    if (value == 0.0f) {
        return 0;
    }

    // Find appropriate exponent
    exponent = (int16_t)(log2f(fabsf(value) / 1024.0f));

    // Clamp exponent to valid range (-16 to 15)
    if (exponent < -16) {
        exponent = -16;
    }
    if (exponent > 15) {
        exponent = 15;
    }

    // Calculate mantissa
    scaledValue = value / powf(2.0f, (float)exponent);
    mantissa = (int16_t)scaledValue;

    // Clamp mantissa to 11-bit signed range
    if (mantissa > 1023) {
        mantissa = 1023;
    }
    if (mantissa < -1024) {
        mantissa = -1024;
    }

    // Pack exponent and mantissa
    result = ((uint16_t)(exponent & 0x1F) << 11) | ((uint16_t)mantissa & 0x7FF);

    return result;
}

/******************************************************************************
 * @brief     : Convert float to LINEAR16 format
 * @param[in] : value --Floating point value to convert, exponent - 5-bit exponent
 * @param[out]: None
 * @return    : 16-bit LINEAR16 mantissa
 * @note      : LINEAR16 format: Y = mantissa * 2^exponent
 *****************************************************************************/
uint16_t PmBusFloatToLinear16(float value, int8_t exponent)
{
    int16_t mantissa = 0;
    float scaledValue = 0.0f;

    if (value == 0.0f) {
        return 0;
    }

    // Calculate mantissa
    scaledValue = value / powf(2.0f, (float)exponent);
    mantissa = (int16_t)scaledValue;

    return (uint16_t)mantissa;
}

/******************************************************************************
 * @brief     : Read VOUT_MODE to get voltage mode and exponent
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: exponent --Pointer to store exponent value
 * @return    : 0 if success, -1 if error
 * @note      : Extracts exponent from VOUT_MODE for LINEAR16 conversions
 *****************************************************************************/
int32_t PmBusGetVoutMode(uint8_t deviceAddr, int8_t* exponent)
{
    int32_t result = 0;
    uint8_t voutMode = 0;

    if (exponent == NULL) {
        return -1;
    }

    result = PmBusReadByte(deviceAddr, PMBUS_CMD_VOUT_MODE, &voutMode);
    if (result != 0) {
        return -1;
    }

    // Extract exponent (bits 4:0) and sign extend
    *exponent = (int8_t)(voutMode & 0x1F);
    if (*exponent > 15) {
        *exponent |= 0xE0;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read output voltage from PMBus device
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: voltage --Pointer to store voltage in volts
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_VOUT command and converts LINEAR16 to float
 *****************************************************************************/
int32_t PmBusReadVoltage(uint8_t deviceAddr, float* voltage)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};
    uint16_t rawValue = 0;
    int8_t exponent = 0;

    if (voltage == NULL) {
        return -1;
    }

    // Get VOUT_MODE exponent
    result = PmBusGetVoutMode(deviceAddr, &exponent);
    if (result != 0) {
        return -1;
    }

    // Read voltage value
    result = PmBusReadData(deviceAddr, PMBUS_CMD_READ_VOUT, buffer, 2);
    if (result != 0) {
        return -1;
    }

    rawValue = (uint16_t)(buffer[0] | (buffer[1] << 8));
    *voltage = PmBusLinear16ToFloat(rawValue, exponent);

    return 0;
}

/******************************************************************************
 * @brief     : Read output current from PMBus device
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: current --Pointer to store current in amperes
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_IOUT command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadCurrent(uint8_t deviceAddr, float* current)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};
    uint16_t rawValue = 0;

    if (current == NULL) {
        return -1;
    }

    result = PmBusReadData(deviceAddr, PMBUS_CMD_READ_IOUT, buffer, 2);
    if (result != 0) {
        return -1;
    }

    rawValue = (uint16_t)(buffer[0] | (buffer[1] << 8));
    *current = PmBusLinear11ToFloat(rawValue);

    return 0;
}

/******************************************************************************
 * @brief     : Read temperature from PMBus device
 * @param[in] : deviceAddr --PMBus device address, sensor - Sensor number (1-3)
 * @param[out]: temperature --Pointer to store temperature in Celsius
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_TEMPERATURE_X command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadTemperature(uint8_t deviceAddr, uint8_t sensor, float* temperature)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};
    uint16_t rawValue = 0;
    uint8_t command = 0;

    if (temperature == NULL) {
        return -1;
    }

    // Select temperature sensor command
    if (sensor == 1) {
        command = PMBUS_CMD_READ_TEMPERATURE_1;
    } else if (sensor == 2) {
        command = PMBUS_CMD_READ_TEMPERATURE_2;
    } else if (sensor == 3) {
        command = PMBUS_CMD_READ_TEMPERATURE_3;
    } else {
        return -1;
    }

    result = PmBusReadData(deviceAddr, command, buffer, 2);
    if (result != 0) {
        return -1;
    }

    rawValue = (uint16_t)(buffer[0] | (buffer[1] << 8));
    *temperature = PmBusLinear11ToFloat(rawValue);

    return 0;
}

/******************************************************************************
 * @brief     : Read input voltage from PMBus device
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: voltage --Pointer to store voltage in volts
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_VIN command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadInputVoltage(uint8_t deviceAddr, float* voltage)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};
    uint16_t rawValue = 0;

    if (voltage == NULL) {
        return -1;
    }

    result = PmBusReadData(deviceAddr, PMBUS_CMD_READ_VIN, buffer, 2);
    if (result != 0) {
        return -1;
    }

    rawValue = (uint16_t)(buffer[0] | (buffer[1] << 8));
    *voltage = PmBusLinear11ToFloat(rawValue);

    return 0;
}

/******************************************************************************
 * @brief     : Read input current from PMBus device
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: current --Pointer to store current in amperes
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_IIN command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadInputCurrent(uint8_t deviceAddr, float* current)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};
    uint16_t rawValue = 0;

    if (current == NULL) {
        return -1;
    }

    result = PmBusReadData(deviceAddr, PMBUS_CMD_READ_IIN, buffer, 2);
    if (result != 0) {
        return -1;
    }

    rawValue = (uint16_t)(buffer[0] | (buffer[1] << 8));
    *current = PmBusLinear11ToFloat(rawValue);

    return 0;
}

/******************************************************************************
 * @brief     : Read output power from PMBus device
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: power --Pointer to store power in watts
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_POUT command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadOutputPower(uint8_t deviceAddr, float* power)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};
    uint16_t rawValue = 0;

    if (power == NULL) {
        return -1;
    }

    result = PmBusReadData(deviceAddr, PMBUS_CMD_READ_POUT, buffer, 2);
    if (result != 0) {
        return -1;
    }

    rawValue = (uint16_t)(buffer[0] | (buffer[1] << 8));
    *power = PmBusLinear11ToFloat(rawValue);

    return 0;
}

/******************************************************************************
 * @brief     : Set output voltage on PMBus device
 * @param[in] : deviceAddr --PMBus device address, voltage - Voltage to set in volts
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes VOUT_COMMAND with LINEAR16 format
 *****************************************************************************/
int32_t PmBusSetVoltage(uint8_t deviceAddr, float voltage)
{
    int32_t result = 0;
    int8_t exponent = 0;
    uint16_t linear16Value = 0;
    uint8_t buffer[2] = {0};

    // Get VOUT_MODE exponent
    result = PmBusGetVoutMode(deviceAddr, &exponent);
    if (result != 0) {
        return -1;
    }

    // Convert voltage to LINEAR16
    linear16Value = PmBusFloatToLinear16(voltage, exponent);

    // Pack into buffer (little endian)
    buffer[0] = (uint8_t)(linear16Value & 0xFF);
    buffer[1] = (uint8_t)((linear16Value >> 8) & 0xFF);

    // Write to device
    result = PmBusWriteData(deviceAddr, PMBUS_CMD_VOUT_COMMAND, buffer, 2);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Get PMBus status word
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: status --Pointer to store 16-bit status word
 * @return    : 0 if success, -1 if error
 * @note      : Reads STATUS_WORD command
 *****************************************************************************/
int32_t PmBusGetStatus(uint8_t deviceAddr, uint16_t* status)
{
    int32_t result = 0;
    uint8_t buffer[2] = {0};

    if (status == NULL) {
        return -1;
    }

    result = PmBusReadData(deviceAddr, PMBUS_CMD_STATUS_WORD, buffer, 2);
    if (result != 0) {
        return -1;
    }

    *status = (uint16_t)(buffer[0] | (buffer[1] << 8));

    return 0;
}

/******************************************************************************
 * @brief     : Get PMBus status byte
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: status --Pointer to store 8-bit status byte
 * @return    : 0 if success, -1 if error
 * @note      : Reads STATUS_BYTE command
 *****************************************************************************/
int32_t PmBusGetStatusByte(uint8_t deviceAddr, uint8_t* status)
{
    int32_t result = 0;

    if (status == NULL) {
        return -1;
    }

    result = PmBusReadByte(deviceAddr, PMBUS_CMD_STATUS_BYTE, status);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Clear all faults on PMBus device
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends CLEAR_FAULTS command
 *****************************************************************************/
int32_t PmBusClearFaults(uint8_t deviceAddr)
{
    return PmBusSendCommand(deviceAddr, PMBUS_CMD_CLEAR_FAULTS);
}

/******************************************************************************
 * @brief     : Read manufacturer ID from PMBus device
 * @param[in] : deviceAddr --PMBus device address, maxLength - Maximum buffer size
 * @param[out]: buffer --Pointer to store manufacturer ID string
 * @return    : 0 if success, -1 if error
 * @note      : Reads MFR_ID command
 *****************************************************************************/
int32_t PmBusReadManufacturerId(uint8_t deviceAddr, uint8_t* buffer, uint16_t maxLength)
{
    int32_t result = 0;
    uint8_t length = 0;

    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    // Read first byte which contains the length
    result = PmBusReadByte(deviceAddr, PMBUS_CMD_MFR_ID, &length);
    if (result != 0) {
        return -1;
    }

    // Ensure we don't overflow buffer
    if (length >= maxLength) {
        length = (uint8_t)(maxLength - 1);
    }

    // Read the actual string data
    result = PmBusReadData(deviceAddr, PMBUS_CMD_MFR_ID, buffer, length + 1);
    if (result != 0) {
        return -1;
    }

    // Null terminate (skip length byte)
    if (length < maxLength) {
        buffer[length] = '\0';
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read model name from PMBus device
 * @param[in] : deviceAddr --PMBus device address, maxLength - Maximum buffer size
 * @param[out]: buffer --Pointer to store model name string
 * @return    : 0 if success, -1 if error
 * @note      : Reads MFR_MODEL command
 *****************************************************************************/
int32_t PmBusReadModel(uint8_t deviceAddr, uint8_t* buffer, uint16_t maxLength)
{
    int32_t result = 0;
    uint8_t length = 0;

    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    // Read first byte which contains the length
    result = PmBusReadByte(deviceAddr, PMBUS_CMD_MFR_MODEL, &length);
    if (result != 0) {
        return -1;
    }

    // Ensure we don't overflow buffer
    if (length >= maxLength) {
        length = (uint8_t)(maxLength - 1);
    }

    // Read the actual string data
    result = PmBusReadData(deviceAddr, PMBUS_CMD_MFR_MODEL, buffer, length + 1);
    if (result != 0) {
        return -1;
    }

    // Null terminate (skip length byte)
    if (length < maxLength) {
        buffer[length] = '\0';
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read PMBus revision
 * @param[in] : deviceAddr --PMBus device address
 * @param[out]: revision --Pointer to store revision byte
 * @return    : 0 if success, -1 if error
 * @note      : Reads PMBUS_REVISION command
 *****************************************************************************/
int32_t PmBusReadRevision(uint8_t deviceAddr, uint8_t* revision)
{
    int32_t result = 0;

    if (revision == NULL) {
        return -1;
    }

    result = PmBusReadByte(deviceAddr, PMBUS_CMD_PMBUS_REVISION, revision);
    if (result != 0) {
        return -1;
    }

    return 0;
}

#ifndef PMBUS_H
#define PMBUS_H

#include <stdint.h>

/* PMBus Standard Commands */
#define PMBUS_CMD_PAGE                  0x00
#define PMBUS_CMD_OPERATION             0x01
#define PMBUS_CMD_ON_OFF_CONFIG         0x02
#define PMBUS_CMD_CLEAR_FAULTS          0x03
#define PMBUS_CMD_PHASE                 0x04
#define PMBUS_CMD_WRITE_PROTECT         0x10
#define PMBUS_CMD_STORE_DEFAULT_ALL     0x11
#define PMBUS_CMD_RESTORE_DEFAULT_ALL   0x12
#define PMBUS_CMD_STORE_USER_ALL        0x15
#define PMBUS_CMD_RESTORE_USER_ALL      0x16
#define PMBUS_CMD_CAPABILITY            0x19
#define PMBUS_CMD_QUERY                 0x1A
#define PMBUS_CMD_VOUT_MODE             0x20
#define PMBUS_CMD_VOUT_COMMAND          0x21
#define PMBUS_CMD_VOUT_TRIM             0x22
#define PMBUS_CMD_VOUT_CAL_OFFSET       0x23
#define PMBUS_CMD_VOUT_MAX              0x24
#define PMBUS_CMD_VOUT_MARGIN_HIGH      0x25
#define PMBUS_CMD_VOUT_MARGIN_LOW       0x26
#define PMBUS_CMD_VOUT_TRANSITION_RATE  0x27
#define PMBUS_CMD_VOUT_DROOP            0x28
#define PMBUS_CMD_VOUT_SCALE_LOOP       0x29
#define PMBUS_CMD_VOUT_SCALE_MONITOR    0x2A
#define PMBUS_CMD_IOUT_CAL_GAIN         0x38
#define PMBUS_CMD_IOUT_CAL_OFFSET       0x39
#define PMBUS_CMD_FAN_CONFIG_1_2        0x3A
#define PMBUS_CMD_FAN_COMMAND_1         0x3B
#define PMBUS_CMD_FAN_COMMAND_2         0x3C
#define PMBUS_CMD_FAN_CONFIG_3_4        0x3D
#define PMBUS_CMD_FAN_COMMAND_3         0x3E
#define PMBUS_CMD_FAN_COMMAND_4         0x3F
#define PMBUS_CMD_VOUT_OV_FAULT_LIMIT   0x40
#define PMBUS_CMD_VOUT_OV_FAULT_RESPONSE 0x41
#define PMBUS_CMD_VOUT_OV_WARN_LIMIT    0x42
#define PMBUS_CMD_VOUT_UV_WARN_LIMIT    0x43
#define PMBUS_CMD_VOUT_UV_FAULT_LIMIT   0x44
#define PMBUS_CMD_VOUT_UV_FAULT_RESPONSE 0x45
#define PMBUS_CMD_IOUT_OC_FAULT_LIMIT   0x46
#define PMBUS_CMD_IOUT_OC_FAULT_RESPONSE 0x47
#define PMBUS_CMD_IOUT_OC_LV_FAULT_LIMIT 0x48
#define PMBUS_CMD_IOUT_OC_LV_FAULT_RESPONSE 0x49
#define PMBUS_CMD_IOUT_OC_WARN_LIMIT    0x4A
#define PMBUS_CMD_IOUT_UC_FAULT_LIMIT   0x4B
#define PMBUS_CMD_IOUT_UC_FAULT_RESPONSE 0x4C
#define PMBUS_CMD_OT_FAULT_LIMIT        0x4F
#define PMBUS_CMD_OT_FAULT_RESPONSE     0x50
#define PMBUS_CMD_OT_WARN_LIMIT         0x51
#define PMBUS_CMD_UT_WARN_LIMIT         0x52
#define PMBUS_CMD_UT_FAULT_LIMIT        0x53
#define PMBUS_CMD_UT_FAULT_RESPONSE     0x54
#define PMBUS_CMD_VIN_OV_FAULT_LIMIT    0x55
#define PMBUS_CMD_VIN_OV_FAULT_RESPONSE 0x56
#define PMBUS_CMD_VIN_OV_WARN_LIMIT     0x57
#define PMBUS_CMD_VIN_UV_WARN_LIMIT     0x58
#define PMBUS_CMD_VIN_UV_FAULT_LIMIT    0x59
#define PMBUS_CMD_VIN_UV_FAULT_RESPONSE 0x5A
#define PMBUS_CMD_IIN_OC_FAULT_LIMIT    0x5B
#define PMBUS_CMD_IIN_OC_FAULT_RESPONSE 0x5C
#define PMBUS_CMD_IIN_OC_WARN_LIMIT     0x5D
#define PMBUS_CMD_POWER_GOOD_ON         0x5E
#define PMBUS_CMD_POWER_GOOD_OFF        0x5F
#define PMBUS_CMD_TON_DELAY             0x60
#define PMBUS_CMD_TON_RISE              0x61
#define PMBUS_CMD_TON_MAX_FAULT_LIMIT   0x62
#define PMBUS_CMD_TON_MAX_FAULT_RESPONSE 0x63
#define PMBUS_CMD_TOFF_DELAY            0x64
#define PMBUS_CMD_TOFF_FALL             0x65
#define PMBUS_CMD_TOFF_MAX_WARN_LIMIT   0x66
#define PMBUS_CMD_STATUS_BYTE           0x78
#define PMBUS_CMD_STATUS_WORD           0x79
#define PMBUS_CMD_STATUS_VOUT           0x7A
#define PMBUS_CMD_STATUS_IOUT           0x7B
#define PMBUS_CMD_STATUS_INPUT          0x7C
#define PMBUS_CMD_STATUS_TEMPERATURE    0x7D
#define PMBUS_CMD_STATUS_CML            0x7E
#define PMBUS_CMD_STATUS_OTHER          0x7F
#define PMBUS_CMD_STATUS_MFR_SPECIFIC   0x80
#define PMBUS_CMD_STATUS_FAN_1_2        0x81
#define PMBUS_CMD_STATUS_FAN_3_4        0x82
#define PMBUS_CMD_READ_VIN              0x88
#define PMBUS_CMD_READ_IIN              0x89
#define PMBUS_CMD_READ_VCAP             0x8A
#define PMBUS_CMD_READ_VOUT             0x8B
#define PMBUS_CMD_READ_IOUT             0x8C
#define PMBUS_CMD_READ_TEMPERATURE_1    0x8D
#define PMBUS_CMD_READ_TEMPERATURE_2    0x8E
#define PMBUS_CMD_READ_TEMPERATURE_3    0x8F
#define PMBUS_CMD_READ_FAN_SPEED_1      0x90
#define PMBUS_CMD_READ_FAN_SPEED_2      0x91
#define PMBUS_CMD_READ_FAN_SPEED_3      0x92
#define PMBUS_CMD_READ_FAN_SPEED_4      0x93
#define PMBUS_CMD_READ_DUTY_CYCLE       0x94
#define PMBUS_CMD_READ_FREQUENCY        0x95
#define PMBUS_CMD_READ_POUT             0x96
#define PMBUS_CMD_READ_PIN              0x97
#define PMBUS_CMD_PMBUS_REVISION        0x98
#define PMBUS_CMD_MFR_ID                0x99
#define PMBUS_CMD_MFR_MODEL             0x9A
#define PMBUS_CMD_MFR_REVISION          0x9B
#define PMBUS_CMD_MFR_LOCATION          0x9C
#define PMBUS_CMD_MFR_DATE              0x9D
#define PMBUS_CMD_MFR_SERIAL            0x9E

/* VOUT_MODE Register Bits */
#define PMBUS_VOUT_MODE_LINEAR          0x00
#define PMBUS_VOUT_MODE_VID             0x20
#define PMBUS_VOUT_MODE_DIRECT          0x40

/* Default I2C Clock Speed for PMBus */
#define PMBUS_DEFAULT_CLOCK_SPEED       100000

/* Maximum number of pages (rails) */
#define PMBUS_MAX_PAGES                 16

/******************************************************************************
 * @brief     : Initialize PMBus peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures PMBus based on SMBus/I2C
 *****************************************************************************/
int32_t PmBusInit(void);

/******************************************************************************
 * @brief     : Deinitialize PMBus peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Releases PMBus resources
 *****************************************************************************/
int32_t PmBusDeinit(void);

/******************************************************************************
 * @brief     : Set PMBus page for multi-rail devices
 * @param[in] : deviceAddr - PMBus device address, page - Page number (0-15)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Selects the target rail/page for subsequent commands
 *****************************************************************************/
int32_t PmBusSetPage(uint8_t deviceAddr, uint8_t page);

/******************************************************************************
 * @brief     : Read output voltage from PMBus device
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: voltage - Pointer to store voltage in volts
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_VOUT command and converts LINEAR16 to float
 *****************************************************************************/
int32_t PmBusReadVoltage(uint8_t deviceAddr, float* voltage);

/******************************************************************************
 * @brief     : Read output current from PMBus device
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: current - Pointer to store current in amperes
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_IOUT command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadCurrent(uint8_t deviceAddr, float* current);

/******************************************************************************
 * @brief     : Read temperature from PMBus device
 * @param[in] : deviceAddr - PMBus device address, sensor - Sensor number (1-3)
 * @param[out]: temperature - Pointer to store temperature in Celsius
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_TEMPERATURE_X command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadTemperature(uint8_t deviceAddr, uint8_t sensor, float* temperature);

/******************************************************************************
 * @brief     : Read input voltage from PMBus device
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: voltage - Pointer to store voltage in volts
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_VIN command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadInputVoltage(uint8_t deviceAddr, float* voltage);

/******************************************************************************
 * @brief     : Read input current from PMBus device
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: current - Pointer to store current in amperes
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_IIN command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadInputCurrent(uint8_t deviceAddr, float* current);

/******************************************************************************
 * @brief     : Read output power from PMBus device
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: power - Pointer to store power in watts
 * @return    : 0 if success, -1 if error
 * @note      : Reads READ_POUT command and converts LINEAR11 to float
 *****************************************************************************/
int32_t PmBusReadOutputPower(uint8_t deviceAddr, float* power);

/******************************************************************************
 * @brief     : Set output voltage on PMBus device
 * @param[in] : deviceAddr - PMBus device address, voltage - Voltage to set in volts
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes VOUT_COMMAND with LINEAR16 format
 *****************************************************************************/
int32_t PmBusSetVoltage(uint8_t deviceAddr, float voltage);

/******************************************************************************
 * @brief     : Get PMBus status word
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: status - Pointer to store 16-bit status word
 * @return    : 0 if success, -1 if error
 * @note      : Reads STATUS_WORD command
 *****************************************************************************/
int32_t PmBusGetStatus(uint8_t deviceAddr, uint16_t* status);

/******************************************************************************
 * @brief     : Get PMBus status byte
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: status - Pointer to store 8-bit status byte
 * @return    : 0 if success, -1 if error
 * @note      : Reads STATUS_BYTE command
 *****************************************************************************/
int32_t PmBusGetStatusByte(uint8_t deviceAddr, uint8_t* status);

/******************************************************************************
 * @brief     : Clear all faults on PMBus device
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends CLEAR_FAULTS command
 *****************************************************************************/
int32_t PmBusClearFaults(uint8_t deviceAddr);

/******************************************************************************
 * @brief     : Convert LINEAR11 format to float
 * @param[in] : linear11 - 16-bit LINEAR11 value (5-bit exponent, 11-bit mantissa)
 * @param[out]: None
 * @return    : Floating point value
 * @note      : LINEAR11 format: Y = mantissa * 2^exponent
 *****************************************************************************/
float PmBusLinear11ToFloat(uint16_t linear11);

/******************************************************************************
 * @brief     : Convert LINEAR16 format to float
 * @param[in] : linear16 - 16-bit LINEAR16 mantissa, exponent - 5-bit exponent from VOUT_MODE
 * @param[out]: None
 * @return    : Floating point value
 * @note      : LINEAR16 format: Y = mantissa * 2^exponent
 *****************************************************************************/
float PmBusLinear16ToFloat(uint16_t linear16, int8_t exponent);

/******************************************************************************
 * @brief     : Convert float to LINEAR11 format
 * @param[in] : value - Floating point value to convert
 * @param[out]: None
 * @return    : 16-bit LINEAR11 value
 * @note      : LINEAR11 format: Y = mantissa * 2^exponent
 *****************************************************************************/
uint16_t PmBusFloatToLinear11(float value);

/******************************************************************************
 * @brief     : Convert float to LINEAR16 format
 * @param[in] : value - Floating point value to convert, exponent - 5-bit exponent
 * @param[out]: None
 * @return    : 16-bit LINEAR16 mantissa
 * @note      : LINEAR16 format: Y = mantissa * 2^exponent
 *****************************************************************************/
uint16_t PmBusFloatToLinear16(float value, int8_t exponent);

/******************************************************************************
 * @brief     : Read VOUT_MODE to get voltage mode and exponent
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: exponent - Pointer to store exponent value
 * @return    : 0 if success, -1 if error
 * @note      : Extracts exponent from VOUT_MODE for LINEAR16 conversions
 *****************************************************************************/
int32_t PmBusGetVoutMode(uint8_t deviceAddr, int8_t* exponent);

/******************************************************************************
 * @brief     : Read manufacturer ID from PMBus device
 * @param[in] : deviceAddr - PMBus device address, maxLength - Maximum buffer size
 * @param[out]: buffer - Pointer to store manufacturer ID string
 * @return    : 0 if success, -1 if error
 * @note      : Reads MFR_ID command
 *****************************************************************************/
int32_t PmBusReadManufacturerId(uint8_t deviceAddr, uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Read model name from PMBus device
 * @param[in] : deviceAddr - PMBus device address, maxLength - Maximum buffer size
 * @param[out]: buffer - Pointer to store model name string
 * @return    : 0 if success, -1 if error
 * @note      : Reads MFR_MODEL command
 *****************************************************************************/
int32_t PmBusReadModel(uint8_t deviceAddr, uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Read PMBus revision
 * @param[in] : deviceAddr - PMBus device address
 * @param[out]: revision - Pointer to store revision byte
 * @return    : 0 if success, -1 if error
 * @note      : Reads PMBUS_REVISION command
 *****************************************************************************/
int32_t PmBusReadRevision(uint8_t deviceAddr, uint8_t* revision);

#endif // PMBUS_H

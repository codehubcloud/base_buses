/******************************************************************************
 * @file    : onewire.c
 * @brief   : 1-Wire protocol implementation
 * @author  : Code Generator
 * @date    : 2025-10-01
 * @version : V1.0
 ******************************************************************************/

#include <stddef.h>
#include "onewire.h"
#include "onewire_hal.h"
#include "securec.h"

/******************************************************************************
 * Private Variables
 ******************************************************************************/

static OneWireSpeed_E g_oneWireSpeed = ONEWIRE_SPEED_STANDARD;

/******************************************************************************
 * @brief     : Initialize 1-Wire interface
 * @param[in] : none
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Configures GPIO and timing parameters
 ******************************************************************************/
int32_t OneWireInit(void)
{
    if (OneWireHalInit() != 0) {
        return -1;
    }

    g_oneWireSpeed = ONEWIRE_SPEED_STANDARD;
    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize 1-Wire interface
 * @param[in] : none
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Releases GPIO resources
 ******************************************************************************/
int32_t OneWireDeinit(void)
{
    return OneWireHalDeinit();
}

/******************************************************************************
 * @brief     : Send reset pulse and check for presence
 * @param[in] : none
 * @param[out]: none
 * @return    : 0 if device present, -1 if no device
 * @note      : Initializes communication sequence
 ******************************************************************************/
int32_t OneWireReset(void)
{
    return OneWireHalReset();
}

/******************************************************************************
 * @brief     : Write single bit to 1-Wire bus
 * @param[in] : bit --Bit value to write (0 or 1)
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Follows 1-Wire timing specification
 ******************************************************************************/
int32_t OneWireWriteBit(uint8_t bit)
{
    return OneWireHalWriteBit(bit);
}

/******************************************************************************
 * @brief     : Read single bit from 1-Wire bus
 * @param[in] : none
 * @param[out]: none
 * @return    : Bit value (0 or 1), -1 if error
 * @note      : Follows 1-Wire timing specification
 ******************************************************************************/
int32_t OneWireReadBit(void)
{
    return OneWireHalReadBit();
}

/******************************************************************************
 * @brief     : Write byte to 1-Wire bus
 * @param[in] : data --Byte to write
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : LSB first transmission
 ******************************************************************************/
int32_t OneWireWriteByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t bit = (data >> i) & 0x01;
        if (OneWireWriteBit(bit) != 0) {
            return -1;
        }
    }
    return 0;
}

/******************************************************************************
 * @brief     : Read byte from 1-Wire bus
 * @param[in] : none
 * @param[out]: data --Pointer to store received byte
 * @return    : 0 if success, -1 if error
 * @note      : LSB first reception
 ******************************************************************************/
int32_t OneWireReadByte(uint8_t* data)
{
    if (data == NULL) {
        return -1;
    }

    uint8_t result = 0;
    for (uint8_t i = 0; i < 8; i++) {
        int32_t bit = OneWireReadBit();
        if (bit < 0) {
            return -1;
        }
        result |= (bit << i);
    }

    *data = result;
    return 0;
}

/******************************************************************************
 * @brief     : Write multiple bytes to 1-Wire bus
 * @param[in] : data --Pointer to data buffer, length - Number of bytes
 * @param[out]: none
 * @return    : Number of bytes written, -1 if error
 * @note      : Sequential byte transmission
 ******************************************************************************/
int32_t OneWireWriteBytes(uint8_t* data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        if (OneWireWriteByte(data[i]) != 0) {
            return -1;
        }
    }

    return length;
}

/******************************************************************************
 * @brief     : Read multiple bytes from 1-Wire bus
 * @param[in] : length --Number of bytes to read
 * @param[out]: data --Pointer to receive buffer
 * @return    : Number of bytes read, -1 if error
 * @note      : Sequential byte reception
 ******************************************************************************/
int32_t OneWireReadBytes(uint8_t* data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        if (OneWireReadByte(&data[i]) != 0) {
            return -1;
        }
    }

    return length;
}

/******************************************************************************
 * @brief     : Calculate CRC8 for 1-Wire data
 * @param[in] : data --Pointer to data buffer, length - Number of bytes
 * @param[out]: none
 * @return    : CRC8 value
 * @note      : Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 ******************************************************************************/
uint8_t OneWireCalculateCRC8(uint8_t* data, uint16_t length)
{
    uint8_t crc = 0x00;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t i = 0; i < length; i++) {
        uint8_t inByte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inByte) & 0x01;
            crc >>= 1;
            if (mix != 0) {
                crc ^= 0x8C;
            }
            inByte >>= 1;
        }
    }

    return crc;
}

/******************************************************************************
 * @brief     : Search for devices on 1-Wire bus
 * @param[in] : maxDevices --Maximum devices to find
 * @param[out]: romCodes --Array to store 64-bit ROM codes
 * @return    : Number of devices found, -1 if error
 * @note      : Implements ROM search algorithm
 ******************************************************************************/
int32_t OneWireSearch(uint64_t* romCodes, uint8_t maxDevices)
{
    if (romCodes == NULL || maxDevices == 0) {
        return -1;
    }

    uint8_t deviceCount = 0;
    uint8_t lastDiscrepancy = 0;
    uint8_t searchDone = 0;
    uint64_t romCode = 0;

    while (searchDone == 0 && deviceCount < maxDevices) {
        if (OneWireReset() != 0) {
            return deviceCount;
        }

        if (OneWireWriteByte(ONEWIRE_CMD_SEARCH_ROM) != 0) {
            return -1;
        }

        uint8_t lastZero = 0;

        for (uint8_t bitPos = 0; bitPos < 64; bitPos++) {
            int32_t bit1 = OneWireReadBit();
            int32_t bit2 = OneWireReadBit();

            if (bit1 < 0 || bit2 < 0) {
                return -1;
            }

            uint8_t searchDirection = 0;

            if (bit1 == 1 && bit2 == 1) {
                break;
            } else if (bit1 == 0 && bit2 == 0) {
                if (bitPos == lastDiscrepancy) {
                    searchDirection = 1;
                } else if (bitPos > lastDiscrepancy) {
                    searchDirection = 0;
                } else {
                    searchDirection = (romCode >> bitPos) & 0x01;
                }

                if (searchDirection == 0) {
                    lastZero = bitPos;
                }
            } else {
                searchDirection = bit1;
            }

            if (searchDirection == 1) {
                romCode |= ((uint64_t)1 << bitPos);
            } else {
                romCode &= ~((uint64_t)1 << bitPos);
            }

            OneWireWriteBit(searchDirection);
        }

        romCodes[deviceCount] = romCode;
        deviceCount++;

        lastDiscrepancy = lastZero;

        if (lastDiscrepancy == 0) {
            searchDone = 1;
        }
    }

    return deviceCount;
}

/******************************************************************************
 * @brief     : Set 1-Wire communication speed
 * @param[in] : speed --Speed mode (standard/overdrive)
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Changes timing parameters
 ******************************************************************************/
int32_t OneWireSetSpeed(OneWireSpeed_E speed)
{
    g_oneWireSpeed = speed;
    return OneWireHalSetSpeed(speed);
}

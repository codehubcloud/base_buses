/******************************************************************************
 * @file    : onewire.h
 * @brief   : 1-Wire protocol interface header file
 * @author  : Code Generator
 * @date    : 2025-10-01
 * @version : V1.0
 * @note    : Single-wire bidirectional communication protocol
 ******************************************************************************/

#ifndef ONEWIRE_H
#define ONEWIRE_H

#include <stdint.h>

/******************************************************************************
 * Type Definitions
 ******************************************************************************/

typedef enum {
    ONEWIRE_SPEED_STANDARD = 0,    /* Standard speed mode */
    ONEWIRE_SPEED_OVERDRIVE        /* Overdrive speed mode */
} OneWireSpeed_E;

typedef enum {
    ONEWIRE_CMD_SEARCH_ROM = 0xF0,      /* Search ROM command */
    ONEWIRE_CMD_READ_ROM = 0x33,        /* Read ROM command */
    ONEWIRE_CMD_MATCH_ROM = 0x55,       /* Match ROM command */
    ONEWIRE_CMD_SKIP_ROM = 0xCC,        /* Skip ROM command */
    ONEWIRE_CMD_ALARM_SEARCH = 0xEC     /* Alarm search command */
} OneWireCommand_E;

/******************************************************************************
 * Function Declarations
 ******************************************************************************/

/******************************************************************************
 * @brief     : Initialize 1-Wire interface
 * @param[in] : none
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Configures GPIO and timing parameters
 ******************************************************************************/
int32_t OneWireInit(void);

/******************************************************************************
 * @brief     : Deinitialize 1-Wire interface
 * @param[in] : none
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Releases GPIO resources
 ******************************************************************************/
int32_t OneWireDeinit(void);

/******************************************************************************
 * @brief     : Send reset pulse and check for presence
 * @param[in] : none
 * @param[out]: none
 * @return    : 0 if device present, -1 if no device
 * @note      : Initializes communication sequence
 ******************************************************************************/
int32_t OneWireReset(void);

/******************************************************************************
 * @brief     : Write single bit to 1-Wire bus
 * @param[in] : bit - Bit value to write (0 or 1)
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Follows 1-Wire timing specification
 ******************************************************************************/
int32_t OneWireWriteBit(uint8_t bit);

/******************************************************************************
 * @brief     : Read single bit from 1-Wire bus
 * @param[in] : none
 * @param[out]: none
 * @return    : Bit value (0 or 1), -1 if error
 * @note      : Follows 1-Wire timing specification
 ******************************************************************************/
int32_t OneWireReadBit(void);

/******************************************************************************
 * @brief     : Write byte to 1-Wire bus
 * @param[in] : data - Byte to write
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : LSB first transmission
 ******************************************************************************/
int32_t OneWireWriteByte(uint8_t data);

/******************************************************************************
 * @brief     : Read byte from 1-Wire bus
 * @param[in] : none
 * @param[out]: data - Pointer to store received byte
 * @return    : 0 if success, -1 if error
 * @note      : LSB first reception
 ******************************************************************************/
int32_t OneWireReadByte(uint8_t* data);

/******************************************************************************
 * @brief     : Write multiple bytes to 1-Wire bus
 * @param[in] : data - Pointer to data buffer, length - Number of bytes
 * @param[out]: none
 * @return    : Number of bytes written, -1 if error
 * @note      : Sequential byte transmission
 ******************************************************************************/
int32_t OneWireWriteBytes(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Read multiple bytes from 1-Wire bus
 * @param[in] : length - Number of bytes to read
 * @param[out]: data - Pointer to receive buffer
 * @return    : Number of bytes read, -1 if error
 * @note      : Sequential byte reception
 ******************************************************************************/
int32_t OneWireReadBytes(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Calculate CRC8 for 1-Wire data
 * @param[in] : data - Pointer to data buffer, length - Number of bytes
 * @param[out]: none
 * @return    : CRC8 value
 * @note      : Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 ******************************************************************************/
uint8_t OneWireCalculateCRC8(uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Search for devices on 1-Wire bus
 * @param[in] : maxDevices - Maximum devices to find
 * @param[out]: romCodes - Array to store 64-bit ROM codes
 * @return    : Number of devices found, -1 if error
 * @note      : Implements ROM search algorithm
 ******************************************************************************/
int32_t OneWireSearch(uint64_t* romCodes, uint8_t maxDevices);

/******************************************************************************
 * @brief     : Set 1-Wire communication speed
 * @param[in] : speed - Speed mode (standard/overdrive)
 * @param[out]: none
 * @return    : 0 if success, -1 if error
 * @note      : Changes timing parameters
 ******************************************************************************/
int32_t OneWireSetSpeed(OneWireSpeed_E speed);

#endif /* ONEWIRE_H */

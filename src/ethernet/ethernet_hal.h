#ifndef ETHERNET_HAL_H
#define ETHERNET_HAL_H

#include <stdint.h>
#include "ethernet.h"

/******************************************************************************
 * @brief     : Enable Ethernet clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetEnableClock(void);

/******************************************************************************
 * @brief     : Disable Ethernet clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetDisableClock(void);

/******************************************************************************
 * @brief     : Configure Ethernet GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetConfigureGpio(void);

/******************************************************************************
 * @brief     : Configure MAC address
 * @param[in] : macAddress - Pointer to MAC address structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetConfigureMac(const EthernetMacAddress_t* macAddress);

/******************************************************************************
 * @brief     : Configure PHY speed and duplex
 * @param[in] : speed - Speed value, duplexMode - Duplex mode
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetConfigurePhy(uint32_t speed, uint8_t duplexMode);

/******************************************************************************
 * @brief     : Enable Ethernet module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void EthernetEnable(void);

/******************************************************************************
 * @brief     : Disable Ethernet module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *****************************************************************************/
void EthernetDisable(void);

/******************************************************************************
 * @brief     : Check if TX buffer is available
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if available, 0 if not available
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetTxBufferAvailable(void);

/******************************************************************************
 * @brief     : Write frame to Ethernet
 * @param[in] : frame - Pointer to frame data, length - Frame length
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetWriteFrame(const uint8_t* frame, uint16_t length);

/******************************************************************************
 * @brief     : Check if RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetRxBufferHasData(void);

/******************************************************************************
 * @brief     : Read frame from Ethernet
 * @param[in] : maxLength - Maximum length to read
 * @param[out]: buffer - Received frame data
 * @return    : Number of bytes read, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetReadFrame(uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Read PHY link status
 * @param[in] : None
 * @param[out]: None
 * @return    : ETHERNET_LINK_UP if link is up, ETHERNET_LINK_DOWN if down
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetReadPhyLinkStatus(void);

/******************************************************************************
 * @brief     : Configure promiscuous mode
 * @param[in] : enable - 1 to enable, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t EthernetConfigurePromiscuousMode(uint8_t enable);

#endif // ETHERNET_HAL_H

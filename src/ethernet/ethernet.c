#include <string.h>
#include "ethernet.h"
#include "ethernet_hal.h"
#include "securec.h"


/* Default Ethernet configuration */
static EthernetConfig_t g_ethernetConfig = {.speed = ETHERNET_SPEED_100M,
                                            .duplexMode = ETHERNET_DUPLEX_FULL,
                                            .macAddress = {{0x00, 0x11, 0x22, 0x33, 0x44, 0x55}},
                                            .promiscuousMode = 0};

/******************************************************************************
 * @brief     : Initialize Ethernet peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures Ethernet with 100M speed, full duplex by default
 *****************************************************************************/
int32_t EthernetInit(void)
{
    int32_t result = 0;

    result = EthernetEnableClock();
    if (result != 0) {
        return -1;
    }

    result = EthernetConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = EthernetConfigureMac(&g_ethernetConfig.macAddress);
    if (result != 0) {
        return -1;
    }

    result = EthernetConfigurePhy(g_ethernetConfig.speed, g_ethernetConfig.duplexMode);
    if (result != 0) {
        return -1;
    }

    EthernetEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize Ethernet peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables Ethernet peripheral and releases resources
 *****************************************************************************/
int32_t EthernetDeinit(void)
{
    EthernetDisable();
    EthernetDisableClock();
    return 0;
}

/******************************************************************************
 * @brief     : Send Ethernet frame
 * @param[in] : frame --Pointer to frame data buffer, length - Frame length in bytes
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer availability
 *****************************************************************************/
int32_t EthernetSendFrame(const uint8_t* frame, uint16_t length)
{
    if ((frame == NULL) || (length == 0) || (length > ETHERNET_MAX_FRAME_SIZE)) {
        return -1;
    }

    if (length < ETHERNET_MIN_FRAME_SIZE) {
        return -1;
    }

    while (EthernetTxBufferAvailable() == 0) {
    }

    return EthernetWriteFrame(frame, length);
}

/******************************************************************************
 * @brief     : Receive Ethernet frame
 * @param[in] : maxLength --Maximum frame length to receive
 * @param[out]: buffer --Received frame data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t EthernetReceiveFrame(uint8_t* buffer, uint16_t maxLength)
{
    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    if (EthernetRxBufferHasData() == 0) {
        return 0;
    }

    return EthernetReadFrame(buffer, maxLength);
}

/******************************************************************************
 * @brief     : Set MAC address
 * @param[in] : macAddress --Pointer to MAC address structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures the MAC address for the Ethernet interface
 *****************************************************************************/
int32_t EthernetSetMacAddress(const EthernetMacAddress_t* macAddress)
{
    if (macAddress == NULL) {
        return -1;
    }

    if (memcpy_s(&g_ethernetConfig.macAddress, sizeof(EthernetMacAddress_t), macAddress, sizeof(EthernetMacAddress_t)) != EOK) {
        return -1;
    }

    return EthernetConfigureMac(macAddress);
}

/******************************************************************************
 * @brief     : Get current link status
 * @param[in] : None
 * @param[out]: None
 * @return    : ETHERNET_LINK_UP if link is up, ETHERNET_LINK_DOWN if down
 * @note      : Checks PHY link status
 *****************************************************************************/
int32_t EthernetGetLinkStatus(void)
{
    return EthernetReadPhyLinkStatus();
}

/******************************************************************************
 * @brief     : Set Ethernet speed and duplex mode
 * @param[in] : speed --Speed value (ETHERNET_SPEED_10M/100M/1000M), duplexMode - Duplex mode
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures PHY speed and duplex settings
 *****************************************************************************/
int32_t EthernetSetSpeed(uint32_t speed, uint8_t duplexMode)
{
    if ((speed != ETHERNET_SPEED_10M) && (speed != ETHERNET_SPEED_100M) && (speed != ETHERNET_SPEED_1000M)) {
        return -1;
    }

    if ((duplexMode != ETHERNET_DUPLEX_HALF) && (duplexMode != ETHERNET_DUPLEX_FULL)) {
        return -1;
    }

    g_ethernetConfig.speed = speed;
    g_ethernetConfig.duplexMode = duplexMode;

    return EthernetConfigurePhy(speed, duplexMode);
}

/******************************************************************************
 * @brief     : Enable promiscuous mode
 * @param[in] : enable --1 to enable, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : In promiscuous mode, all frames are received regardless of MAC
 *****************************************************************************/
int32_t EthernetEnablePromiscuous(uint8_t enable)
{
    g_ethernetConfig.promiscuousMode = enable;
    return EthernetConfigurePromiscuousMode(enable);
}

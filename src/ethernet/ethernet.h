#ifndef ETHERNET_H
#define ETHERNET_H

#include <stdint.h>

/* Ethernet speed definitions */
#define ETHERNET_SPEED_10M 10000000     /* 10 Mbps */
#define ETHERNET_SPEED_100M 100000000   /* 100 Mbps */
#define ETHERNET_SPEED_1000M 1000000000 /* 1000 Mbps (1 Gbps) */

/* Ethernet duplex modes */
#define ETHERNET_DUPLEX_HALF 0
#define ETHERNET_DUPLEX_FULL 1

/* Ethernet frame size limits */
#define ETHERNET_MTU 1500
#define ETHERNET_HEADER_SIZE 14
#define ETHERNET_CRC_SIZE 4
#define ETHERNET_MIN_FRAME_SIZE 60
#define ETHERNET_MAX_FRAME_SIZE (ETHERNET_MTU + ETHERNET_HEADER_SIZE + ETHERNET_CRC_SIZE)

/* MAC address length */
#define ETHERNET_MAC_ADDR_LEN 6

/* Ethernet link status */
#define ETHERNET_LINK_DOWN 0
#define ETHERNET_LINK_UP 1

/* EtherType definitions */
#define ETHERNET_TYPE_IPV4 0x0800
#define ETHERNET_TYPE_ARP 0x0806
#define ETHERNET_TYPE_IPV6 0x86DD

/**
 * @brief MAC address structure
 */
typedef struct {
    uint8_t addr[ETHERNET_MAC_ADDR_LEN];
} EthernetMacAddress_t;

/**
 * @brief Ethernet frame header structure
 */
typedef struct {
    EthernetMacAddress_t destMac;
    EthernetMacAddress_t srcMac;
    uint16_t etherType;
} EthernetFrameHeader_t;

/**
 * @brief Ethernet configuration structure
 */
typedef struct {
    uint32_t speed;
    uint8_t duplexMode;
    EthernetMacAddress_t macAddress;
    uint8_t promiscuousMode;
} EthernetConfig_t;

/******************************************************************************
 * @brief     : Initialize Ethernet peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures Ethernet with 100M speed, full duplex by default
 *****************************************************************************/
int32_t EthernetInit(void);

/******************************************************************************
 * @brief     : Deinitialize Ethernet peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables Ethernet peripheral and releases resources
 *****************************************************************************/
int32_t EthernetDeinit(void);

/******************************************************************************
 * @brief     : Send Ethernet frame
 * @param[in] : frame - Pointer to frame data buffer, length - Frame length in bytes
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for TX buffer availability
 *****************************************************************************/
int32_t EthernetSendFrame(const uint8_t* frame, uint16_t length);

/******************************************************************************
 * @brief     : Receive Ethernet frame
 * @param[in] : maxLength - Maximum frame length to receive
 * @param[out]: buffer - Received frame data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Non-blocking function that returns immediately if no data
 *****************************************************************************/
int32_t EthernetReceiveFrame(uint8_t* buffer, uint16_t maxLength);

/******************************************************************************
 * @brief     : Set MAC address
 * @param[in] : macAddress - Pointer to MAC address structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures the MAC address for the Ethernet interface
 *****************************************************************************/
int32_t EthernetSetMacAddress(const EthernetMacAddress_t* macAddress);

/******************************************************************************
 * @brief     : Get current link status
 * @param[in] : None
 * @param[out]: None
 * @return    : ETHERNET_LINK_UP if link is up, ETHERNET_LINK_DOWN if down
 * @note      : Checks PHY link status
 *****************************************************************************/
int32_t EthernetGetLinkStatus(void);

/******************************************************************************
 * @brief     : Set Ethernet speed and duplex mode
 * @param[in] : speed - Speed value (ETHERNET_SPEED_10M/100M/1000M), duplexMode - Duplex mode
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures PHY speed and duplex settings
 *****************************************************************************/
int32_t EthernetSetSpeed(uint32_t speed, uint8_t duplexMode);

/******************************************************************************
 * @brief     : Enable promiscuous mode
 * @param[in] : enable - 1 to enable, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : In promiscuous mode, all frames are received regardless of MAC
 *****************************************************************************/
int32_t EthernetEnablePromiscuous(uint8_t enable);

#endif // ETHERNET_H

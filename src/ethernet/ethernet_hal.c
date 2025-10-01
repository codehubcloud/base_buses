#include "ethernet_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static ETH_HandleTypeDef g_ethHandle;
static uint8_t g_txBuffer[ETHERNET_MAX_FRAME_SIZE];
static uint8_t g_rxBuffer[ETHERNET_MAX_FRAME_SIZE];
#endif

#ifdef PLATFORM_STM32F1
static ETH_HandleTypeDef g_ethHandle;
static uint8_t g_txBuffer[ETHERNET_MAX_FRAME_SIZE];
static uint8_t g_rxBuffer[ETHERNET_MAX_FRAME_SIZE];
#endif

#ifdef PLATFORM_ESP32
#include "esp_eth.h"
#include "esp_netif.h"
static esp_eth_handle_t g_ethHandle = NULL;
static esp_netif_t* g_ethNetif = NULL;
static uint8_t g_rxBuffer[ETHERNET_MAX_FRAME_SIZE];
#endif

#ifdef PLATFORM_LINUX
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

static int g_sockFd = -1;
static struct ifreq g_ifr;
#endif

/******************************************************************************
 * @brief     : Enable Ethernet clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_ETH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_ETH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 Ethernet clock is managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable Ethernet clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetDisableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_ETH_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_ETH_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    return 0;
#elif defined(PLATFORM_LINUX)
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure Ethernet GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* RMII interface pins:
     * PA1  - ETH_RMII_REF_CLK
     * PA2  - ETH_RMII_MDIO
     * PA7  - ETH_RMII_CRS_DV
     * PC1  - ETH_RMII_MDC
     * PC4  - ETH_RMII_RXD0
     * PC5  - ETH_RMII_RXD1
     * PB11 - ETH_RMII_TX_EN
     * PB12 - ETH_RMII_TXD0
     * PB13 - ETH_RMII_TXD1
     */

    /* Configure PA1, PA2, PA7 */
    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure PC1, PC4, PC5 */
    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    /* Configure PB11, PB12, PB13 */
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* RMII interface GPIO configuration for STM32F1 */
    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by esp_eth driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses network interface, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure MAC address
 * @param[in] : macAddress - Pointer to MAC address structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetConfigureMac(const EthernetMacAddress_t* macAddress)
{
    if (macAddress == NULL) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    g_ethHandle.Instance = ETH;
    if (memcpy_s(g_ethHandle.Init.MACAddr, ETHERNET_MAC_ADDR_LEN, macAddress->addr, ETHERNET_MAC_ADDR_LEN) != EOK) {
        return -1;
    }
    g_ethHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    g_ethHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    g_ethHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    return (HAL_ETH_Init(&g_ethHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_ethHandle.Instance = ETH;
    if (memcpy_s(g_ethHandle.Init.MACAddr, ETHERNET_MAC_ADDR_LEN, macAddress->addr, ETHERNET_MAC_ADDR_LEN) != EOK) {
        return -1;
    }
    g_ethHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
    g_ethHandle.Init.RxMode = ETH_RXINTERRUPT_MODE;
    g_ethHandle.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
    return (HAL_ETH_Init(&g_ethHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    esp_eth_mac_t* mac = esp_eth_mac_new_esp32(&(eth_mac_config_t){});
    esp_eth_phy_t* phy = esp_eth_phy_new_lan8720(&(eth_phy_config_t){});

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    if (esp_eth_driver_install(&config, &g_ethHandle) != ESP_OK) {
        return -1;
    }

    if (esp_eth_ioctl(g_ethHandle, ETH_CMD_S_MAC_ADDR, (void*)macAddress->addr) != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    g_sockFd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (g_sockFd < 0) {
        return -1;
    }

    if (memset_s(&g_ifr, sizeof(g_ifr), 0, sizeof(g_ifr)) != EOK) {
        return -1;
    }
    if (strncpy_s(g_ifr.ifr_name, IFNAMSIZ, "eth0", IFNAMSIZ - 1) != EOK) {
        return -1;
    }

    if (ioctl(g_sockFd, SIOCGIFINDEX, &g_ifr) < 0) {
        return -1;
    }

    /* Set MAC address */
    if (memcpy_s(g_ifr.ifr_hwaddr.sa_data, ETHERNET_MAC_ADDR_LEN, macAddress->addr, ETHERNET_MAC_ADDR_LEN) != EOK) {
        return -1;
    }
    g_ifr.ifr_hwaddr.sa_family = ARPHRD_ETHER;

    if (ioctl(g_sockFd, SIOCSIFHWADDR, &g_ifr) < 0) {
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure PHY speed and duplex
 * @param[in] : speed - Speed value, duplexMode - Duplex mode
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetConfigurePhy(uint32_t speed, uint8_t duplexMode)
{
#ifdef PLATFORM_STM32F4
    uint32_t phySpeed = (speed == ETHERNET_SPEED_100M) ? ETH_SPEED_100M : ETH_SPEED_10M;
    uint32_t phyDuplex = (duplexMode == ETHERNET_DUPLEX_FULL) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;

    g_ethHandle.Init.Speed = phySpeed;
    g_ethHandle.Init.DuplexMode = phyDuplex;
    g_ethHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;

    return 0;
#elif defined(PLATFORM_STM32F1)
    uint32_t phySpeed = (speed == ETHERNET_SPEED_100M) ? ETH_SPEED_100M : ETH_SPEED_10M;
    uint32_t phyDuplex = (duplexMode == ETHERNET_DUPLEX_FULL) ? ETH_FULLDUPLEX_MODE : ETH_HALFDUPLEX_MODE;

    g_ethHandle.Init.Speed = phySpeed;
    g_ethHandle.Init.DuplexMode = phyDuplex;
    g_ethHandle.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;

    return 0;
#elif defined(PLATFORM_ESP32)
    eth_speed_t espSpeed = (speed == ETHERNET_SPEED_100M) ? ETH_SPEED_100M : ETH_SPEED_10M;
    eth_duplex_t espDuplex = (duplexMode == ETHERNET_DUPLEX_FULL) ? ETH_DUPLEX_FULL : ETH_DUPLEX_HALF;

    if (esp_eth_ioctl(g_ethHandle, ETH_CMD_S_SPEED, &espSpeed) != ESP_OK) {
        return -1;
    }

    if (esp_eth_ioctl(g_ethHandle, ETH_CMD_S_DUPLEX_MODE, &espDuplex) != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    struct ifreq ifr;
    if (memset_s(&ifr, sizeof(ifr), 0, sizeof(ifr)) != EOK) {
        return -1;
    }
    if (strncpy_s(ifr.ifr_name, IFNAMSIZ, "eth0", IFNAMSIZ - 1) != EOK) {
        return -1;
    }

    /* Linux speed/duplex configuration via ethtool would go here */
    /* For simplicity, we assume the interface is already configured */

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable Ethernet module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void EthernetEnable(void)
{
#ifdef PLATFORM_STM32F4
    HAL_ETH_Start(&g_ethHandle);
#elif defined(PLATFORM_STM32F1)
    HAL_ETH_Start(&g_ethHandle);
#elif defined(PLATFORM_ESP32)
    if (g_ethHandle != NULL) {
        esp_eth_start(g_ethHandle);
    }
#elif defined(PLATFORM_LINUX)
    struct ifreq ifr;
    if (g_sockFd >= 0) {
        if (memset_s(&ifr, sizeof(ifr), 0, sizeof(ifr)) != EOK) {
            return;
        }
        if (strncpy_s(ifr.ifr_name, IFNAMSIZ, "eth0", IFNAMSIZ - 1) != EOK) {
            return;
        }
        if (ioctl(g_sockFd, SIOCGIFFLAGS, &ifr) < 0) {
            return;
        }
        ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
        (void)ioctl(g_sockFd, SIOCSIFFLAGS, &ifr);
    }
#endif
}

/******************************************************************************
 * @brief     : Disable Ethernet module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void EthernetDisable(void)
{
#ifdef PLATFORM_STM32F4
    HAL_ETH_Stop(&g_ethHandle);
#elif defined(PLATFORM_STM32F1)
    HAL_ETH_Stop(&g_ethHandle);
#elif defined(PLATFORM_ESP32)
    if (g_ethHandle != NULL) {
        esp_eth_stop(g_ethHandle);
    }
#elif defined(PLATFORM_LINUX)
    if (g_sockFd >= 0) {
        struct ifreq ifr;
        if (memset_s(&ifr, sizeof(ifr), 0, sizeof(ifr)) != EOK) {
            return;
        }
        if (strncpy_s(ifr.ifr_name, IFNAMSIZ, "eth0", IFNAMSIZ - 1) != EOK) {
            return;
        }
        if (ioctl(g_sockFd, SIOCGIFFLAGS, &ifr) < 0) {
            close(g_sockFd);
            g_sockFd = -1;
            return;
        }
        ifr.ifr_flags &= ~(IFF_UP | IFF_RUNNING);
        (void)ioctl(g_sockFd, SIOCSIFFLAGS, &ifr);
        close(g_sockFd);
        g_sockFd = -1;
    }
#endif
}

/******************************************************************************
 * @brief     : Check if TX buffer is available
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if available, 0 if not available
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetTxBufferAvailable(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_ETH_GetState(&g_ethHandle) == HAL_ETH_STATE_READY) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (HAL_ETH_GetState(&g_ethHandle) == HAL_ETH_STATE_READY) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1; /* ESP32 driver handles buffering */
#elif defined(PLATFORM_LINUX)
    return 1; /* Linux socket handles buffering */
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Write frame to Ethernet
 * @param[in] : frame - Pointer to frame data, length - Frame length
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetWriteFrame(const uint8_t* frame, uint16_t length)
{
    if ((frame == NULL) || (length == 0)) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    if (memcpy_s(g_txBuffer, ETHERNET_MAX_FRAME_SIZE, frame, length) != EOK) {
        return -1;
    }
    return (HAL_ETH_TransmitFrame(&g_ethHandle, length) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    if (memcpy_s(g_txBuffer, ETHERNET_MAX_FRAME_SIZE, frame, length) != EOK) {
        return -1;
    }
    return (HAL_ETH_TransmitFrame(&g_ethHandle, length) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    if (g_ethHandle == NULL) {
        return -1;
    }
    return (esp_eth_transmit(g_ethHandle, (void*)frame, length) == ESP_OK) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_sockFd < 0) {
        return -1;
    }

    struct sockaddr_ll sockAddr;
    if (memset_s(&sockAddr, sizeof(sockAddr), 0, sizeof(sockAddr)) != EOK) {
        return -1;
    }
    sockAddr.sll_ifindex = g_ifr.ifr_ifindex;
    sockAddr.sll_halen = ETH_ALEN;

    ssize_t sent = sendto(g_sockFd, frame, length, 0, (struct sockaddr*)&sockAddr, sizeof(sockAddr));
    return (sent == length) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetRxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_ETH_GetReceivedFrame(&g_ethHandle) == HAL_OK) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (HAL_ETH_GetReceivedFrame(&g_ethHandle) == HAL_OK) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses callback mechanism for RX, simplified check */
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_sockFd < 0) {
        return 0;
    }

    fd_set readFds;
    struct timeval timeout = {0, 0};

    FD_ZERO(&readFds);
    FD_SET(g_sockFd, &readFds);

    int result = select(g_sockFd + 1, &readFds, NULL, NULL, &timeout);
    return (result > 0) ? 1 : 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Read frame from Ethernet
 * @param[in] : maxLength - Maximum length to read
 * @param[out]: buffer - Received frame data
 * @return    : Number of bytes read, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetReadFrame(uint8_t* buffer, uint16_t maxLength)
{
    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    ETH_BufferTypeDef rxBuffer;
    if (HAL_ETH_ReadData(&g_ethHandle, (void**)&rxBuffer) != HAL_OK) {
        return -1;
    }

    uint16_t length = (rxBuffer.len < maxLength) ? rxBuffer.len : maxLength;
    if (memcpy_s(buffer, maxLength, rxBuffer.buffer, length) != EOK) {
        return -1;
    }

    return (int32_t)length;
#elif defined(PLATFORM_STM32F1)
    ETH_BufferTypeDef rxBuffer;
    if (HAL_ETH_ReadData(&g_ethHandle, (void**)&rxBuffer) != HAL_OK) {
        return -1;
    }

    uint16_t length = (rxBuffer.len < maxLength) ? rxBuffer.len : maxLength;
    if (memcpy_s(buffer, maxLength, rxBuffer.buffer, length) != EOK) {
        return -1;
    }

    return (int32_t)length;
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses callback-based reception */
    /* For simplicity, return 0 (would need more complex buffer management) */
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_sockFd < 0) {
        return -1;
    }

    ssize_t received = recvfrom(g_sockFd, buffer, maxLength, 0, NULL, NULL);
    return (received > 0) ? (int32_t)received : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read PHY link status
 * @param[in] : None
 * @param[out]: None
 * @return    : ETHERNET_LINK_UP if link is up, ETHERNET_LINK_DOWN if down
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetReadPhyLinkStatus(void)
{
#ifdef PLATFORM_STM32F4
    uint32_t phyStatus = 0;
    HAL_ETH_ReadPHYRegister(&g_ethHandle, PHY_BSR, &phyStatus);
    return (phyStatus & PHY_LINKED_STATUS) ? ETHERNET_LINK_UP : ETHERNET_LINK_DOWN;
#elif defined(PLATFORM_STM32F1)
    uint32_t phyStatus = 0;
    HAL_ETH_ReadPHYRegister(&g_ethHandle, PHY_BSR, &phyStatus);
    return (phyStatus & PHY_LINKED_STATUS) ? ETHERNET_LINK_UP : ETHERNET_LINK_DOWN;
#elif defined(PLATFORM_ESP32)
    eth_link_t linkStatus;
    if (esp_eth_ioctl(g_ethHandle, ETH_CMD_G_LINK, &linkStatus) == ESP_OK) {
        return (linkStatus == ETH_LINK_UP) ? ETHERNET_LINK_UP : ETHERNET_LINK_DOWN;
    }
    return ETHERNET_LINK_DOWN;
#elif defined(PLATFORM_LINUX)
    if (g_sockFd < 0) {
        return ETHERNET_LINK_DOWN;
    }

    struct ifreq ifr;
    if (memset_s(&ifr, sizeof(ifr), 0, sizeof(ifr)) != EOK) {
        return ETHERNET_LINK_DOWN;
    }
    if (strncpy_s(ifr.ifr_name, IFNAMSIZ, "eth0", IFNAMSIZ - 1) != EOK) {
        return ETHERNET_LINK_DOWN;
    }

    if (ioctl(g_sockFd, SIOCGIFFLAGS, &ifr) >= 0) {
        return (ifr.ifr_flags & IFF_RUNNING) ? ETHERNET_LINK_UP : ETHERNET_LINK_DOWN;
    }

    return ETHERNET_LINK_DOWN;
#else
    return ETHERNET_LINK_DOWN;
#endif
}

/******************************************************************************
 * @brief     : Configure promiscuous mode
 * @param[in] : enable - 1 to enable, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EthernetConfigurePromiscuousMode(uint8_t enable)
{
#ifdef PLATFORM_STM32F4
    if (enable) {
        g_ethHandle.Instance->MACFFR |= ETH_MACFFR_PM;
    } else {
        g_ethHandle.Instance->MACFFR &= ~ETH_MACFFR_PM;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (enable) {
        g_ethHandle.Instance->MACFFR |= ETH_MACFFR_PM;
    } else {
        g_ethHandle.Instance->MACFFR &= ~ETH_MACFFR_PM;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    eth_promiscuous_mode_t mode = enable ? true : false;
    return (esp_eth_ioctl(g_ethHandle, ETH_CMD_S_PROMISCUOUS, &mode) == ESP_OK) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_sockFd < 0) {
        return -1;
    }

    struct ifreq ifr;
    if (memset_s(&ifr, sizeof(ifr), 0, sizeof(ifr)) != EOK) {
        return -1;
    }
    if (strncpy_s(ifr.ifr_name, IFNAMSIZ, "eth0", IFNAMSIZ - 1) != EOK) {
        return -1;
    }

    if (ioctl(g_sockFd, SIOCGIFFLAGS, &ifr) < 0) {
        return -1;
    }

    if (enable) {
        ifr.ifr_flags |= IFF_PROMISC;
    } else {
        ifr.ifr_flags &= ~IFF_PROMISC;
    }

    return (ioctl(g_sockFd, SIOCSIFFLAGS, &ifr) >= 0) ? 0 : -1;
#else
    return -1;
#endif
}

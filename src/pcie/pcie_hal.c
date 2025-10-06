/**
 * @file pcie_hal.c
 * * @brief PCIe Hardware Abstraction Layer implementation
 * @details Multi-platform HAL implementation for PCIe operations
 *          Supports: STM32F4, STM32F1, ESP32 (not supported), Linux (full support)
 */

#include <string.h>
#include "pcie_hal.h"


/* Platform detection */
#if defined(STM32F407xx) || defined(STM32F429xx)
#define PLATFORM_STM32F4
#elif defined(STM32F103xB) || defined(STM32F103xE)
#define PLATFORM_STM32F1
#elif defined(ESP32)
#define PLATFORM_ESP32
#else
#define PLATFORM_LINUX
#endif

/* Linux-specific includes */
#ifdef PLATFORM_LINUX
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "securec.h"


/* Linux sysfs paths */
#define PCIE_SYSFS_PATH "/sys/bus/pci/devices"
#define PCIE_CONFIG_PATH_FORMAT "/sys/bus/pci/devices/%04x:%02x:%02x.%x/config"
#define PCIE_LINK_STATUS_PATH "/sys/bus/pci/devices/%04x:%02x:%02x.%x/current_link_speed"
#define PCIE_LINK_WIDTH_PATH "/sys/bus/pci/devices/%04x:%02x:%02x.%x/current_link_width"

/* Internal state for Linux */
static int linuxPcieInitialized = 0;
static uint16_t currentDomain = 0; /* PCI domain (usually 0) */

#endif                             /* PLATFORM_LINUX */

/*=============================================================================
 * STM32F4 Platform Implementation (Not Supported)
 *===========================================================================*/
#ifdef PLATFORM_STM32F4

/******************************************************************************
 * @brief      : Initialize PCIe hardware
 * @param[in]  : config --Pointer to PCIe configuration structure
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalInit(const PcieConfig* config)
{
    (void)config;
    /* STM32F4 does not have PCIe controller */
    return -1;
}

/******************************************************************************
 * @brief      : Deinitialize PCIe hardware
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalDeinit(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Read from PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset size --Size of data to read
 * @param[out] : data --Pointer to store read data
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size)
{
    (void)bus;
    (void)device;
    (void)function;
    (void)offset;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset data --Data to write size --Size of data to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size)
{
    (void)bus;
    (void)device;
    (void)function;
    (void)offset;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Send TLP packet
 * @param[in]  : packet --Pointer to TLP packet
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    return -1;
}

/******************************************************************************
 * @brief      : Receive TLP packet
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] : packet --Pointer to store received TLP packet
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    return -1;
}

/******************************************************************************
 * @brief      : Get PCIe link status
 * @param[in]  :
 * @param[out] : status --Pointer to link status structure
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    (void)status;
    return -1;
}

/******************************************************************************
 * @brief      : Read from PCIe memory-mapped region
 * @param[in]  : address --Physical address size --Number of bytes to read
 * @param[out] : data --Buffer to store read data
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe memory-mapped region
 * @param[in]  : address --Physical address data --Data to write size --Number of bytes to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Enable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalEnableInterrupts(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Disable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalDisableInterrupts(void)
{
    return -1;
}

#endif /* PLATFORM_STM32F4 */

/*=============================================================================
 * STM32F1 Platform Implementation (Not Supported)
 *===========================================================================*/
#ifdef PLATFORM_STM32F1

/******************************************************************************
 * @brief      : Initialize PCIe hardware
 * @param[in]  : config --Pointer to PCIe configuration structure
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalInit(const PcieConfig* config)
{
    (void)config;
    /* STM32F1 does not have PCIe controller */
    return -1;
}

/******************************************************************************
 * @brief      : Deinitialize PCIe hardware
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalDeinit(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Read from PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset size --Size of data to read
 * @param[out] : data --Pointer to store read data
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size)
{
    (void)bus;
    (void)device;
    (void)function;
    (void)offset;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset data --Data to write size --Size of data to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size)
{
    (void)bus;
    (void)device;
    (void)function;
    (void)offset;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Send TLP packet
 * @param[in]  : packet --Pointer to TLP packet
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    return -1;
}

/******************************************************************************
 * @brief      : Receive TLP packet
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] : packet --Pointer to store received TLP packet
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    return -1;
}

/******************************************************************************
 * @brief      : Get PCIe link status
 * @param[in]  :
 * @param[out] : status --Pointer to link status structure
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    (void)status;
    return -1;
}

/******************************************************************************
 * @brief      : Read from PCIe memory-mapped region
 * @param[in]  : address --Physical address size --Number of bytes to read
 * @param[out] : data --Buffer to store read data
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe memory-mapped region
 * @param[in]  : address --Physical address data --Data to write size --Number of bytes to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Enable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalEnableInterrupts(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Disable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalDisableInterrupts(void)
{
    return -1;
}

#endif /* PLATFORM_STM32F1 */

/*=============================================================================
 * ESP32 Platform Implementation (Not Supported)
 *===========================================================================*/
#ifdef PLATFORM_ESP32

/******************************************************************************
 * @brief      : Initialize PCIe hardware
 * @param[in]  : config --Pointer to PCIe configuration structure
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalInit(const PcieConfig* config)
{
    (void)config;
    /* ESP32 does not have PCIe controller */
    return -1;
}

/******************************************************************************
 * @brief      : Deinitialize PCIe hardware
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalDeinit(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Read from PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset size --Size of data to read
 * @param[out] : data --Pointer to store read data
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size)
{
    (void)bus;
    (void)device;
    (void)function;
    (void)offset;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset data --Data to write size --Size of data to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size)
{
    (void)bus;
    (void)device;
    (void)function;
    (void)offset;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Send TLP packet
 * @param[in]  : packet --Pointer to TLP packet
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    return -1;
}

/******************************************************************************
 * @brief      : Receive TLP packet
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] : packet --Pointer to store received TLP packet
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    return -1;
}

/******************************************************************************
 * @brief      : Get PCIe link status
 * @param[in]  :
 * @param[out] : status --Pointer to link status structure
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    (void)status;
    return -1;
}

/******************************************************************************
 * @brief      : Read from PCIe memory-mapped region
 * @param[in]  : address --Physical address size --Number of bytes to read
 * @param[out] : data --Buffer to store read data
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe memory-mapped region
 * @param[in]  : address --Physical address data --Data to write size --Number of bytes to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

/******************************************************************************
 * @brief      : Enable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalEnableInterrupts(void)
{
    return -1;
}

/******************************************************************************
 * @brief      : Disable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Platform does not support PCIe hardware
 *****************************************************************************/
int PcieHalDisableInterrupts(void)
{
    return -1;
}

#endif /* PLATFORM_ESP32 */

/*=============================================================================
 * Linux Platform Implementation (Full Support)
 *===========================================================================*/
#ifdef PLATFORM_LINUX

/******************************************************************************
 * @brief      : Helper function to open PCIe config file
 * @param[in]  : bus --Bus number device --Device number function --Function number mode --Open mode (O_RDONLY or O_RDWR)
 * @param[out] :
 * @return     : File descriptor on success, -1 on error
 * @note       :
 *****************************************************************************/
static int OpenConfigFile(uint8_t bus, uint8_t device, uint8_t function, int mode)
{
    char configPath[256];
    int ret;

    ret = snprintf_s(configPath, sizeof(configPath), sizeof(configPath) - 1, PCIE_CONFIG_PATH_FORMAT, currentDomain, bus, device, function);
    if (ret < 0) {
        return -1;
    }

    return open(configPath, mode);
}

/******************************************************************************
 * @brief      : Initialize PCIe hardware
 * @param[in]  : config --Pointer to PCIe configuration structure
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       : Linux implementation verifies /sys/bus/pci/devices exists
 *****************************************************************************/
int PcieHalInit(const PcieConfig* config)
{
    DIR* dir;

    if (config == NULL) {
        return -1;
    }

    /* Check if sysfs PCI interface exists */
    dir = opendir(PCIE_SYSFS_PATH);
    if (dir == NULL) {
        return -1;
    }
    closedir(dir);

    linuxPcieInitialized = 1;
    return 0;
}

/******************************************************************************
 * @brief      : Deinitialize PCIe hardware
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       : Linux implementation clears initialization flag
 *****************************************************************************/
int PcieHalDeinit(void)
{
    if (linuxPcieInitialized == 0) {
        return -1;
    }

    linuxPcieInitialized = 0;
    return 0;
}

/******************************************************************************
 * @brief      : Read from PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset size --Size of data to read (1, 2, or 4 bytes)
 * @param[out] : data --Pointer to store read data
 * @return     : 0 on success, -1 on error
 * @note       : Linux implementation uses /sys/bus/pci/devices/DDDD:BB:DD.F/config
 *****************************************************************************/
int PcieHalConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size)
{
    int fd;
    ssize_t bytesRead;
    uint8_t buffer[4] = {0};

    if (linuxPcieInitialized == 0) {
        return -1;
    }

    if (data == NULL) {
        return -1;
    }

    /* Open config file */
    fd = OpenConfigFile(bus, device, function, O_RDONLY);
    if (fd < 0) {
        return -1;
    }

    /* Seek to offset */
    if (lseek(fd, offset, SEEK_SET) < 0) {
        close(fd);
        return -1;
    }

    /* Read data */
    bytesRead = read(fd, buffer, size);
    close(fd);

    if (bytesRead != size) {
        return -1;
    }

    /* Convert to uint32_t based on size */
    *data = 0;
    if (size == 1) {
        *data = buffer[0];
    } else if (size == 2) {
        *data = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8);
    } else if (size == 4) {
        *data = (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
    }

    return 0;
}

/******************************************************************************
 * @brief      : Write to PCIe configuration space
 * @param[in]  : bus --Bus number device --Device number function --Function number offset --Register offset data --Data to write size --Size of data to write (1, 2, or 4 bytes)
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       : Linux implementation uses /sys/bus/pci/devices/DDDD:BB:DD.F/config
 *****************************************************************************/
int PcieHalConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size)
{
    int fd;
    ssize_t bytesWritten;
    uint8_t buffer[4];

    if (linuxPcieInitialized == 0) {
        return -1;
    }

    /* Open config file */
    fd = OpenConfigFile(bus, device, function, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    /* Seek to offset */
    if (lseek(fd, offset, SEEK_SET) < 0) {
        close(fd);
        return -1;
    }

    /* Prepare data buffer */
    buffer[0] = (uint8_t)(data & 0xFF);
    if (size >= 2) {
        buffer[1] = (uint8_t)((data >> 8) & 0xFF);
    }
    if (size >= 4) {
        buffer[2] = (uint8_t)((data >> 16) & 0xFF);
        buffer[3] = (uint8_t)((data >> 24) & 0xFF);
    }

    /* Write data */
    bytesWritten = write(fd, buffer, size);
    close(fd);

    if (bytesWritten != size) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Send TLP packet
 * @param[in]  : packet --Pointer to TLP packet
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : TLP-level operations not directly supported via sysfs, would require kernel driver or VFIO interface
 *****************************************************************************/
int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    /* TLP-level operations not directly supported via sysfs */
    /* Would require kernel driver or VFIO interface */
    return -1;
}

/******************************************************************************
 * @brief      : Receive TLP packet
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] : packet --Pointer to store received TLP packet
 * @return     : -1 (always fails)
 * @note       : TLP-level operations not directly supported via sysfs, would require kernel driver or VFIO interface
 *****************************************************************************/
int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    /* TLP-level operations not directly supported via sysfs */
    return -1;
}

/******************************************************************************
 * @brief      : Get PCIe link status
 * @param[in]  :
 * @param[out] : status --Pointer to link status structure
 * @return     : 0 on success, -1 on error
 * @note       : Linux implementation reads current_link_speed and current_link_width from sysfs
 *****************************************************************************/
int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    char speedPath[256];
    char widthPath[256];
    FILE* fp;
    char speedStr[32] = {0};
    int width;
    int ret;

    if (linuxPcieInitialized == 0) {
        return -1;
    }

    if (status == NULL) {
        return -1;
    }

    /* Initialize status */
    if (memset_s(status, sizeof(PcieLinkStatus), 0, sizeof(PcieLinkStatus)) != EOK) {
        return -1;
    }

    /* For simplicity, check bus 0, device 0, function 0 */
    /* In real implementation, should check root complex or specific device */
    ret = snprintf_s(speedPath, sizeof(speedPath), sizeof(speedPath) - 1, PCIE_LINK_STATUS_PATH, currentDomain, 0, 0, 0);
    if (ret < 0) {
        return -1;
    }

    ret = snprintf_s(widthPath, sizeof(widthPath), sizeof(widthPath) - 1, PCIE_LINK_WIDTH_PATH, currentDomain, 0, 0, 0);
    if (ret < 0) {
        return -1;
    }

    /* Read link speed */
    fp = fopen(speedPath, "r");
    if (fp != NULL) {
        if (fgets(speedStr, sizeof(speedStr), fp) != NULL) {
            /* Parse speed string (e.g., "8.0 GT/s" for Gen3) */
            if (strstr(speedStr, "2.5") != NULL) {
                status->generation = PCIE_GEN1;
            } else if (strstr(speedStr, "5.0") != NULL) {
                status->generation = PCIE_GEN2;
            } else if (strstr(speedStr, "8.0") != NULL) {
                status->generation = PCIE_GEN3;
            } else if (strstr(speedStr, "16.0") != NULL) {
                status->generation = PCIE_GEN4;
            }
        }
        fclose(fp);
    }

    /* Read link width */
    fp = fopen(widthPath, "r");
    if (fp != NULL) {
        if (fscanf(fp, "%d", &width) == 1) {
            status->linkWidth = (uint8_t)width;
            status->maxLinkWidth = (uint8_t)width;
        }
        fclose(fp);
    }

    /* If we got any valid data, mark link as up */
    if (status->generation > 0 || status->linkWidth > 0) {
        status->isLinkUp = 1;
    }

    return 0;
}

/******************************************************************************
 * @brief      : Read from PCIe memory-mapped region
 * @param[in]  : address --Physical address size --Number of bytes to read
 * @param[out] : data --Buffer to store read data
 * @return     : -1 (always fails)
 * @note       : Direct memory access requires /dev/mem and root privileges, not implemented for safety reasons
 *****************************************************************************/
int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    /* Direct memory access requires /dev/mem and root privileges */
    /* Not implemented for safety reasons */
    return -1;
}

/******************************************************************************
 * @brief      : Write to PCIe memory-mapped region
 * @param[in]  : address --Physical address data --Data to write size --Number of bytes to write
 * @param[out] :
 * @return     : -1 (always fails)
 * @note       : Direct memory access requires /dev/mem and root privileges, not implemented for safety reasons
 *****************************************************************************/
int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    /* Direct memory access requires /dev/mem and root privileges */
    return -1;
}

/******************************************************************************
 * @brief      : Enable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : 0 (success)
 * @note       : Interrupt management handled by kernel drivers
 *****************************************************************************/
int PcieHalEnableInterrupts(void)
{
    /* Interrupt management handled by kernel drivers */
    return 0;
}

/******************************************************************************
 * @brief      : Disable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : 0 (success)
 * @note       : Interrupt management handled by kernel drivers
 *****************************************************************************/
int PcieHalDisableInterrupts(void)
{
    /* Interrupt management handled by kernel drivers */
    return 0;
}

#endif /* PLATFORM_LINUX */

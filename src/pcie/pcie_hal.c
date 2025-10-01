/**
 * @file pcie_hal.c
 * @brief PCIe Hardware Abstraction Layer implementation
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

int PcieHalInit(const PcieConfig* config)
{
    (void)config;
    /* STM32F4 does not have PCIe controller */
    return -1;
}

int PcieHalDeinit(void)
{
    return -1;
}

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

int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    return -1;
}

int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    return -1;
}

int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    (void)status;
    return -1;
}

int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

int PcieHalEnableInterrupts(void)
{
    return -1;
}

int PcieHalDisableInterrupts(void)
{
    return -1;
}

#endif /* PLATFORM_STM32F4 */

/*=============================================================================
 * STM32F1 Platform Implementation (Not Supported)
 *===========================================================================*/
#ifdef PLATFORM_STM32F1

int PcieHalInit(const PcieConfig* config)
{
    (void)config;
    /* STM32F1 does not have PCIe controller */
    return -1;
}

int PcieHalDeinit(void)
{
    return -1;
}

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

int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    return -1;
}

int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    return -1;
}

int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    (void)status;
    return -1;
}

int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

int PcieHalEnableInterrupts(void)
{
    return -1;
}

int PcieHalDisableInterrupts(void)
{
    return -1;
}

#endif /* PLATFORM_STM32F1 */

/*=============================================================================
 * ESP32 Platform Implementation (Not Supported)
 *===========================================================================*/
#ifdef PLATFORM_ESP32

int PcieHalInit(const PcieConfig* config)
{
    (void)config;
    /* ESP32 does not have PCIe controller */
    return -1;
}

int PcieHalDeinit(void)
{
    return -1;
}

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

int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    return -1;
}

int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    return -1;
}

int PcieHalGetLinkStatus(PcieLinkStatus* status)
{
    (void)status;
    return -1;
}

int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    return -1;
}

int PcieHalEnableInterrupts(void)
{
    return -1;
}

int PcieHalDisableInterrupts(void)
{
    return -1;
}

#endif /* PLATFORM_ESP32 */

/*=============================================================================
 * Linux Platform Implementation (Full Support)
 *===========================================================================*/
#ifdef PLATFORM_LINUX

/**
 * @brief Helper function to open PCIe config file
 * @param bus Bus number
 * @param device Device number
 * @param function Function number
 * @param mode Open mode (O_RDONLY or O_RDWR)
 * @return File descriptor on success, -1 on error
 */
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

int PcieHalDeinit(void)
{
    if (linuxPcieInitialized == 0) {
        return -1;
    }

    linuxPcieInitialized = 0;
    return 0;
}

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

int PcieHalSendTlp(const PcieTlpPacket* packet)
{
    (void)packet;
    /* TLP-level operations not directly supported via sysfs */
    /* Would require kernel driver or VFIO interface */
    return -1;
}

int PcieHalReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    (void)packet;
    (void)timeoutMs;
    /* TLP-level operations not directly supported via sysfs */
    return -1;
}

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
    (void)memset_s(status, sizeof(PcieLinkStatus), 0, sizeof(PcieLinkStatus));

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

int PcieHalMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    /* Direct memory access requires /dev/mem and root privileges */
    /* Not implemented for safety reasons */
    return -1;
}

int PcieHalMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    (void)address;
    (void)data;
    (void)size;
    /* Direct memory access requires /dev/mem and root privileges */
    return -1;
}

int PcieHalEnableInterrupts(void)
{
    /* Interrupt management handled by kernel drivers */
    return 0;
}

int PcieHalDisableInterrupts(void)
{
    /* Interrupt management handled by kernel drivers */
    return 0;
}

#endif /* PLATFORM_LINUX */

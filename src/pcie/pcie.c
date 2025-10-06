/**
 * @file pcie.c
 * @brief PCIe (PCI Express) protocol implementation
 * @details Provides protocol-layer implementation for PCIe communication
 */

#include <string.h>
#include "pcie.h"
#include "pcie_hal.h"
#include "securec.h"


/* Internal state tracking */
static uint8_t pcieInitialized = 0;
static PcieConfig currentConfig = {0};

/******************************************************************************
 * @brief      : Validate bus, device, function parameters
 * @param[in]  : bus --Bus number device --Device number function --Function number
 * @param[out] :
 * @return     : 0 if valid, -1 if invalid
 * @note       :
 *****************************************************************************/
static int ValidateBDF(uint8_t bus, uint8_t device, uint8_t function)
{
    if (device >= PCIE_MAX_DEVICE) {
        return -1;
    }
    if (function >= PCIE_MAX_FUNCTION) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief      : Validate configuration space offset and size
 * @param[in]  : offset --Register offset size --Access size
 * @param[out] :
 * @return     : 0 if valid, -1 if invalid
 * @note       :
 *****************************************************************************/
static int ValidateConfigAccess(uint16_t offset, uint8_t size)
{
    if (offset >= PCIE_CONFIG_SPACE_SIZE) {
        return -1;
    }
    if (size != 1 && size != 2 && size != 4) {
        return -1;
    }
    if ((offset + size) > PCIE_CONFIG_SPACE_SIZE) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief      : Initialize PCIe controller
 * @param[in]  : config --Pointer to PCIe configuration structure
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieInit(const PcieConfig* config)
{
    if (config == NULL) {
        return -1;
    }

    /* Validate configuration parameters */
    if (config->targetGeneration < PCIE_GEN1 || config->targetGeneration > PCIE_GEN4) {
        return -1;
    }

    /* Validate link width */
    if (config->targetLinkWidth != PCIE_LINK_WIDTH_X1 && config->targetLinkWidth != PCIE_LINK_WIDTH_X2
        && config->targetLinkWidth != PCIE_LINK_WIDTH_X4 && config->targetLinkWidth != PCIE_LINK_WIDTH_X8
        && config->targetLinkWidth != PCIE_LINK_WIDTH_X16 && config->targetLinkWidth != PCIE_LINK_WIDTH_X32) {
        return -1;
    }

    /* Initialize hardware layer */
    if (PcieHalInit(config) != 0) {
        return -1;
    }

    /* Save current configuration */
    if (memcpy_s(&currentConfig, sizeof(PcieConfig), config, sizeof(PcieConfig)) != EOK) {
        PcieHalDeinit();
        return -1;
    }

    pcieInitialized = 1;
    return 0;
}

/******************************************************************************
 * @brief      : Deinitialize PCIe controller
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieDeinit(void)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    /* Deinitialize hardware layer */
    if (PcieHalDeinit() != 0) {
        return -1;
    }

    pcieInitialized = 0;
    if (memset_s(&currentConfig, sizeof(PcieConfig), 0, sizeof(PcieConfig)) != EOK) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief      : Read from PCIe configuration space
 * @param[in]  : bus --Bus number (0-255) device --Device number (0-31) function --Function number (0-7) offset --Register offset in configuration space size --Size of data to read (1, 2, or 4 bytes)
 * @param[out] : data --Pointer to store read data
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    if (data == NULL) {
        return -1;
    }

    /* Validate parameters */
    if (ValidateBDF(bus, device, function) != 0) {
        return -1;
    }

    if (ValidateConfigAccess(offset, size) != 0) {
        return -1;
    }

    /* Perform hardware read */
    return PcieHalConfigRead(bus, device, function, offset, data, size);
}

/******************************************************************************
 * @brief      : Write to PCIe configuration space
 * @param[in]  : bus --Bus number (0-255) device --Device number (0-31) function --Function number (0-7) offset --Register offset in configuration space data --Data to write size --Size of data to write (1, 2, or 4 bytes)
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    /* Validate parameters */
    if (ValidateBDF(bus, device, function) != 0) {
        return -1;
    }

    if (ValidateConfigAccess(offset, size) != 0) {
        return -1;
    }

    /* Perform hardware write */
    return PcieHalConfigWrite(bus, device, function, offset, data, size);
}

/******************************************************************************
 * @brief      : Send TLP packet
 * @param[in]  : packet --Pointer to TLP packet structure
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieSendTlp(const PcieTlpPacket* packet)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    if (packet == NULL) {
        return -1;
    }

    /* Validate packet length */
    if (packet->length > PCIE_MAX_TLP_SIZE) {
        return -1;
    }

    /* Send through hardware layer */
    return PcieHalSendTlp(packet);
}

/******************************************************************************
 * @brief      : Receive TLP packet
 * @param[in]  : timeoutMs --Timeout in milliseconds
 * @param[out] : packet --Pointer to TLP packet structure to store received data
 * @return     : 0 on success, -1 on error or timeout
 * @note       :
 *****************************************************************************/
int PcieReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    if (packet == NULL) {
        return -1;
    }

    /* Clear packet structure */
    if (memset_s(packet, sizeof(PcieTlpPacket), 0, sizeof(PcieTlpPacket)) != EOK) {
        return -1;
    }

    /* Receive through hardware layer */
    return PcieHalReceiveTlp(packet, timeoutMs);
}

/******************************************************************************
 * @brief      : Get current PCIe link status
 * @param[in]  :
 * @param[out] : status --Pointer to link status structure
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieGetLinkStatus(PcieLinkStatus* status)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    if (status == NULL) {
        return -1;
    }

    /* Clear status structure */
    if (memset_s(status, sizeof(PcieLinkStatus), 0, sizeof(PcieLinkStatus)) != EOK) {
        return -1;
    }

    /* Get status from hardware layer */
    return PcieHalGetLinkStatus(status);
}

/******************************************************************************
 * @brief      : Enumerate PCIe devices on the bus
 * @param[in]  : maxDevices --Maximum number of devices to enumerate
 * @param[out] : devices --Array to store discovered device information deviceCount --Pointer to store actual number of devices found
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieEnumerateDevices(PcieDeviceInfo* devices, uint32_t maxDevices, uint32_t* deviceCount)
{
    uint8_t bus, device, function;
    uint32_t vendorDevice;
    uint32_t count = 0;

    if (pcieInitialized == 0) {
        return -1;
    }

    if (devices == NULL || deviceCount == NULL) {
        return -1;
    }

    if (maxDevices == 0) {
        return -1;
    }

    *deviceCount = 0;

    /* Scan all buses, devices, and functions */
    for (bus = 0; bus < PCIE_MAX_BUS && count < maxDevices; bus++) {
        for (device = 0; device < PCIE_MAX_DEVICE && count < maxDevices; device++) {
            for (function = 0; function < PCIE_MAX_FUNCTION && count < maxDevices; function++) {
                /* Read vendor ID and device ID */
                if (PcieConfigRead(bus, device, function, PCIE_CFG_VENDOR_ID, &vendorDevice, 4) != 0) {
                    continue;
                }

                /* Check if device exists (vendor ID != 0xFFFF) */
                uint16_t vendorId = (uint16_t)(vendorDevice & 0xFFFF);
                if (vendorId == 0xFFFF || vendorId == 0x0000) {
                    /* No device present, skip remaining functions */
                    if (function == 0) {
                        break;
                    }
                    continue;
                }

                /* Device found, populate information */
                devices[count].bus = bus;
                devices[count].device = device;
                devices[count].function = function;
                devices[count].vendorId = vendorId;
                devices[count].deviceId = (uint16_t)((vendorDevice >> 16) & 0xFFFF);

                /* Read class code */
                uint32_t classData;
                if (PcieConfigRead(bus, device, function, PCIE_CFG_CLASS_CODE, &classData, 4) == 0) {
                    devices[count].classCode[0] = (uint8_t)((classData >> 24) & 0xFF);
                    devices[count].classCode[1] = (uint8_t)((classData >> 16) & 0xFF);
                    devices[count].classCode[2] = (uint8_t)((classData >> 8) & 0xFF);
                }

                /* Read header type */
                uint32_t headerData;
                if (PcieConfigRead(bus, device, function, PCIE_CFG_HEADER_TYPE, &headerData, 1) == 0) {
                    devices[count].headerType = (uint8_t)(headerData & 0xFF);
                }

                count++;

                /* If this is function 0 and not multi-function, skip */
                if (function == 0 && (devices[count - 1].headerType & 0x80) == 0) {
                    break;
                }
            }
        }
    }

    *deviceCount = count;
    return 0;
}

/******************************************************************************
 * @brief      : Read PCIe memory-mapped region
 * @param[in]  : address --Physical address to read from size --Number of bytes to read
 * @param[out] : data --Buffer to store read data
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieMemoryRead(uint64_t address, uint8_t* data, uint32_t size)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    if (data == NULL || size == 0) {
        return -1;
    }

    return PcieHalMemoryRead(address, data, size);
}

/******************************************************************************
 * @brief      : Write to PCIe memory-mapped region
 * @param[in]  : address --Physical address to write to data --Data to write size --Number of bytes to write
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    if (data == NULL || size == 0) {
        return -1;
    }

    return PcieHalMemoryWrite(address, data, size);
}

/******************************************************************************
 * @brief      : Enable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieEnableInterrupts(void)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    return PcieHalEnableInterrupts();
}

/******************************************************************************
 * @brief      : Disable PCIe interrupts
 * @param[in]  :
 * @param[out] :
 * @return     : 0 on success, -1 on error
 * @note       :
 *****************************************************************************/
int PcieDisableInterrupts(void)
{
    if (pcieInitialized == 0) {
        return -1;
    }

    return PcieHalDisableInterrupts();
}

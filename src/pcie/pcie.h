/**
 * @file pcie.h
 * @brief PCIe (PCI Express) protocol interface
 * @details Provides high-level PCIe communication functions including configuration
 *          space access, TLP packet handling, and device enumeration
 */

#ifndef PCIE_H
#define PCIE_H

#include <stddef.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* PCIe Generation Definitions */
#define PCIE_GEN1_SPEED 2500000000ULL  /**< PCIe Gen1: 2.5 GT/s */
#define PCIE_GEN2_SPEED 5000000000ULL  /**< PCIe Gen2: 5.0 GT/s */
#define PCIE_GEN3_SPEED 8000000000ULL  /**< PCIe Gen3: 8.0 GT/s */
#define PCIE_GEN4_SPEED 16000000000ULL /**< PCIe Gen4: 16.0 GT/s */

/* PCIe Link Width Definitions */
#define PCIE_LINK_WIDTH_X1 1   /**< x1 lane */
#define PCIE_LINK_WIDTH_X2 2   /**< x2 lanes */
#define PCIE_LINK_WIDTH_X4 4   /**< x4 lanes */
#define PCIE_LINK_WIDTH_X8 8   /**< x8 lanes */
#define PCIE_LINK_WIDTH_X16 16 /**< x16 lanes */
#define PCIE_LINK_WIDTH_X32 32 /**< x32 lanes */

/* PCIe Device Types */
#define PCIE_DEVICE_TYPE_ENDPOINT 0x0     /**< PCIe endpoint device */
#define PCIE_DEVICE_TYPE_ROOT_COMPLEX 0x4 /**< PCIe root complex */
#define PCIE_DEVICE_TYPE_SWITCH_UP 0x5    /**< PCIe switch upstream port */
#define PCIE_DEVICE_TYPE_SWITCH_DOWN 0x6  /**< PCIe switch downstream port */

/* Configuration Space Access Types */
#define PCIE_CONFIG_TYPE0 0 /**< Type 0 configuration (direct device) */
#define PCIE_CONFIG_TYPE1 1 /**< Type 1 configuration (through bridge) */

/* TLP (Transaction Layer Packet) Types */
#define PCIE_TLP_TYPE_MEM_READ 0x00        /**< Memory read request */
#define PCIE_TLP_TYPE_MEM_WRITE 0x40       /**< Memory write request */
#define PCIE_TLP_TYPE_IO_READ 0x02         /**< I/O read request */
#define PCIE_TLP_TYPE_IO_WRITE 0x42        /**< I/O write request */
#define PCIE_TLP_TYPE_CFG_READ0 0x04       /**< Configuration read Type 0 */
#define PCIE_TLP_TYPE_CFG_WRITE0 0x44      /**< Configuration write Type 0 */
#define PCIE_TLP_TYPE_CFG_READ1 0x05       /**< Configuration read Type 1 */
#define PCIE_TLP_TYPE_CFG_WRITE1 0x45      /**< Configuration write Type 1 */
#define PCIE_TLP_TYPE_MSG 0x30             /**< Message request */
#define PCIE_TLP_TYPE_COMPLETION 0x0A      /**< Completion without data */
#define PCIE_TLP_TYPE_COMPLETION_DATA 0x4A /**< Completion with data */

/* PCIe Configuration Space Offsets */
#define PCIE_CFG_VENDOR_ID 0x00   /**< Vendor ID register */
#define PCIE_CFG_DEVICE_ID 0x02   /**< Device ID register */
#define PCIE_CFG_COMMAND 0x04     /**< Command register */
#define PCIE_CFG_STATUS 0x06      /**< Status register */
#define PCIE_CFG_CLASS_CODE 0x09  /**< Class code register */
#define PCIE_CFG_HEADER_TYPE 0x0E /**< Header type register */
#define PCIE_CFG_BAR0 0x10        /**< Base Address Register 0 */

/* Maximum Values */
#define PCIE_MAX_BUS 256           /**< Maximum bus number */
#define PCIE_MAX_DEVICE 32         /**< Maximum device number per bus */
#define PCIE_MAX_FUNCTION 8        /**< Maximum function number per device */
#define PCIE_MAX_TLP_SIZE 4096     /**< Maximum TLP packet size in bytes */
#define PCIE_CONFIG_SPACE_SIZE 256 /**< Standard configuration space size */

/**
 * @brief PCIe generation enumeration
 */
typedef enum {
    PCIE_GEN1 = 1, /**< PCIe Generation 1 (2.5 GT/s) */
    PCIE_GEN2 = 2, /**< PCIe Generation 2 (5.0 GT/s) */
    PCIE_GEN3 = 3, /**< PCIe Generation 3 (8.0 GT/s) */
    PCIE_GEN4 = 4  /**< PCIe Generation 4 (16.0 GT/s) */
} PcieGeneration;

/**
 * @brief PCIe link status structure
 */
typedef struct {
    PcieGeneration generation; /**< Current PCIe generation */
    uint8_t linkWidth;         /**< Current link width (x1, x2, x4, etc.) */
    uint8_t maxLinkWidth;      /**< Maximum supported link width */
    uint8_t isLinkUp;          /**< Link status: 1 = up, 0 = down */
} PcieLinkStatus;

/**
 * @brief PCIe device information structure
 */
typedef struct {
    uint8_t bus;          /**< Bus number */
    uint8_t device;       /**< Device number */
    uint8_t function;     /**< Function number */
    uint16_t vendorId;    /**< Vendor ID */
    uint16_t deviceId;    /**< Device ID */
    uint8_t classCode[3]; /**< Class code (base, sub, prog IF) */
    uint8_t headerType;   /**< Header type */
} PcieDeviceInfo;

/**
 * @brief PCIe TLP packet structure
 */
typedef struct {
    uint8_t type;                    /**< TLP type */
    uint8_t format;                  /**< TLP format */
    uint16_t requesterID;            /**< Requester ID (Bus:Device:Function) */
    uint8_t tag;                     /**< Transaction tag */
    uint64_t address;                /**< Memory/IO address */
    uint32_t length;                 /**< Data length in bytes */
    uint8_t data[PCIE_MAX_TLP_SIZE]; /**< Payload data */
} PcieTlpPacket;

/**
 * @brief PCIe configuration structure
 */
typedef struct {
    PcieGeneration targetGeneration; /**< Target PCIe generation */
    uint8_t targetLinkWidth;         /**< Target link width */
    uint8_t enableHotplug;           /**< Enable hot-plug support */
    uint8_t enableAER;               /**< Enable Advanced Error Reporting */
} PcieConfig;

/**
 * @brief Initialize PCIe controller
 * @param config Pointer to PCIe configuration structure
 * @return 0 on success, -1 on error
 */
int PcieInit(const PcieConfig* config);

/**
 * @brief Deinitialize PCIe controller
 * @return 0 on success, -1 on error
 */
int PcieDeinit(void);

/**
 * @brief Read from PCIe configuration space
 * @param bus Bus number (0-255)
 * @param device Device number (0-31)
 * @param function Function number (0-7)
 * @param offset Register offset in configuration space
 * @param data Pointer to store read data
 * @param size Size of data to read (1, 2, or 4 bytes)
 * @return 0 on success, -1 on error
 */
int PcieConfigRead(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t* data, uint8_t size);

/**
 * @brief Write to PCIe configuration space
 * @param bus Bus number (0-255)
 * @param device Device number (0-31)
 * @param function Function number (0-7)
 * @param offset Register offset in configuration space
 * @param data Data to write
 * @param size Size of data to write (1, 2, or 4 bytes)
 * @return 0 on success, -1 on error
 */
int PcieConfigWrite(uint8_t bus, uint8_t device, uint8_t function, uint16_t offset, uint32_t data, uint8_t size);

/**
 * @brief Send TLP packet
 * @param packet Pointer to TLP packet structure
 * @return 0 on success, -1 on error
 */
int PcieSendTlp(const PcieTlpPacket* packet);

/**
 * @brief Receive TLP packet
 * @param packet Pointer to TLP packet structure to store received data
 * @param timeoutMs Timeout in milliseconds
 * @return 0 on success, -1 on error or timeout
 */
int PcieReceiveTlp(PcieTlpPacket* packet, uint32_t timeoutMs);

/**
 * @brief Get current PCIe link status
 * @param status Pointer to link status structure
 * @return 0 on success, -1 on error
 */
int PcieGetLinkStatus(PcieLinkStatus* status);

/**
 * @brief Enumerate PCIe devices on the bus
 * @param devices Array to store discovered device information
 * @param maxDevices Maximum number of devices to enumerate
 * @param deviceCount Pointer to store actual number of devices found
 * @return 0 on success, -1 on error
 */
int PcieEnumerateDevices(PcieDeviceInfo* devices, uint32_t maxDevices, uint32_t* deviceCount);

/**
 * @brief Read PCIe memory-mapped region
 * @param address Physical address to read from
 * @param data Buffer to store read data
 * @param size Number of bytes to read
 * @return 0 on success, -1 on error
 */
int PcieMemoryRead(uint64_t address, uint8_t* data, uint32_t size);

/**
 * @brief Write to PCIe memory-mapped region
 * @param address Physical address to write to
 * @param data Data to write
 * @param size Number of bytes to write
 * @return 0 on success, -1 on error
 */
int PcieMemoryWrite(uint64_t address, const uint8_t* data, uint32_t size);

/**
 * @brief Enable PCIe interrupts
 * @return 0 on success, -1 on error
 */
int PcieEnableInterrupts(void);

/**
 * @brief Disable PCIe interrupts
 * @return 0 on success, -1 on error
 */
int PcieDisableInterrupts(void);

#ifdef __cplusplus
}
#endif

#endif /* PCIE_H */

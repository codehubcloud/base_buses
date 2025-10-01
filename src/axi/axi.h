/**
 * @file axi.h
 * @brief AXI (Advanced eXtensible Interface) bus protocol interface
 * @details Implements AXI4, AXI4-Lite, and AXI4-Stream protocol support
 *          with complete channel management for read/write transactions
 */

#ifndef AXI_H
#define AXI_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup AXI_Constants AXI Protocol Constants
 * @{
 */

/** Maximum burst length for AXI4 (256 transfers) */
#define AXI_MAX_BURST_LEN 256

/** Maximum burst length for AXI4-Lite (1 transfer) */
#define AXI_LITE_MAX_BURST_LEN 1

/** @} */

/**
 * @defgroup AXI_BurstTypes AXI Burst Types
 * @{
 */

/** Burst type enumeration (AxBURST[1:0]) */
typedef enum {
    AXI_BURST_FIXED = 0x00,  /**< Fixed address burst */
    AXI_BURST_INCR = 0x01,   /**< Incrementing address burst */
    AXI_BURST_WRAP = 0x02,   /**< Wrapping address burst */
    AXI_BURST_RESERVED = 0x03 /**< Reserved */
} AxiBurstType;

/** @} */

/**
 * @defgroup AXI_ResponseTypes AXI Response Types
 * @{
 */

/** Response type enumeration (xRESP[1:0]) */
typedef enum {
    AXI_RESP_OKAY = 0x00,   /**< Normal access success */
    AXI_RESP_EXOKAY = 0x01, /**< Exclusive access okay */
    AXI_RESP_SLVERR = 0x02, /**< Slave error */
    AXI_RESP_DECERR = 0x03  /**< Decode error */
} AxiResponseType;

/** @} */

/**
 * @defgroup AXI_SizeTypes AXI Transfer Size Types
 * @{
 */

/** Transfer size enumeration (AxSIZE[2:0]) */
typedef enum {
    AXI_SIZE_1_BYTE = 0x00,    /**< 1 byte (8 bits) */
    AXI_SIZE_2_BYTES = 0x01,   /**< 2 bytes (16 bits) */
    AXI_SIZE_4_BYTES = 0x02,   /**< 4 bytes (32 bits) */
    AXI_SIZE_8_BYTES = 0x03,   /**< 8 bytes (64 bits) */
    AXI_SIZE_16_BYTES = 0x04,  /**< 16 bytes (128 bits) */
    AXI_SIZE_32_BYTES = 0x05,  /**< 32 bytes (256 bits) */
    AXI_SIZE_64_BYTES = 0x06,  /**< 64 bytes (512 bits) */
    AXI_SIZE_128_BYTES = 0x07  /**< 128 bytes (1024 bits) */
} AxiSizeType;

/** @} */

/**
 * @defgroup AXI_ProtectionTypes AXI Protection Types
 * @{
 */

/** Protection attributes (AxPROT[2:0]) */
typedef enum {
    AXI_PROT_PRIVILEGED = 0x01,   /**< Privileged access */
    AXI_PROT_NONSECURE = 0x02,    /**< Non-secure access */
    AXI_PROT_INSTRUCTION = 0x04   /**< Instruction access */
} AxiProtectionType;

/** @} */

/**
 * @defgroup AXI_CacheTypes AXI Cache Types
 * @{
 */

/** Cache attributes (AxCACHE[3:0]) */
typedef enum {
    AXI_CACHE_DEVICE_NON_BUFFERABLE = 0x00,  /**< Device non-bufferable */
    AXI_CACHE_DEVICE_BUFFERABLE = 0x01,      /**< Device bufferable */
    AXI_CACHE_NORMAL_NON_CACHEABLE = 0x02,   /**< Normal non-cacheable */
    AXI_CACHE_WRITE_THROUGH = 0x06,          /**< Write-through */
    AXI_CACHE_WRITE_BACK = 0x07,             /**< Write-back */
    AXI_CACHE_ALLOCATE_ON_WRITE = 0x08,      /**< Allocate on write */
    AXI_CACHE_ALLOCATE_ON_READ = 0x04        /**< Allocate on read */
} AxiCacheType;

/** @} */

/**
 * @defgroup AXI_LockTypes AXI Lock Types
 * @{
 */

/** Lock type enumeration (AxLOCK) */
typedef enum {
    AXI_LOCK_NORMAL = 0x00,    /**< Normal access */
    AXI_LOCK_EXCLUSIVE = 0x01  /**< Exclusive access */
} AxiLockType;

/** @} */

/**
 * @defgroup AXI_Structures AXI Data Structures
 * @{
 */

/**
 * @brief AXI write address channel structure
 */
typedef struct {
    uint32_t address;           /**< Write address (AWADDR) */
    uint8_t burstLength;        /**< Burst length (AWLEN) */
    AxiSizeType burstSize;      /**< Burst size (AWSIZE) */
    AxiBurstType burstType;     /**< Burst type (AWBURST) */
    AxiLockType lock;           /**< Lock type (AWLOCK) */
    uint8_t cache;              /**< Cache type (AWCACHE) */
    uint8_t protection;         /**< Protection type (AWPROT) */
    uint8_t qos;                /**< Quality of Service (AWQOS) */
    uint8_t region;             /**< Region identifier (AWREGION) */
    uint16_t id;                /**< Transaction ID (AWID) */
    uint8_t user;               /**< User-defined signal (AWUSER) */
} AxiWriteAddress;

/**
 * @brief AXI write data channel structure
 */
typedef struct {
    uint8_t *data;              /**< Write data pointer (WDATA) */
    uint8_t *strobe;            /**< Write strobe pointer (WSTRB) */
    uint8_t last;               /**< Last transfer flag (WLAST) */
    uint8_t user;               /**< User-defined signal (WUSER) */
} AxiWriteData;

/**
 * @brief AXI write response channel structure
 */
typedef struct {
    AxiResponseType response;   /**< Write response (BRESP) */
    uint16_t id;                /**< Transaction ID (BID) */
    uint8_t user;               /**< User-defined signal (BUSER) */
} AxiWriteResponse;

/**
 * @brief AXI read address channel structure
 */
typedef struct {
    uint32_t address;           /**< Read address (ARADDR) */
    uint8_t burstLength;        /**< Burst length (ARLEN) */
    AxiSizeType burstSize;      /**< Burst size (ARSIZE) */
    AxiBurstType burstType;     /**< Burst type (ARBURST) */
    AxiLockType lock;           /**< Lock type (ARLOCK) */
    uint8_t cache;              /**< Cache type (ARCACHE) */
    uint8_t protection;         /**< Protection type (ARPROT) */
    uint8_t qos;                /**< Quality of Service (ARQOS) */
    uint8_t region;             /**< Region identifier (ARREGION) */
    uint16_t id;                /**< Transaction ID (ARID) */
    uint8_t user;               /**< User-defined signal (ARUSER) */
} AxiReadAddress;

/**
 * @brief AXI read data channel structure
 */
typedef struct {
    uint8_t *data;              /**< Read data pointer (RDATA) */
    AxiResponseType response;   /**< Read response (RRESP) */
    uint8_t last;               /**< Last transfer flag (RLAST) */
    uint16_t id;                /**< Transaction ID (RID) */
    uint8_t user;               /**< User-defined signal (RUSER) */
} AxiReadData;

/**
 * @brief AXI4-Stream data structure
 */
typedef struct {
    uint8_t *data;              /**< Stream data pointer (TDATA) */
    uint8_t *strobe;            /**< Stream byte qualifier (TSTRB) */
    uint8_t *keep;              /**< Stream byte keep (TKEEP) */
    uint8_t last;               /**< Last transfer flag (TLAST) */
    uint16_t id;                /**< Stream ID (TID) */
    uint16_t dest;              /**< Stream destination (TDEST) */
    uint8_t user;               /**< User-defined signal (TUSER) */
    size_t length;              /**< Data length in bytes */
} AxiStreamData;

/**
 * @brief AXI bus configuration structure
 */
typedef struct {
    uint8_t dataWidth;          /**< Data bus width (32, 64, 128, 256, 512, 1024) */
    uint8_t addressWidth;       /**< Address bus width (typically 32 or 64) */
    uint8_t idWidth;            /**< ID width (transaction identifier) */
    uint8_t maxBurstLength;     /**< Maximum burst length supported */
    uint8_t isLite;             /**< AXI4-Lite mode flag (1 = lite, 0 = full) */
    uint32_t timeout;           /**< Transaction timeout in milliseconds */
} AxiConfig;

/**
 * @brief AXI bus status structure
 */
typedef struct {
    uint8_t isInitialized;      /**< Initialization status */
    uint8_t isBusy;             /**< Bus busy status */
    uint32_t errorCount;        /**< Total error count */
    uint32_t writeCount;        /**< Total write transactions */
    uint32_t readCount;         /**< Total read transactions */
    AxiResponseType lastError;  /**< Last error response */
} AxiStatus;

/** @} */

/**
 * @defgroup AXI_Functions AXI Function Declarations
 * @{
 */

/**
 * @brief Initialize AXI bus interface
 * @param[in] config Configuration structure pointer
 * @return 0 on success, -1 on failure
 */
int AxiInit(const AxiConfig *config);

/**
 * @brief Deinitialize AXI bus interface
 * @return 0 on success, -1 on failure
 */
int AxiDeinit(void);

/**
 * @brief Perform single AXI write transaction
 * @param[in] writeAddr Write address channel structure
 * @param[in] writeData Write data channel structure
 * @param[out] writeResp Write response channel structure
 * @return 0 on success, -1 on failure
 */
int AxiWriteTransaction(const AxiWriteAddress *writeAddr,
                        const AxiWriteData *writeData,
                        AxiWriteResponse *writeResp);

/**
 * @brief Perform single AXI read transaction
 * @param[in] readAddr Read address channel structure
 * @param[out] readData Read data channel structure
 * @return 0 on success, -1 on failure
 */
int AxiReadTransaction(const AxiReadAddress *readAddr,
                       AxiReadData *readData);

/**
 * @brief Perform AXI burst write operation
 * @param[in] writeAddr Write address channel structure
 * @param[in] writeData Array of write data structures
 * @param[in] burstLength Number of transfers in burst
 * @param[out] writeResp Write response channel structure
 * @return 0 on success, -1 on failure
 */
int AxiWriteBurst(const AxiWriteAddress *writeAddr,
                  const AxiWriteData *writeData,
                  uint8_t burstLength,
                  AxiWriteResponse *writeResp);

/**
 * @brief Perform AXI burst read operation
 * @param[in] readAddr Read address channel structure
 * @param[out] readData Array of read data structures
 * @param[in] burstLength Number of transfers in burst
 * @return 0 on success, -1 on failure
 */
int AxiReadBurst(const AxiReadAddress *readAddr,
                 AxiReadData *readData,
                 uint8_t burstLength);

/**
 * @brief Configure AXI Quality of Service settings
 * @param[in] qosValue QoS value (0-15)
 * @return 0 on success, -1 on failure
 */
int AxiSetQoS(uint8_t qosValue);

/**
 * @brief Get current AXI bus status
 * @param[out] status Status structure pointer
 * @return 0 on success, -1 on failure
 */
int AxiGetStatus(AxiStatus *status);

/**
 * @brief Send data via AXI4-Stream interface
 * @param[in] streamData Stream data structure
 * @return 0 on success, -1 on failure
 */
int AxiStreamSend(const AxiStreamData *streamData);

/**
 * @brief Receive data via AXI4-Stream interface
 * @param[out] streamData Stream data structure
 * @param[in] maxLength Maximum length to receive
 * @return Number of bytes received on success, -1 on failure
 */
int AxiStreamReceive(AxiStreamData *streamData, size_t maxLength);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* AXI_H */

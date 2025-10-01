#ifndef EMMC_H
#define EMMC_H

#include <stdint.h>

/* eMMC Command Definitions (CMD0-CMD63) */
#define EMMC_CMD0_GO_IDLE_STATE 0
#define EMMC_CMD1_SEND_OP_COND 1
#define EMMC_CMD2_ALL_SEND_CID 2
#define EMMC_CMD3_SET_RELATIVE_ADDR 3
#define EMMC_CMD6_SWITCH 6
#define EMMC_CMD7_SELECT_CARD 7
#define EMMC_CMD8_SEND_EXT_CSD 8
#define EMMC_CMD9_SEND_CSD 9
#define EMMC_CMD10_SEND_CID 10
#define EMMC_CMD12_STOP_TRANSMISSION 12
#define EMMC_CMD13_SEND_STATUS 13
#define EMMC_CMD16_SET_BLOCKLEN 16
#define EMMC_CMD17_READ_SINGLE_BLOCK 17
#define EMMC_CMD18_READ_MULTIPLE_BLOCK 18
#define EMMC_CMD23_SET_BLOCK_COUNT 23
#define EMMC_CMD24_WRITE_BLOCK 24
#define EMMC_CMD25_WRITE_MULTIPLE_BLOCK 25
#define EMMC_CMD35_ERASE_GROUP_START 35
#define EMMC_CMD36_ERASE_GROUP_END 36
#define EMMC_CMD38_ERASE 38

/* eMMC Bus Width Configuration */
typedef enum { EMMC_BUS_WIDTH_1BIT = 0, EMMC_BUS_WIDTH_4BIT = 1, EMMC_BUS_WIDTH_8BIT = 2 } EmmcBusWidth_E;

/* eMMC Speed Mode Configuration */
typedef enum {
    EMMC_SPEED_MODE_LEGACY = 0,     /* Legacy speed (up to 26MHz) */
    EMMC_SPEED_MODE_HIGH_SPEED = 1, /* High speed (up to 52MHz) */
    EMMC_SPEED_MODE_HS200 = 2,      /* HS200 (up to 200MHz, SDR) */
    EMMC_SPEED_MODE_HS400 = 3,      /* HS400 (up to 200MHz, DDR) */
    EMMC_SPEED_MODE_DDR52 = 4       /* DDR52 (up to 52MHz, DDR) */
} EmmcSpeedMode_E;

/* eMMC Partition Type */
typedef enum {
    EMMC_PARTITION_USER = 0,  /* User data area */
    EMMC_PARTITION_BOOT1 = 1, /* Boot partition 1 */
    EMMC_PARTITION_BOOT2 = 2, /* Boot partition 2 */
    EMMC_PARTITION_RPMB = 3   /* Replay Protected Memory Block */
} EmmcPartition_E;

/* eMMC Card Status Flags */
#define EMMC_STATUS_READY_FOR_DATA (1U << 8)
#define EMMC_STATUS_CURRENT_STATE_MASK (0xFU << 9)
#define EMMC_STATUS_ERASE_RESET (1U << 13)
#define EMMC_STATUS_CARD_ECC_DISABLED (1U << 14)
#define EMMC_STATUS_WP_ERASE_SKIP (1U << 15)
#define EMMC_STATUS_ERROR (1U << 19)
#define EMMC_STATUS_CC_ERROR (1U << 20)
#define EMMC_STATUS_CARD_ECC_FAILED (1U << 21)
#define EMMC_STATUS_ILLEGAL_COMMAND (1U << 22)
#define EMMC_STATUS_COM_CRC_ERROR (1U << 23)
#define EMMC_STATUS_LOCK_UNLOCK_FAILED (1U << 24)
#define EMMC_STATUS_CARD_IS_LOCKED (1U << 25)
#define EMMC_STATUS_WP_VIOLATION (1U << 26)
#define EMMC_STATUS_ERASE_PARAM (1U << 27)
#define EMMC_STATUS_ERASE_SEQ_ERROR (1U << 28)
#define EMMC_STATUS_BLOCK_LEN_ERROR (1U << 29)
#define EMMC_STATUS_ADDRESS_ERROR (1U << 30)
#define EMMC_STATUS_OUT_OF_RANGE (1U << 31)

/* eMMC Current States */
#define EMMC_CURRENT_STATE_IDLE 0
#define EMMC_CURRENT_STATE_READY 1
#define EMMC_CURRENT_STATE_IDENT 2
#define EMMC_CURRENT_STATE_STBY 3
#define EMMC_CURRENT_STATE_TRAN 4
#define EMMC_CURRENT_STATE_DATA 5
#define EMMC_CURRENT_STATE_RCV 6
#define EMMC_CURRENT_STATE_PRG 7
#define EMMC_CURRENT_STATE_DIS 8

/* eMMC Block Size */
#define EMMC_BLOCK_SIZE 512

/* eMMC CID Structure (Card Identification) */
typedef struct {
    uint8_t manufacturerId;       /* Manufacturer ID */
    uint16_t oemApplicationId;    /* OEM/Application ID */
    uint8_t productName[6];       /* Product name (6 bytes) */
    uint8_t productRevision;      /* Product revision */
    uint32_t productSerialNumber; /* Product serial number */
    uint16_t manufacturingDate;   /* Manufacturing date */
    uint8_t crc7;                 /* CRC7 checksum */
} EmmcCid_S;

/* eMMC CSD Structure (Card Specific Data) */
typedef struct {
    uint8_t csdStructure;     /* CSD structure version */
    uint8_t specVersion;      /* System specification version */
    uint8_t taac;             /* Data read access-time */
    uint8_t nsac;             /* Data read access-time in CLK cycles */
    uint8_t tranSpeed;        /* Max data transfer rate */
    uint16_t ccc;             /* Card command classes */
    uint8_t readBlLen;        /* Max read data block length */
    uint8_t readBlPartial;    /* Partial blocks for read allowed */
    uint8_t writeBlkMisalign; /* Write block misalignment */
    uint8_t readBlkMisalign;  /* Read block misalignment */
    uint8_t dsrImp;           /* DSR implemented */
    uint32_t cSize;           /* Device size */
    uint8_t eraseGrpSize;     /* Erase group size */
    uint8_t eraseGrpMult;     /* Erase group size multiplier */
    uint8_t wpGrpSize;        /* Write protect group size */
    uint8_t wpGrpEnable;      /* Write protect group enable */
    uint8_t r2wFactor;        /* Write speed factor */
    uint8_t writeBlLen;       /* Max write data block length */
    uint8_t writeBlPartial;   /* Partial blocks for write allowed */
    uint8_t fileFormatGrp;    /* File format group */
    uint8_t copy;             /* Copy flag */
    uint8_t permWriteProtect; /* Permanent write protection */
    uint8_t tmpWriteProtect;  /* Temporary write protection */
    uint8_t fileFormat;       /* File format */
    uint8_t crc7;             /* CRC7 checksum */
} EmmcCsd_S;

/* eMMC Extended CSD Structure (simplified, key fields only) */
typedef struct {
    uint8_t extCsdRev;          /* Extended CSD revision */
    uint8_t cmdSet;             /* Command set */
    uint32_t sectorCount;       /* Sector count (in 512-byte sectors) */
    uint8_t busWidth;           /* Bus width mode */
    uint8_t hsTimingMode;       /* High-speed interface timing */
    uint8_t powerClass;         /* Power class */
    uint8_t cardType;           /* Card type */
    uint8_t driverStrength;     /* Driver strength */
    uint8_t partitionConfig;    /* Partition configuration */
    uint8_t bootBusWidth;       /* Boot bus width */
    uint8_t strobeSupport;      /* Strobe support */
    uint32_t bootPartitionSize; /* Boot partition size (in 128KB units) */
    uint32_t rpmbSize;          /* RPMB size (in 128KB units) */
} EmmcExtCsd_S;

/* eMMC Card Information */
typedef struct {
    EmmcCid_S cid;                    /* Card identification */
    EmmcCsd_S csd;                    /* Card specific data */
    EmmcExtCsd_S extCsd;              /* Extended CSD */
    uint32_t rca;                     /* Relative card address */
    uint64_t capacity;                /* Total capacity in bytes */
    uint32_t blockCount;              /* Total number of blocks */
    uint16_t blockSize;               /* Block size in bytes */
    EmmcBusWidth_E busWidth;          /* Current bus width */
    EmmcSpeedMode_E speedMode;        /* Current speed mode */
    EmmcPartition_E currentPartition; /* Current selected partition */
} EmmcCardInfo_S;

/******************************************************************************
 * @brief     : Initialize eMMC device
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Performs card initialization sequence and identifies card
 *****************************************************************************/
int32_t EmmcInit(void);

/******************************************************************************
 * @brief     : Deinitialize eMMC device
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Powers down eMMC and releases resources
 *****************************************************************************/
int32_t EmmcDeinit(void);

/******************************************************************************
 * @brief     : Read single block from eMMC
 * @param[in] : blockAddress - Block address to read from
 * @param[out]: data - Buffer to store read data (must be at least 512 bytes)
 * @return    : 0 if success, -1 if error
 * @note      : Reads one 512-byte block
 *****************************************************************************/
int32_t EmmcReadBlock(uint32_t blockAddress, uint8_t* data);

/******************************************************************************
 * @brief     : Write single block to eMMC
 * @param[in] : blockAddress - Block address to write to, data - Data to write (must be 512 bytes)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes one 512-byte block
 *****************************************************************************/
int32_t EmmcWriteBlock(uint32_t blockAddress, const uint8_t* data);

/******************************************************************************
 * @brief     : Read multiple blocks from eMMC
 * @param[in] : blockAddress - Starting block address, blockCount - Number of blocks to read
 * @param[out]: data - Buffer to store read data (must be at least blockCount * 512 bytes)
 * @return    : 0 if success, -1 if error
 * @note      : Reads multiple 512-byte blocks
 *****************************************************************************/
int32_t EmmcReadMultipleBlocks(uint32_t blockAddress, uint32_t blockCount, uint8_t* data);

/******************************************************************************
 * @brief     : Write multiple blocks to eMMC
 * @param[in] : blockAddress - Starting block address, blockCount - Number of blocks to write, data - Data to write (must be blockCount *
 *512 bytes)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes multiple 512-byte blocks
 *****************************************************************************/
int32_t EmmcWriteMultipleBlocks(uint32_t blockAddress, uint32_t blockCount, const uint8_t* data);

/******************************************************************************
 * @brief     : Set eMMC bus width
 * @param[in] : busWidth - Bus width to set (1-bit, 4-bit, or 8-bit)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures data bus width for communication
 *****************************************************************************/
int32_t EmmcSetBusWidth(EmmcBusWidth_E busWidth);

/******************************************************************************
 * @brief     : Set eMMC speed mode
 * @param[in] : speedMode - Speed mode to set (Legacy, HS, HS200, HS400, DDR52)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures clock speed and timing mode
 *****************************************************************************/
int32_t EmmcSetSpeedMode(EmmcSpeedMode_E speedMode);

/******************************************************************************
 * @brief     : Get eMMC card information
 * @param[in] : None
 * @param[out]: cardInfo - Pointer to structure to store card information
 * @return    : 0 if success, -1 if error
 * @note      : Reads CID, CSD, EXT_CSD and calculates capacity
 *****************************************************************************/
int32_t EmmcGetCardInfo(EmmcCardInfo_S* cardInfo);

/******************************************************************************
 * @brief     : Select eMMC partition
 * @param[in] : partition - Partition to select (USER, BOOT1, BOOT2, RPMB)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Switches active partition for read/write operations
 *****************************************************************************/
int32_t EmmcSelectPartition(EmmcPartition_E partition);

/******************************************************************************
 * @brief     : Get eMMC card status
 * @param[in] : None
 * @param[out]: status - Pointer to store card status register value
 * @return    : 0 if success, -1 if error
 * @note      : Reads current card status flags
 *****************************************************************************/
int32_t EmmcGetCardStatus(uint32_t* status);

/******************************************************************************
 * @brief     : Erase blocks on eMMC
 * @param[in] : startBlock - Start block address, endBlock - End block address
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Erases blocks from startBlock to endBlock (inclusive)
 *****************************************************************************/
int32_t EmmcEraseBlocks(uint32_t startBlock, uint32_t endBlock);

#endif // EMMC_H

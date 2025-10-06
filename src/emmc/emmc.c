#include <string.h>
#include "emmc.h"
#include "emmc_hal.h"
#include "securec.h"


/* Global card information */
static EmmcCardInfo_S g_emmcCardInfo;
static uint8_t g_emmcInitialized = 0;

/* Helper function prototypes */
static int32_t EmmcSendCommand(uint8_t cmd, uint32_t arg, uint32_t* response);
static int32_t EmmcWaitForReady(void);
static int32_t EmmcIdentifyCard(void);
static uint32_t EmmcExtractBits(const uint8_t* data, uint32_t startBit, uint32_t bitCount);

/******************************************************************************
 * @brief     : Initialize eMMC device
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Performs card initialization sequence and identifies card
 *****************************************************************************/
int32_t EmmcInit(void)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized != 0) {
        return 0;
    }

    /* Initialize HAL layer */
    result = EmmcHalInit();
    if (result != 0) {
        return -1;
    }

    /* Send CMD0 (GO_IDLE_STATE) to reset card */
    result = EmmcSendCommand(EMMC_CMD0_GO_IDLE_STATE, 0, NULL);
    if (result != 0) {
        return -1;
    }

    /* Wait for card to be ready */
    EmmcHalDelay(10);

    /* Send CMD1 (SEND_OP_COND) to get card ready */
    for (uint32_t i = 0; i < 100; i++) {
        result = EmmcSendCommand(EMMC_CMD1_SEND_OP_COND, 0x40FF8000, &response);
        if (result == 0 && (response & 0x80000000) != 0) {
            break;
        }
        EmmcHalDelay(10);
    }

    if ((response & 0x80000000) == 0) {
        return -1;
    }

    /* Identify card and read CID/CSD */
    result = EmmcIdentifyCard();
    if (result != 0) {
        return -1;
    }

    /* Set default block size to 512 bytes */
    result = EmmcSendCommand(EMMC_CMD16_SET_BLOCKLEN, EMMC_BLOCK_SIZE, &response);
    if (result != 0) {
        return -1;
    }

    /* Set default bus width to 1-bit */
    g_emmcCardInfo.busWidth = EMMC_BUS_WIDTH_1BIT;
    g_emmcCardInfo.speedMode = EMMC_SPEED_MODE_LEGACY;
    g_emmcCardInfo.currentPartition = EMMC_PARTITION_USER;
    g_emmcCardInfo.blockSize = EMMC_BLOCK_SIZE;

    g_emmcInitialized = 1;
    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize eMMC device
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Powers down eMMC and releases resources
 *****************************************************************************/
int32_t EmmcDeinit(void)
{
    if (g_emmcInitialized == 0) {
        return 0;
    }

    EmmcHalDeinit();
    g_emmcInitialized = 0;
    if (memset_s(&g_emmcCardInfo, sizeof(g_emmcCardInfo), 0, sizeof(g_emmcCardInfo)) != EOK) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read single block from eMMC
 * @param[in] : blockAddress --Block address to read from
 * @param[out]: data --Buffer to store read data (must be at least 512 bytes)
 * @return    : 0 if success, -1 if error
 * @note      : Reads one 512-byte block
 *****************************************************************************/
int32_t EmmcReadBlock(uint32_t blockAddress, uint8_t* data)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized == 0 || data == NULL) {
        return -1;
    }

    /* Wait for card to be ready */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Send CMD17 (READ_SINGLE_BLOCK) */
    result = EmmcSendCommand(EMMC_CMD17_READ_SINGLE_BLOCK, blockAddress, &response);
    if (result != 0) {
        return -1;
    }

    /* Read data block */
    result = EmmcHalReadData(data, EMMC_BLOCK_SIZE);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write single block to eMMC
 * @param[in] : blockAddress --Block address to write to, data - Data to write (must be 512 bytes)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes one 512-byte block
 *****************************************************************************/
int32_t EmmcWriteBlock(uint32_t blockAddress, const uint8_t* data)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized == 0 || data == NULL) {
        return -1;
    }

    /* Wait for card to be ready */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Send CMD24 (WRITE_BLOCK) */
    result = EmmcSendCommand(EMMC_CMD24_WRITE_BLOCK, blockAddress, &response);
    if (result != 0) {
        return -1;
    }

    /* Write data block */
    result = EmmcHalWriteData(data, EMMC_BLOCK_SIZE);
    if (result != 0) {
        return -1;
    }

    /* Wait for programming to complete */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read multiple blocks from eMMC
 * @param[in] : blockAddress --Starting block address, blockCount - Number of blocks to read
 * @param[out]: data --Buffer to store read data (must be at least blockCount * 512 bytes)
 * @return    : 0 if success, -1 if error
 * @note      : Reads multiple 512-byte blocks
 *****************************************************************************/
int32_t EmmcReadMultipleBlocks(uint32_t blockAddress, uint32_t blockCount, uint8_t* data)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized == 0 || data == NULL || blockCount == 0) {
        return -1;
    }

    /* Wait for card to be ready */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Send CMD23 (SET_BLOCK_COUNT) for pre-defined block count */
    result = EmmcSendCommand(EMMC_CMD23_SET_BLOCK_COUNT, blockCount, &response);
    if (result != 0) {
        return -1;
    }

    /* Send CMD18 (READ_MULTIPLE_BLOCK) */
    result = EmmcSendCommand(EMMC_CMD18_READ_MULTIPLE_BLOCK, blockAddress, &response);
    if (result != 0) {
        return -1;
    }

    /* Read data blocks */
    for (uint32_t i = 0; i < blockCount; i++) {
        result = EmmcHalReadData(&data[i * EMMC_BLOCK_SIZE], EMMC_BLOCK_SIZE);
        if (result != 0) {
            EmmcSendCommand(EMMC_CMD12_STOP_TRANSMISSION, 0, NULL);
            return -1;
        }
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write multiple blocks to eMMC
 * @param[in] : blockAddress --Starting block address, blockCount - Number of blocks to write, data - Data to write (must be blockCount *
 *512 bytes)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes multiple 512-byte blocks
 *****************************************************************************/
int32_t EmmcWriteMultipleBlocks(uint32_t blockAddress, uint32_t blockCount, const uint8_t* data)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized == 0 || data == NULL || blockCount == 0) {
        return -1;
    }

    /* Wait for card to be ready */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Send CMD23 (SET_BLOCK_COUNT) for pre-defined block count */
    result = EmmcSendCommand(EMMC_CMD23_SET_BLOCK_COUNT, blockCount, &response);
    if (result != 0) {
        return -1;
    }

    /* Send CMD25 (WRITE_MULTIPLE_BLOCK) */
    result = EmmcSendCommand(EMMC_CMD25_WRITE_MULTIPLE_BLOCK, blockAddress, &response);
    if (result != 0) {
        return -1;
    }

    /* Write data blocks */
    for (uint32_t i = 0; i < blockCount; i++) {
        result = EmmcHalWriteData(&data[i * EMMC_BLOCK_SIZE], EMMC_BLOCK_SIZE);
        if (result != 0) {
            EmmcSendCommand(EMMC_CMD12_STOP_TRANSMISSION, 0, NULL);
            return -1;
        }
    }

    /* Wait for programming to complete */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set eMMC bus width
 * @param[in] : busWidth --Bus width to set (1-bit, 4-bit, or 8-bit)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures data bus width for communication
 *****************************************************************************/
int32_t EmmcSetBusWidth(EmmcBusWidth_E busWidth)
{
    int32_t result = 0;
    uint32_t response = 0;
    uint32_t argument = 0;

    if (g_emmcInitialized == 0) {
        return -1;
    }

    /* Prepare CMD6 argument for bus width setting */
    /* Access = 0x03 (write byte), Index = 183 (BUS_WIDTH) */
    switch (busWidth) {
        case EMMC_BUS_WIDTH_1BIT:
            argument = 0x03B70000;
            break;
        case EMMC_BUS_WIDTH_4BIT:
            argument = 0x03B70100;
            break;
        case EMMC_BUS_WIDTH_8BIT:
            argument = 0x03B70200;
            break;
        default:
            return -1;
    }

    /* Send CMD6 (SWITCH) to change bus width */
    result = EmmcSendCommand(EMMC_CMD6_SWITCH, argument, &response);
    if (result != 0) {
        return -1;
    }

    /* Wait for switch to complete */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Configure HAL bus width */
    result = EmmcHalSetBusWidth(busWidth);
    if (result != 0) {
        return -1;
    }

    g_emmcCardInfo.busWidth = busWidth;
    return 0;
}

/******************************************************************************
 * @brief     : Set eMMC speed mode
 * @param[in] : speedMode --Speed mode to set (Legacy, HS, HS200, HS400, DDR52)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures clock speed and timing mode
 *****************************************************************************/
int32_t EmmcSetSpeedMode(EmmcSpeedMode_E speedMode)
{
    int32_t result = 0;
    uint32_t response = 0;
    uint32_t argument = 0;

    if (g_emmcInitialized == 0) {
        return -1;
    }

    /* Prepare CMD6 argument for high-speed timing setting */
    /* Access = 0x03 (write byte), Index = 185 (HS_TIMING) */
    switch (speedMode) {
        case EMMC_SPEED_MODE_LEGACY:
            argument = 0x03B90000;
            break;
        case EMMC_SPEED_MODE_HIGH_SPEED:
            argument = 0x03B90100;
            break;
        case EMMC_SPEED_MODE_HS200:
            argument = 0x03B90200;
            break;
        case EMMC_SPEED_MODE_HS400:
            argument = 0x03B90300;
            break;
        case EMMC_SPEED_MODE_DDR52:
            argument = 0x03B90100;
            break;
        default:
            return -1;
    }

    /* Send CMD6 (SWITCH) to change speed mode */
    result = EmmcSendCommand(EMMC_CMD6_SWITCH, argument, &response);
    if (result != 0) {
        return -1;
    }

    /* Wait for switch to complete */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Configure HAL clock speed */
    result = EmmcHalSetClockSpeed(speedMode);
    if (result != 0) {
        return -1;
    }

    g_emmcCardInfo.speedMode = speedMode;
    return 0;
}

/******************************************************************************
 * @brief     : Get eMMC card information
 * @param[in] : None
 * @param[out]: cardInfo --Pointer to structure to store card information
 * @return    : 0 if success, -1 if error
 * @note      : Reads CID, CSD, EXT_CSD and calculates capacity
 *****************************************************************************/
int32_t EmmcGetCardInfo(EmmcCardInfo_S* cardInfo)
{
    if (g_emmcInitialized == 0 || cardInfo == NULL) {
        return -1;
    }

    if (memcpy_s(cardInfo, sizeof(EmmcCardInfo_S), &g_emmcCardInfo, sizeof(EmmcCardInfo_S)) != EOK) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Select eMMC partition
 * @param[in] : partition --Partition to select (USER, BOOT1, BOOT2, RPMB)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Switches active partition for read/write operations
 *****************************************************************************/
int32_t EmmcSelectPartition(EmmcPartition_E partition)
{
    int32_t result = 0;
    uint32_t response = 0;
    uint32_t argument = 0;

    if (g_emmcInitialized == 0) {
        return -1;
    }

    /* Prepare CMD6 argument for partition configuration */
    /* Access = 0x03 (write byte), Index = 179 (PARTITION_CONFIG) */
    uint8_t partitionAccess = 0;
    switch (partition) {
        case EMMC_PARTITION_USER:
            partitionAccess = 0x00;
            break;
        case EMMC_PARTITION_BOOT1:
            partitionAccess = 0x01;
            break;
        case EMMC_PARTITION_BOOT2:
            partitionAccess = 0x02;
            break;
        case EMMC_PARTITION_RPMB:
            partitionAccess = 0x03;
            break;
        default:
            return -1;
    }

    argument = 0x03B30000 | (partitionAccess << 8);

    /* Send CMD6 (SWITCH) to change partition */
    result = EmmcSendCommand(EMMC_CMD6_SWITCH, argument, &response);
    if (result != 0) {
        return -1;
    }

    /* Wait for switch to complete */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    g_emmcCardInfo.currentPartition = partition;
    return 0;
}

/******************************************************************************
 * @brief     : Get eMMC card status
 * @param[in] : None
 * @param[out]: status --Pointer to store card status register value
 * @return    : 0 if success, -1 if error
 * @note      : Reads current card status flags
 *****************************************************************************/
int32_t EmmcGetCardStatus(uint32_t* status)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized == 0 || status == NULL) {
        return -1;
    }

    /* Send CMD13 (SEND_STATUS) */
    result = EmmcSendCommand(EMMC_CMD13_SEND_STATUS, g_emmcCardInfo.rca << 16, &response);
    if (result != 0) {
        return -1;
    }

    *status = response;
    return 0;
}

/******************************************************************************
 * @brief     : Erase blocks on eMMC
 * @param[in] : startBlock --Start block address, endBlock - End block address
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Erases blocks from startBlock to endBlock (inclusive)
 *****************************************************************************/
int32_t EmmcEraseBlocks(uint32_t startBlock, uint32_t endBlock)
{
    int32_t result = 0;
    uint32_t response = 0;

    if (g_emmcInitialized == 0 || startBlock > endBlock) {
        return -1;
    }

    /* Wait for card to be ready */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    /* Send CMD35 (ERASE_GROUP_START) */
    result = EmmcSendCommand(EMMC_CMD35_ERASE_GROUP_START, startBlock, &response);
    if (result != 0) {
        return -1;
    }

    /* Send CMD36 (ERASE_GROUP_END) */
    result = EmmcSendCommand(EMMC_CMD36_ERASE_GROUP_END, endBlock, &response);
    if (result != 0) {
        return -1;
    }

    /* Send CMD38 (ERASE) */
    result = EmmcSendCommand(EMMC_CMD38_ERASE, 0, &response);
    if (result != 0) {
        return -1;
    }

    /* Wait for erase to complete */
    result = EmmcWaitForReady();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Send eMMC command
 * @param[in] : cmd --Command index, arg - Command argument
 * @param[out]: response --Command response (can be NULL if not needed)
 * @return    : 0 if success, -1 if error
 * @note      : Internal helper function
 *****************************************************************************/
static int32_t EmmcSendCommand(uint8_t cmd, uint32_t arg, uint32_t* response)
{
    return EmmcHalSendCommand(cmd, arg, response);
}

/******************************************************************************
 * @brief     : Wait for eMMC card to be ready
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Internal helper function
 *****************************************************************************/
static int32_t EmmcWaitForReady(void)
{
    uint32_t status = 0;
    int32_t result = 0;

    for (uint32_t i = 0; i < 1000; i++) {
        result = EmmcGetCardStatus(&status);
        if (result != 0) {
            return -1;
        }

        if ((status & EMMC_STATUS_READY_FOR_DATA) != 0) {
            uint32_t currentState = (status & EMMC_STATUS_CURRENT_STATE_MASK) >> 9;
            if (currentState == EMMC_CURRENT_STATE_TRAN) {
                return 0;
            }
        }

        EmmcHalDelay(1);
    }

    return -1;
}

/******************************************************************************
 * @brief     : Identify eMMC card
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Internal helper function
 *****************************************************************************/
static int32_t EmmcIdentifyCard(void)
{
    int32_t result = 0;
    uint32_t cidData[4] = {0};
    uint32_t csdData[4] = {0};
    uint8_t extCsdData[512] = {0};

    /* Send CMD2 (ALL_SEND_CID) */
    result = EmmcHalSendCommand(EMMC_CMD2_ALL_SEND_CID, 0, cidData);
    if (result != 0) {
        return -1;
    }

    /* Parse CID */
    g_emmcCardInfo.cid.manufacturerId = (uint8_t)((cidData[0] >> 24) & 0xFF);
    g_emmcCardInfo.cid.oemApplicationId = (uint16_t)((cidData[0] >> 8) & 0xFFFF);
    g_emmcCardInfo.cid.productSerialNumber = cidData[2];

    /* Send CMD3 (SET_RELATIVE_ADDR) */
    g_emmcCardInfo.rca = 1;
    result = EmmcHalSendCommand(EMMC_CMD3_SET_RELATIVE_ADDR, g_emmcCardInfo.rca << 16, NULL);
    if (result != 0) {
        return -1;
    }

    /* Send CMD9 (SEND_CSD) */
    result = EmmcHalSendCommand(EMMC_CMD9_SEND_CSD, g_emmcCardInfo.rca << 16, csdData);
    if (result != 0) {
        return -1;
    }

    /* Parse CSD for capacity calculation */
    g_emmcCardInfo.csd.cSize = (csdData[1] >> 16) & 0xFFF;

    /* Select card with CMD7 */
    result = EmmcHalSendCommand(EMMC_CMD7_SELECT_CARD, g_emmcCardInfo.rca << 16, NULL);
    if (result != 0) {
        return -1;
    }

    /* Send CMD8 (SEND_EXT_CSD) to get extended information */
    result = EmmcHalSendCommand(EMMC_CMD8_SEND_EXT_CSD, 0, NULL);
    if (result == 0) {
        result = EmmcHalReadData(extCsdData, 512);
        if (result == 0) {
            /* Parse Extended CSD */
            g_emmcCardInfo.extCsd.sectorCount = (uint32_t)extCsdData[212] | ((uint32_t)extCsdData[213] << 8)
                                                | ((uint32_t)extCsdData[214] << 16) | ((uint32_t)extCsdData[215] << 24);

            g_emmcCardInfo.capacity = (uint64_t)g_emmcCardInfo.extCsd.sectorCount * 512;
            g_emmcCardInfo.blockCount = g_emmcCardInfo.extCsd.sectorCount;
        }
    }

    return 0;
}

/******************************************************************************
 * @brief     : Extract bits from byte array
 * @param[in] : data --Byte array, startBit - Start bit position, bitCount - Number of bits
 * @param[out]: None
 * @return    : Extracted value
 * @note      : Internal helper function
 *****************************************************************************/
static uint32_t EmmcExtractBits(const uint8_t* data, uint32_t startBit, uint32_t bitCount)
{
    uint32_t result = 0;
    uint32_t byteIndex = 0;
    uint32_t bitIndex = 0;

    for (uint32_t i = 0; i < bitCount; i++) {
        byteIndex = (startBit + i) / 8;
        bitIndex = (startBit + i) % 8;
        if ((data[byteIndex] & (1 << bitIndex)) != 0) {
            result |= (1U << i);
        }
    }

    return result;
}

#include "emmc_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific includes and definitions */
#ifdef PLATFORM_STM32F4
#include <string.h>
static SD_HandleTypeDef g_emmcHandle;
#endif

#ifdef PLATFORM_STM32F1
#include <string.h>
static SD_HandleTypeDef g_emmcHandle;
#endif

#ifdef PLATFORM_ESP32
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_host.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

static sdmmc_card_t* g_emmcCard = NULL;
static sdmmc_host_t g_emmcHost = SDMMC_HOST_DEFAULT();
#endif

#ifdef PLATFORM_LINUX
#include <errno.h>
#include <linux/mmc/ioctl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

static int g_emmcFd = -1;
static const char* g_emmcDevicePath = "/dev/mmcblk0";
#endif

/******************************************************************************
 * @brief     : Initialize eMMC hardware abstraction layer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalInit(void)
{
#ifdef PLATFORM_STM32F4
    /* Enable SDMMC and GPIO clocks */
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure GPIO pins for SDMMC1 in 8-bit mode */
    GPIO_InitTypeDef gpioInit = {0};

    /* PC8-PC11: D0-D3, PC12: CLK */
    gpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    /* PD2: CMD */
    gpioInit.Pin = GPIO_PIN_2;
    gpioInit.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOD, &gpioInit);

    /* Configure SDMMC1 for eMMC mode */
    g_emmcHandle.Instance = SDMMC1;
    g_emmcHandle.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    g_emmcHandle.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
    g_emmcHandle.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    g_emmcHandle.Init.BusWide = SDMMC_BUS_WIDE_1B;
    g_emmcHandle.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    g_emmcHandle.Init.ClockDiv = 118; /* ~400 kHz for initialization */

    if (HAL_SD_Init(&g_emmcHandle) != HAL_OK) {
        return -1;
    }

    return 0;

#elif defined(PLATFORM_STM32F1)
    /* STM32F1 has SDIO (not SDMMC), limited to 4-bit mode */
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure GPIO pins for SDIO in 4-bit mode */
    GPIO_InitTypeDef gpioInit = {0};

    /* PC8-PC11: D0-D3, PC12: CLK */
    gpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpioInit);

    /* PD2: CMD */
    gpioInit.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &gpioInit);

    /* Configure SDIO for eMMC mode */
    g_emmcHandle.Instance = SDIO;
    g_emmcHandle.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    g_emmcHandle.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    g_emmcHandle.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    g_emmcHandle.Init.BusWide = SDIO_BUS_WIDE_1B;
    g_emmcHandle.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    g_emmcHandle.Init.ClockDiv = 118; /* ~400 kHz for initialization */

    if (HAL_SD_Init(&g_emmcHandle) != HAL_OK) {
        return -1;
    }

    return 0;

#elif defined(PLATFORM_ESP32)
    /* Configure SDMMC host for eMMC */
    g_emmcHost.flags = SDMMC_HOST_FLAG_8BIT | SDMMC_HOST_FLAG_DDR;
    g_emmcHost.slot = SDMMC_HOST_SLOT_1;
    g_emmcHost.max_freq_khz = SDMMC_FREQ_DEFAULT;

    /* Initialize SDMMC slot */
    sdmmc_slot_config_t slotConfig = SDMMC_SLOT_CONFIG_DEFAULT();
    slotConfig.width = 8; /* 8-bit bus width */
    slotConfig.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = sdmmc_host_init();
    if (ret != ESP_OK) {
        return -1;
    }

    ret = sdmmc_host_init_slot(g_emmcHost.slot, &slotConfig);
    if (ret != ESP_OK) {
        return -1;
    }

    /* Allocate card structure */
    g_emmcCard = (sdmmc_card_t*)malloc(sizeof(sdmmc_card_t));
    if (g_emmcCard == NULL) {
        return -1;
    }
    memset(g_emmcCard, 0, sizeof(sdmmc_card_t));

    return 0;

#elif defined(PLATFORM_LINUX)
    /* Open eMMC block device */
    g_emmcFd = open(g_emmcDevicePath, O_RDWR);
    if (g_emmcFd < 0) {
        return -1;
    }

    return 0;

#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Deinitialize eMMC hardware abstraction layer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalDeinit(void)
{
#ifdef PLATFORM_STM32F4
    HAL_SD_DeInit(&g_emmcHandle);
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    return 0;

#elif defined(PLATFORM_STM32F1)
    HAL_SD_DeInit(&g_emmcHandle);
    __HAL_RCC_SDIO_CLK_DISABLE();
    return 0;

#elif defined(PLATFORM_ESP32)
    if (g_emmcCard != NULL) {
        free(g_emmcCard);
        g_emmcCard = NULL;
    }
    sdmmc_host_deinit();
    return 0;

#elif defined(PLATFORM_LINUX)
    if (g_emmcFd >= 0) {
        close(g_emmcFd);
        g_emmcFd = -1;
    }
    return 0;

#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Send command to eMMC device
 * @param[in] : cmd - Command index, arg - Command argument
 * @param[out]: response - Command response (can be NULL if not needed)
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalSendCommand(uint8_t cmd, uint32_t arg, uint32_t* response)
{
#ifdef PLATFORM_STM32F4
    SDMMC_CmdInitTypeDef cmdInit = {0};

    cmdInit.Argument = arg;
    cmdInit.CmdIndex = cmd;

    /* Determine response type based on command */
    if (cmd == EMMC_CMD0_GO_IDLE_STATE) {
        cmdInit.Response = SDMMC_RESPONSE_NO;
    } else if (cmd == EMMC_CMD2_ALL_SEND_CID || cmd == EMMC_CMD9_SEND_CSD || cmd == EMMC_CMD10_SEND_CID) {
        cmdInit.Response = SDMMC_RESPONSE_LONG;
    } else {
        cmdInit.Response = SDMMC_RESPONSE_SHORT;
    }

    cmdInit.WaitForInterrupt = SDMMC_WAIT_NO;
    cmdInit.CPSM = SDMMC_CPSM_ENABLE;

    SDMMC_SendCommand(g_emmcHandle.Instance, &cmdInit);

    /* Wait for response */
    uint32_t timeout = 1000;
    while (timeout > 0) {
        uint32_t sta = g_emmcHandle.Instance->STA;
        if ((sta & (SDMMC_STA_CMDREND | SDMMC_STA_CMDSENT | SDMMC_STA_CTIMEOUT | SDMMC_STA_CCRCFAIL)) != 0) {
            break;
        }
        timeout--;
        HAL_Delay(1);
    }

    if (timeout == 0) {
        return -1;
    }

    /* Check for errors */
    if ((g_emmcHandle.Instance->STA & (SDMMC_STA_CTIMEOUT | SDMMC_STA_CCRCFAIL)) != 0) {
        SDMMC_ClearFlag(g_emmcHandle.Instance, SDMMC_STATIC_FLAGS);
        return -1;
    }

    /* Get response if requested */
    if (response != NULL) {
        if (cmdInit.Response == SDMMC_RESPONSE_LONG) {
            response[0] = g_emmcHandle.Instance->RESP1;
            response[1] = g_emmcHandle.Instance->RESP2;
            response[2] = g_emmcHandle.Instance->RESP3;
            response[3] = g_emmcHandle.Instance->RESP4;
        } else {
            response[0] = g_emmcHandle.Instance->RESP1;
        }
    }

    SDMMC_ClearFlag(g_emmcHandle.Instance, SDMMC_STATIC_FLAGS);
    return 0;

#elif defined(PLATFORM_STM32F1)
    /* Similar to STM32F4 but using SDIO peripheral */
    SDIO_CmdInitTypeDef cmdInit = {0};

    cmdInit.Argument = arg;
    cmdInit.CmdIndex = cmd;

    if (cmd == EMMC_CMD0_GO_IDLE_STATE) {
        cmdInit.Response = SDIO_RESPONSE_NO;
    } else if (cmd == EMMC_CMD2_ALL_SEND_CID || cmd == EMMC_CMD9_SEND_CSD || cmd == EMMC_CMD10_SEND_CID) {
        cmdInit.Response = SDIO_RESPONSE_LONG;
    } else {
        cmdInit.Response = SDIO_RESPONSE_SHORT;
    }

    cmdInit.WaitForInterrupt = SDIO_WAIT_NO;
    cmdInit.CPSM = SDIO_CPSM_ENABLE;

    SDIO_SendCommand(SDIO, &cmdInit);

    /* Wait for response */
    uint32_t timeout = 1000;
    while (timeout > 0) {
        uint32_t sta = SDIO->STA;
        if ((sta & (SDIO_STA_CMDREND | SDIO_STA_CMDSENT | SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL)) != 0) {
            break;
        }
        timeout--;
        HAL_Delay(1);
    }

    if (timeout == 0) {
        return -1;
    }

    /* Check for errors */
    if ((SDIO->STA & (SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL)) != 0) {
        SDIO_ClearFlag(SDIO, SDIO_STATIC_FLAGS);
        return -1;
    }

    /* Get response if requested */
    if (response != NULL) {
        if (cmdInit.Response == SDIO_RESPONSE_LONG) {
            response[0] = SDIO->RESP1;
            response[1] = SDIO->RESP2;
            response[2] = SDIO->RESP3;
            response[3] = SDIO->RESP4;
        } else {
            response[0] = SDIO->RESP1;
        }
    }

    SDIO_ClearFlag(SDIO, SDIO_STATIC_FLAGS);
    return 0;

#elif defined(PLATFORM_ESP32)
    if (g_emmcCard == NULL) {
        return -1;
    }

    sdmmc_command_t sdmmcCmd = {.opcode = cmd, .arg = arg, .flags = 0, .data = NULL, .datalen = 0, .blklen = 0, .timeout_ms = 1000};

    /* Set response flags based on command */
    if (cmd == EMMC_CMD0_GO_IDLE_STATE) {
        sdmmcCmd.flags = SCF_RSP_NONE;
    } else if (cmd == EMMC_CMD2_ALL_SEND_CID || cmd == EMMC_CMD9_SEND_CSD || cmd == EMMC_CMD10_SEND_CID) {
        sdmmcCmd.flags = SCF_RSP_R2;
    } else {
        sdmmcCmd.flags = SCF_RSP_R1;
    }

    esp_err_t ret = sdmmc_send_cmd(g_emmcCard, &sdmmcCmd);
    if (ret != ESP_OK) {
        return -1;
    }

    /* Copy response if requested */
    if (response != NULL) {
        response[0] = sdmmcCmd.response[0];
        if (sdmmcCmd.flags & SCF_RSP_R2) {
            response[1] = sdmmcCmd.response[1];
            response[2] = sdmmcCmd.response[2];
            response[3] = sdmmcCmd.response[3];
        }
    }

    return 0;

#elif defined(PLATFORM_LINUX)
    struct mmc_ioc_cmd iocCmd;
    (void)memset_s(&iocCmd, sizeof(iocCmd), 0, sizeof(iocCmd));

    iocCmd.opcode = cmd;
    iocCmd.arg = arg;
    iocCmd.flags = MMC_RSP_R1;
    iocCmd.blksz = 512;
    iocCmd.blocks = 0;

    /* Set response flags based on command */
    if (cmd == EMMC_CMD0_GO_IDLE_STATE) {
        iocCmd.flags = 0;
    } else if (cmd == EMMC_CMD2_ALL_SEND_CID || cmd == EMMC_CMD9_SEND_CSD || cmd == EMMC_CMD10_SEND_CID) {
        iocCmd.flags = MMC_RSP_R2;
    }

    if (ioctl(g_emmcFd, MMC_IOC_CMD, &iocCmd) < 0) {
        return -1;
    }

    if (response != NULL) {
        response[0] = iocCmd.response[0];
        if (iocCmd.flags == MMC_RSP_R2) {
            response[1] = iocCmd.response[1];
            response[2] = iocCmd.response[2];
            response[3] = iocCmd.response[3];
        }
    }

    return 0;

#else
    (void)cmd;
    (void)arg;
    (void)response;
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read data from eMMC device
 * @param[in] : length - Number of bytes to read
 * @param[out]: data - Buffer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalReadData(uint8_t* data, uint32_t length)
{
#ifdef PLATFORM_STM32F4
    if (data == NULL || length == 0) {
        return -1;
    }

    /* Configure data path */
    SDMMC_DataInitTypeDef dataConfig = {0};
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = length;
    dataConfig.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
    dataConfig.TransferDir = SDMMC_TRANSFER_DIR_TO_SDMMC;
    dataConfig.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
    dataConfig.DPSM = SDMMC_DPSM_ENABLE;

    SDMMC_ConfigData(g_emmcHandle.Instance, &dataConfig);

    /* Read data */
    uint32_t* dataPtr = (uint32_t*)data;
    uint32_t count = length / 4;

    while (count > 0) {
        if ((g_emmcHandle.Instance->STA & SDMMC_STA_RXFIFOE) == 0) {
            *dataPtr = SDMMC_ReadFIFO(g_emmcHandle.Instance);
            dataPtr++;
            count--;
        }
    }

    /* Wait for transfer complete */
    uint32_t timeout = 1000;
    while (timeout > 0) {
        if ((g_emmcHandle.Instance->STA & SDMMC_STA_DATAEND) != 0) {
            break;
        }
        timeout--;
        HAL_Delay(1);
    }

    SDMMC_ClearFlag(g_emmcHandle.Instance, SDMMC_STATIC_DATA_FLAGS);
    return (timeout == 0) ? -1 : 0;

#elif defined(PLATFORM_STM32F1)
    if (data == NULL || length == 0) {
        return -1;
    }

    /* Configure data path */
    SDIO_DataInitTypeDef dataConfig = {0};
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = length;
    dataConfig.DataBlockSize = SDIO_DATABLOCK_SIZE_512B;
    dataConfig.TransferDir = SDIO_TRANSFER_DIR_TO_SDIO;
    dataConfig.TransferMode = SDIO_TRANSFER_MODE_BLOCK;
    dataConfig.DPSM = SDIO_DPSM_ENABLE;

    SDIO_ConfigData(SDIO, &dataConfig);

    /* Read data */
    uint32_t* dataPtr = (uint32_t*)data;
    uint32_t count = length / 4;

    while (count > 0) {
        if ((SDIO->STA & SDIO_STA_RXFIFOE) == 0) {
            *dataPtr = SDIO_ReadFIFO(SDIO);
            dataPtr++;
            count--;
        }
    }

    /* Wait for transfer complete */
    uint32_t timeout = 1000;
    while (timeout > 0) {
        if ((SDIO->STA & SDIO_STA_DATAEND) != 0) {
            break;
        }
        timeout--;
        HAL_Delay(1);
    }

    SDIO_ClearFlag(SDIO, SDIO_STATIC_DATA_FLAGS);
    return (timeout == 0) ? -1 : 0;

#elif defined(PLATFORM_ESP32)
    if (data == NULL || length == 0 || g_emmcCard == NULL) {
        return -1;
    }

    /* Data transfer is handled by command-specific functions in ESP32 */
    return 0;

#elif defined(PLATFORM_LINUX)
    if (data == NULL || length == 0 || g_emmcFd < 0) {
        return -1;
    }

    ssize_t bytesRead = read(g_emmcFd, data, length);
    return (bytesRead == (ssize_t)length) ? 0 : -1;

#else
    (void)data;
    (void)length;
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Write data to eMMC device
 * @param[in] : data - Data to write, length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalWriteData(const uint8_t* data, uint32_t length)
{
#ifdef PLATFORM_STM32F4
    if (data == NULL || length == 0) {
        return -1;
    }

    /* Configure data path */
    SDMMC_DataInitTypeDef dataConfig = {0};
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = length;
    dataConfig.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
    dataConfig.TransferDir = SDMMC_TRANSFER_DIR_TO_CARD;
    dataConfig.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
    dataConfig.DPSM = SDMMC_DPSM_ENABLE;

    SDMMC_ConfigData(g_emmcHandle.Instance, &dataConfig);

    /* Write data */
    const uint32_t* dataPtr = (const uint32_t*)data;
    uint32_t count = length / 4;

    while (count > 0) {
        if ((g_emmcHandle.Instance->STA & SDMMC_STA_TXFIFOF) == 0) {
            SDMMC_WriteFIFO(g_emmcHandle.Instance, *dataPtr);
            dataPtr++;
            count--;
        }
    }

    /* Wait for transfer complete */
    uint32_t timeout = 1000;
    while (timeout > 0) {
        if ((g_emmcHandle.Instance->STA & SDMMC_STA_DATAEND) != 0) {
            break;
        }
        timeout--;
        HAL_Delay(1);
    }

    SDMMC_ClearFlag(g_emmcHandle.Instance, SDMMC_STATIC_DATA_FLAGS);
    return (timeout == 0) ? -1 : 0;

#elif defined(PLATFORM_STM32F1)
    if (data == NULL || length == 0) {
        return -1;
    }

    /* Configure data path */
    SDIO_DataInitTypeDef dataConfig = {0};
    dataConfig.DataTimeOut = 0xFFFFFFFF;
    dataConfig.DataLength = length;
    dataConfig.DataBlockSize = SDIO_DATABLOCK_SIZE_512B;
    dataConfig.TransferDir = SDIO_TRANSFER_DIR_TO_CARD;
    dataConfig.TransferMode = SDIO_TRANSFER_MODE_BLOCK;
    dataConfig.DPSM = SDIO_DPSM_ENABLE;

    SDIO_ConfigData(SDIO, &dataConfig);

    /* Write data */
    const uint32_t* dataPtr = (const uint32_t*)data;
    uint32_t count = length / 4;

    while (count > 0) {
        if ((SDIO->STA & SDIO_STA_TXFIFOF) == 0) {
            SDIO_WriteFIFO(SDIO, *dataPtr);
            dataPtr++;
            count--;
        }
    }

    /* Wait for transfer complete */
    uint32_t timeout = 1000;
    while (timeout > 0) {
        if ((SDIO->STA & SDIO_STA_DATAEND) != 0) {
            break;
        }
        timeout--;
        HAL_Delay(1);
    }

    SDIO_ClearFlag(SDIO, SDIO_STATIC_DATA_FLAGS);
    return (timeout == 0) ? -1 : 0;

#elif defined(PLATFORM_ESP32)
    if (data == NULL || length == 0 || g_emmcCard == NULL) {
        return -1;
    }

    /* Data transfer is handled by command-specific functions in ESP32 */
    return 0;

#elif defined(PLATFORM_LINUX)
    if (data == NULL || length == 0 || g_emmcFd < 0) {
        return -1;
    }

    ssize_t bytesWritten = write(g_emmcFd, data, length);
    return (bytesWritten == (ssize_t)length) ? 0 : -1;

#else
    (void)data;
    (void)length;
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Set eMMC bus width at hardware level
 * @param[in] : busWidth - Bus width to set (1-bit, 4-bit, or 8-bit)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalSetBusWidth(EmmcBusWidth_E busWidth)
{
#ifdef PLATFORM_STM32F4
    uint32_t sdmmcBusWidth;

    switch (busWidth) {
        case EMMC_BUS_WIDTH_1BIT:
            sdmmcBusWidth = SDMMC_BUS_WIDE_1B;
            break;
        case EMMC_BUS_WIDTH_4BIT:
            sdmmcBusWidth = SDMMC_BUS_WIDE_4B;
            break;
        case EMMC_BUS_WIDTH_8BIT:
            sdmmcBusWidth = SDMMC_BUS_WIDE_8B;
            break;
        default:
            return -1;
    }

    MODIFY_REG(g_emmcHandle.Instance->CLKCR, SDMMC_CLKCR_WIDBUS, sdmmcBusWidth);
    return 0;

#elif defined(PLATFORM_STM32F1)
    /* STM32F1 SDIO only supports up to 4-bit mode */
    uint32_t sdioBusWidth;

    switch (busWidth) {
        case EMMC_BUS_WIDTH_1BIT:
            sdioBusWidth = SDIO_BUS_WIDE_1B;
            break;
        case EMMC_BUS_WIDTH_4BIT:
            sdioBusWidth = SDIO_BUS_WIDE_4B;
            break;
        case EMMC_BUS_WIDTH_8BIT:
            return -1; /* 8-bit not supported on STM32F1 */
        default:
            return -1;
    }

    MODIFY_REG(SDIO->CLKCR, SDIO_CLKCR_WIDBUS, sdioBusWidth);
    return 0;

#elif defined(PLATFORM_ESP32)
    if (g_emmcCard == NULL) {
        return -1;
    }

    uint8_t width;
    switch (busWidth) {
        case EMMC_BUS_WIDTH_1BIT:
            width = 1;
            break;
        case EMMC_BUS_WIDTH_4BIT:
            width = 4;
            break;
        case EMMC_BUS_WIDTH_8BIT:
            width = 8;
            break;
        default:
            return -1;
    }

    esp_err_t ret = sdmmc_host_set_bus_width(g_emmcHost.slot, width);
    return (ret == ESP_OK) ? 0 : -1;

#elif defined(PLATFORM_LINUX)
    /* Linux handles bus width configuration automatically */
    (void)busWidth;
    return 0;

#else
    (void)busWidth;
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Set eMMC clock speed at hardware level
 * @param[in] : speedMode - Speed mode to set
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalSetClockSpeed(EmmcSpeedMode_E speedMode)
{
#ifdef PLATFORM_STM32F4
    uint32_t clockDiv;

    switch (speedMode) {
        case EMMC_SPEED_MODE_LEGACY:
            clockDiv = 4; /* ~25 MHz */
            break;
        case EMMC_SPEED_MODE_HIGH_SPEED:
            clockDiv = 2; /* ~50 MHz */
            break;
        case EMMC_SPEED_MODE_HS200:
            clockDiv = 0; /* ~200 MHz */
            break;
        case EMMC_SPEED_MODE_HS400:
            clockDiv = 0; /* ~200 MHz DDR */
            break;
        case EMMC_SPEED_MODE_DDR52:
            clockDiv = 2; /* ~50 MHz DDR */
            break;
        default:
            return -1;
    }

    MODIFY_REG(g_emmcHandle.Instance->CLKCR, SDMMC_CLKCR_CLKDIV, clockDiv);
    return 0;

#elif defined(PLATFORM_STM32F1)
    uint32_t clockDiv;

    switch (speedMode) {
        case EMMC_SPEED_MODE_LEGACY:
            clockDiv = 4; /* ~25 MHz */
            break;
        case EMMC_SPEED_MODE_HIGH_SPEED:
            clockDiv = 2; /* ~48 MHz max on STM32F1 */
            break;
        default:
            return -1; /* HS200/HS400/DDR52 not supported on STM32F1 */
    }

    MODIFY_REG(SDIO->CLKCR, SDIO_CLKCR_CLKDIV, clockDiv);
    return 0;

#elif defined(PLATFORM_ESP32)
    if (g_emmcCard == NULL) {
        return -1;
    }

    uint32_t freqKhz;
    switch (speedMode) {
        case EMMC_SPEED_MODE_LEGACY:
            freqKhz = 25000;
            break;
        case EMMC_SPEED_MODE_HIGH_SPEED:
            freqKhz = 52000;
            break;
        case EMMC_SPEED_MODE_HS200:
            freqKhz = 200000;
            break;
        case EMMC_SPEED_MODE_HS400:
            freqKhz = 200000;
            break;
        case EMMC_SPEED_MODE_DDR52:
            freqKhz = 52000;
            break;
        default:
            return -1;
    }

    esp_err_t ret = sdmmc_host_set_card_clk(g_emmcHost.slot, freqKhz);
    return (ret == ESP_OK) ? 0 : -1;

#elif defined(PLATFORM_LINUX)
    /* Linux handles clock speed configuration automatically */
    (void)speedMode;
    return 0;

#else
    (void)speedMode;
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Delay for specified milliseconds
 * @param[in] : ms - Milliseconds to delay
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void EmmcHalDelay(uint32_t ms)
{
#ifdef PLATFORM_STM32F4
    HAL_Delay(ms);
#elif defined(PLATFORM_STM32F1)
    HAL_Delay(ms);
#elif defined(PLATFORM_ESP32)
    vTaskDelay(ms / portTICK_PERIOD_MS);
#elif defined(PLATFORM_LINUX)
    usleep(ms * 1000);
#else
    (void)ms;
#endif
}

/******************************************************************************
 * @brief     : Enable eMMC clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_SDIO_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* Clock enabled in init */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure eMMC GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    /* GPIO configured in init */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* GPIO configured in init */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* GPIO configured in init */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Wait for transfer complete
 * @param[in] : timeoutMs - Timeout in milliseconds
 * @param[out]: None
 * @return    : 0 if success, -1 if error or timeout
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t EmmcHalWaitTransferComplete(uint32_t timeoutMs)
{
#ifdef PLATFORM_STM32F4
    uint32_t tickStart = HAL_GetTick();
    while ((HAL_GetTick() - tickStart) < timeoutMs) {
        if ((g_emmcHandle.Instance->STA & SDMMC_STA_DATAEND) != 0) {
            SDMMC_ClearFlag(g_emmcHandle.Instance, SDMMC_STATIC_DATA_FLAGS);
            return 0;
        }
    }
    return -1;

#elif defined(PLATFORM_STM32F1)
    uint32_t tickStart = HAL_GetTick();
    while ((HAL_GetTick() - tickStart) < timeoutMs) {
        if ((SDIO->STA & SDIO_STA_DATAEND) != 0) {
            SDIO_ClearFlag(SDIO, SDIO_STATIC_DATA_FLAGS);
            return 0;
        }
    }
    return -1;

#elif defined(PLATFORM_ESP32)
    /* Handled by driver */
    (void)timeoutMs;
    return 0;

#elif defined(PLATFORM_LINUX)
    /* Handled by kernel driver */
    (void)timeoutMs;
    return 0;

#else
    (void)timeoutMs;
    return -1;
#endif
}

#include "securec.h"
#include "canfd_hal.h"
#include "platform_config.h"

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static FDCAN_HandleTypeDef g_canFdHandle;
static FDCAN_TxHeaderTypeDef g_txHeader;
static FDCAN_RxHeaderTypeDef g_rxHeader;
#endif

#ifdef PLATFORM_STM32F1
/* STM32F1 does not support CAN FD - only classic CAN */
#endif

#ifdef PLATFORM_ESP32
#include "driver/twai.h"
static twai_general_config_t g_generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
static twai_timing_config_t g_timingConfig = TWAI_TIMING_CONFIG_500KBITS();
static twai_filter_config_t g_filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
/* Note: ESP32 TWAI controller supports CAN FD on newer chips (ESP32-C3/S3) */
#endif

#ifdef PLATFORM_LINUX
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

/* Linux CAN FD structures */
#ifndef CANFD_MTU
#define CANFD_MTU 72
#endif

#ifndef CAN_MAX_DLEN
#define CAN_MAX_DLEN 8
#endif

#ifndef CANFD_MAX_DLEN
#define CANFD_MAX_DLEN 64
#endif

struct canfd_frame {
    uint32_t can_id;
    uint8_t len;
    uint8_t flags;
    uint8_t __res0;
    uint8_t __res1;
    uint8_t data[CANFD_MAX_DLEN];
};

#define CANFD_BRS 0x01
#define CANFD_ESI 0x02

static int g_canFdFd = -1;
#endif

/******************************************************************************
 * Helper function to convert DLC to actual data length for CAN FD
 *****************************************************************************/
static uint8_t DlcToDataLength(uint8_t dlc)
{
    if (dlc <= 8) {
        return dlc;
    }
    switch (dlc) {
        case 9:  return 12;
        case 10: return 16;
        case 11: return 20;
        case 12: return 24;
        case 13: return 32;
        case 14: return 48;
        case 15: return 64;
        default: return 0;
    }
}

/******************************************************************************
 * Helper function to convert data length to DLC for CAN FD
 *****************************************************************************/
static uint8_t DataLengthToDlc(uint8_t dataLength)
{
    if (dataLength <= 8) {
        return dataLength;
    }
    if (dataLength <= 12) {
        return 9;
    }
    if (dataLength <= 16) {
        return 10;
    }
    if (dataLength <= 20) {
        return 11;
    }
    if (dataLength <= 24) {
        return 12;
    }
    if (dataLength <= 32) {
        return 13;
    }
    if (dataLength <= 48) {
        return 14;
    }
    if (dataLength <= 64) {
        return 15;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Enable CAN FD clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 CAN FD clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable CAN FD clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdDisableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_FDCAN_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 CAN FD clock is managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock disabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure CAN FD GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* FDCAN1 GPIO Configuration: PA11(RX), PA12(TX) */
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by twai_driver_install */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses network interface, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable CAN FD module
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdEnable(void)
{
#ifdef PLATFORM_STM32F4
    if (HAL_FDCAN_Start(&g_canFdHandle) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    if (twai_driver_install(&g_generalConfig, &g_timingConfig, &g_filterConfig) != ESP_OK) {
        return -1;
    }
    if (twai_start() != ESP_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_canFdFd < 0) {
        struct sockaddr_can addr;
        struct ifreq ifr;
        int enable_canfd = 1;

        g_canFdFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (g_canFdFd < 0) {
            return -1;
        }

        /* Enable CAN FD support */
        if (setsockopt(g_canFdFd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
            close(g_canFdFd);
            g_canFdFd = -1;
            return -1;
        }

        if (strcpy_s(ifr.ifr_name, sizeof(ifr.ifr_name), "can0") != EOK) {
            close(g_canFdFd);
            g_canFdFd = -1;
            return -1;
        }
        if (ioctl(g_canFdFd, SIOCGIFINDEX, &ifr) < 0) {
            close(g_canFdFd);
            g_canFdFd = -1;
            return -1;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(g_canFdFd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(g_canFdFd);
            g_canFdFd = -1;
            return -1;
        }
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable CAN FD module
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdDisable(void)
{
#ifdef PLATFORM_STM32F4
    if (HAL_FDCAN_Stop(&g_canFdHandle) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    if (twai_stop() != ESP_OK) {
        return -1;
    }
    if (twai_driver_uninstall() != ESP_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_canFdFd >= 0) {
        close(g_canFdFd);
        g_canFdFd = -1;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if CAN FD TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdTxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_FDCAN_GetTxFifoFreeLevel(&g_canFdHandle) > 0) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return 0;
#elif defined(PLATFORM_ESP32)
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        return (status.msgs_to_tx == 0) ? 1 : 0;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    return 1;  /* Linux driver handles buffering */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write CAN FD frame
 * @param[in] : frame - Pointer to CAN FD frame structure to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdWriteFrame(const CanFdFrame* frame)
{
#ifdef PLATFORM_STM32F4
    if (frame == NULL) {
        return -1;
    }

    g_txHeader.Identifier = frame->id;
    g_txHeader.IdType = (frame->idType == CANFD_ID_TYPE_EXTENDED) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    g_txHeader.TxFrameType = FDCAN_DATA_FRAME;
    g_txHeader.DataLength = DataLengthToDlc(frame->dataLength) << 16;
    g_txHeader.ErrorStateIndicator = (frame->esiFlag == CANFD_ESI_ERROR_PASSIVE) ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
    g_txHeader.BitRateSwitch = (frame->brsFlag == CANFD_BRS_ENABLED) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    g_txHeader.FDFormat = (frame->frameType == CANFD_FRAME_TYPE_FD) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    g_txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_txHeader.MessageMarker = 0;

    return (HAL_FDCAN_AddMessageToTxFifoQ(&g_canFdHandle, &g_txHeader, (uint8_t*)frame->data) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    if (frame == NULL || frame->dataLength > 8) {
        return -1;  /* ESP32 TWAI limited to 8 bytes */
    }

    twai_message_t message;
    message.identifier = frame->id;
    message.data_length_code = frame->dataLength;
    message.flags = (frame->idType == CANFD_ID_TYPE_EXTENDED) ? TWAI_MSG_FLAG_EXTD : TWAI_MSG_FLAG_NONE;

    for (int i = 0; i < frame->dataLength; i++) {
        message.data[i] = frame->data[i];
    }

    return (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_canFdFd < 0 || frame == NULL) {
        return -1;
    }

    struct canfd_frame canfdFrame;
    if (memset_s(&canfdFrame, sizeof(canfdFrame), 0, sizeof(canfdFrame)) != EOK) {
        return -1;
    }

    canfdFrame.can_id = frame->id;
    if (frame->idType == CANFD_ID_TYPE_EXTENDED) {
        canfdFrame.can_id |= CAN_EFF_FLAG;
    }

    canfdFrame.len = frame->dataLength;
    canfdFrame.flags = 0;

    if (frame->frameType == CANFD_FRAME_TYPE_FD) {
        if (frame->brsFlag == CANFD_BRS_ENABLED) {
            canfdFrame.flags |= CANFD_BRS;
        }
        if (frame->esiFlag == CANFD_ESI_ERROR_PASSIVE) {
            canfdFrame.flags |= CANFD_ESI;
        }
    }

    if (memcpy_s(canfdFrame.data, sizeof(canfdFrame.data), frame->data, frame->dataLength) != EOK) {
        return -1;
    }

    int frameSize = (frame->frameType == CANFD_FRAME_TYPE_FD) ? CANFD_MTU : CAN_MTU;
    return (write(g_canFdFd, &canfdFrame, frameSize) == frameSize) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if CAN FD RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdRxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_FDCAN_GetRxFifoFillLevel(&g_canFdHandle, FDCAN_RX_FIFO0) > 0) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return 0;
#elif defined(PLATFORM_ESP32)
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        return (status.msgs_to_rx > 0) ? 1 : 0;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_canFdFd >= 0) {
        int bytes = 0;
        if (ioctl(g_canFdFd, FIONREAD, &bytes) >= 0) {
            return (bytes > 0) ? 1 : 0;
        }
    }
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Read CAN FD frame
 * @param[in] : None
 * @param[out]: frame - Pointer to CAN FD frame structure to store received data
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdReadFrame(CanFdFrame* frame)
{
#ifdef PLATFORM_STM32F4
    if (frame == NULL) {
        return -1;
    }

    if (HAL_FDCAN_GetRxMessage(&g_canFdHandle, FDCAN_RX_FIFO0, &g_rxHeader, frame->data) != HAL_OK) {
        return -1;
    }

    frame->id = g_rxHeader.Identifier;
    frame->idType = (g_rxHeader.IdType == FDCAN_EXTENDED_ID) ? CANFD_ID_TYPE_EXTENDED : CANFD_ID_TYPE_STANDARD;
    frame->dataLength = DlcToDataLength((uint8_t)(g_rxHeader.DataLength >> 16));
    frame->frameType = (g_rxHeader.FDFormat == FDCAN_FD_CAN) ? CANFD_FRAME_TYPE_FD : CANFD_FRAME_TYPE_CLASSIC;
    frame->brsFlag = (g_rxHeader.BitRateSwitch == FDCAN_BRS_ON) ? CANFD_BRS_ENABLED : CANFD_BRS_DISABLED;
    frame->esiFlag = (g_rxHeader.ErrorStateIndicator == FDCAN_ESI_PASSIVE) ? CANFD_ESI_ERROR_PASSIVE : CANFD_ESI_ERROR_ACTIVE;

    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    if (frame == NULL) {
        return -1;
    }

    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(100)) != ESP_OK) {
        return -1;
    }

    frame->id = message.identifier;
    frame->idType = (message.flags & TWAI_MSG_FLAG_EXTD) ? CANFD_ID_TYPE_EXTENDED : CANFD_ID_TYPE_STANDARD;
    frame->dataLength = message.data_length_code;
    frame->frameType = CANFD_FRAME_TYPE_CLASSIC;  /* ESP32 TWAI doesn't support FD */
    frame->brsFlag = CANFD_BRS_DISABLED;
    frame->esiFlag = CANFD_ESI_ERROR_ACTIVE;

    for (int i = 0; i < message.data_length_code && i < CANFD_MAX_DATA_LENGTH; i++) {
        frame->data[i] = message.data[i];
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_canFdFd < 0 || frame == NULL) {
        return -1;
    }

    struct canfd_frame canfdFrame;
    int nbytes = read(g_canFdFd, &canfdFrame, sizeof(canfdFrame));

    if (nbytes < 0) {
        return -1;
    }

    frame->id = canfdFrame.can_id & CAN_EFF_MASK;
    frame->idType = (canfdFrame.can_id & CAN_EFF_FLAG) ? CANFD_ID_TYPE_EXTENDED : CANFD_ID_TYPE_STANDARD;
    frame->dataLength = canfdFrame.len;
    frame->frameType = (nbytes == CANFD_MTU) ? CANFD_FRAME_TYPE_FD : CANFD_FRAME_TYPE_CLASSIC;
    frame->brsFlag = (canfdFrame.flags & CANFD_BRS) ? CANFD_BRS_ENABLED : CANFD_BRS_DISABLED;
    frame->esiFlag = (canfdFrame.flags & CANFD_ESI) ? CANFD_ESI_ERROR_PASSIVE : CANFD_ESI_ERROR_ACTIVE;

    if (memcpy_s(frame->data, sizeof(frame->data), canfdFrame.data, canfdFrame.len) != EOK) {
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure CAN FD bit rates
 * @param[in] : nominalBitRate - Nominal phase bit rate in bps
 * @param[in] : dataBitRate - Data phase bit rate in bps
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdConfigureBitRate(uint32_t nominalBitRate, uint32_t dataBitRate)
{
#ifdef PLATFORM_STM32F4
    g_canFdHandle.Instance = FDCAN1;
    g_canFdHandle.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    g_canFdHandle.Init.Mode = FDCAN_MODE_NORMAL;
    g_canFdHandle.Init.AutoRetransmission = ENABLE;
    g_canFdHandle.Init.TransmitPause = DISABLE;
    g_canFdHandle.Init.ProtocolException = DISABLE;

    /* Nominal bit timing (500 kbps with 80 MHz clock) */
    g_canFdHandle.Init.NominalPrescaler = 10;
    g_canFdHandle.Init.NominalSyncJumpWidth = 1;
    g_canFdHandle.Init.NominalTimeSeg1 = 13;
    g_canFdHandle.Init.NominalTimeSeg2 = 2;

    /* Data bit timing (2 Mbps with 80 MHz clock) */
    g_canFdHandle.Init.DataPrescaler = 2;
    g_canFdHandle.Init.DataSyncJumpWidth = 1;
    g_canFdHandle.Init.DataTimeSeg1 = 13;
    g_canFdHandle.Init.DataTimeSeg2 = 2;

    /* Message RAM configuration */
    g_canFdHandle.Init.StdFiltersNbr = 1;
    g_canFdHandle.Init.ExtFiltersNbr = 0;
    g_canFdHandle.Init.RxFifo0ElmtsNbr = 16;
    g_canFdHandle.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    g_canFdHandle.Init.RxFifo1ElmtsNbr = 0;
    g_canFdHandle.Init.RxBuffersNbr = 0;
    g_canFdHandle.Init.TxEventsNbr = 0;
    g_canFdHandle.Init.TxBuffersNbr = 0;
    g_canFdHandle.Init.TxFifoQueueElmtsNbr = 16;
    g_canFdHandle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    g_canFdHandle.Init.TxElmtSize = FDCAN_DATA_BYTES_64;

    /* Adjust prescalers based on bit rates */
    if (nominalBitRate == 125000) {
        g_canFdHandle.Init.NominalPrescaler = 40;
    } else if (nominalBitRate == 250000) {
        g_canFdHandle.Init.NominalPrescaler = 20;
    } else if (nominalBitRate == 500000) {
        g_canFdHandle.Init.NominalPrescaler = 10;
    } else if (nominalBitRate == 1000000) {
        g_canFdHandle.Init.NominalPrescaler = 5;
    }

    if (dataBitRate == 1000000) {
        g_canFdHandle.Init.DataPrescaler = 4;
    } else if (dataBitRate == 2000000) {
        g_canFdHandle.Init.DataPrescaler = 2;
    } else if (dataBitRate == 4000000) {
        g_canFdHandle.Init.DataPrescaler = 1;
    }

    return (HAL_FDCAN_Init(&g_canFdHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    /* Update timing config based on nominal bit rate */
    if (nominalBitRate == 125000) {
        g_timingConfig = TWAI_TIMING_CONFIG_125KBITS();
    } else if (nominalBitRate == 250000) {
        g_timingConfig = TWAI_TIMING_CONFIG_250KBITS();
    } else if (nominalBitRate == 500000) {
        g_timingConfig = TWAI_TIMING_CONFIG_500KBITS();
    } else if (nominalBitRate == 1000000) {
        g_timingConfig = TWAI_TIMING_CONFIG_1MBITS();
    }
    /* ESP32 TWAI doesn't support separate data bit rate */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux bit rate is configured via ip command */
    /* ip link set can0 type can bitrate <nominal> dbitrate <data> fd on */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure CAN FD acceptance filter
 * @param[in] : filterId - Filter ID value
 * @param[in] : filterMask - Filter mask value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdConfigureFilter(uint32_t filterId, uint32_t filterMask)
{
#ifdef PLATFORM_STM32F4
    FDCAN_FilterTypeDef filterConfig;

    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = filterId;
    filterConfig.FilterID2 = filterMask;

    return (HAL_FDCAN_ConfigFilter(&g_canFdHandle, &filterConfig) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    /* Update filter configuration */
    g_filterConfig.acceptance_code = filterId;
    g_filterConfig.acceptance_mask = filterMask;
    /* Filter needs to be reconfigured on driver restart */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux filter is configured via setsockopt CAN_RAW_FILTER */
    if (g_canFdFd >= 0) {
        struct can_filter rfilter;
        rfilter.can_id = filterId;
        rfilter.can_mask = filterMask;
        return (setsockopt(g_canFdFd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) >= 0) ? 0 : -1;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read CAN FD error counters
 * @param[in] : None
 * @param[out]: errorCount - Pointer to structure to store error counters
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanFdReadErrorCount(CanFdErrorCount* errorCount)
{
#ifdef PLATFORM_STM32F4
    if (errorCount == NULL) {
        return -1;
    }

    FDCAN_ProtocolStatusTypeDef protocolStatus;
    if (HAL_FDCAN_GetProtocolStatus(&g_canFdHandle, &protocolStatus) == HAL_OK) {
        errorCount->txErrorCount = protocolStatus.TDC;
        errorCount->rxErrorCount = protocolStatus.DataLastErrorCode;
        return 0;
    }
    return -1;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
    return -1;
#elif defined(PLATFORM_ESP32)
    if (errorCount == NULL) {
        return -1;
    }

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        errorCount->txErrorCount = status.tx_error_counter;
        errorCount->rxErrorCount = status.rx_error_counter;
        return 0;
    }
    return -1;
#elif defined(PLATFORM_LINUX)
    if (errorCount == NULL) {
        return -1;
    }

    /* Linux doesn't provide easy access to error counters via socket API */
    /* Would need to use netlink or read from sysfs */
    errorCount->txErrorCount = 0;
    errorCount->rxErrorCount = 0;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable CAN FD RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanFdEnableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_FDCAN_ActivateNotification(&g_canFdHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts for notifications */
    twai_reconfigure_alerts(TWAI_ALERT_RX_DATA, NULL);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll mechanisms */
#endif
}

/******************************************************************************
 * @brief     : Enable CAN FD TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanFdEnableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_FDCAN_ActivateNotification(&g_canFdHandle, FDCAN_IT_TX_FIFO_EMPTY, 0);
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts for notifications */
    twai_reconfigure_alerts(TWAI_ALERT_TX_IDLE, NULL);
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use TX interrupts in userspace */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for CAN FD
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanFdConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

/******************************************************************************
 * @brief     : Disable CAN FD RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanFdDisableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_FDCAN_DeactivateNotification(&g_canFdHandle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts */
    twai_reconfigure_alerts(TWAI_ALERT_NONE, NULL);
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable CAN FD TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanFdDisableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_FDCAN_DeactivateNotification(&g_canFdHandle, FDCAN_IT_TX_FIFO_EMPTY);
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts */
    twai_reconfigure_alerts(TWAI_ALERT_NONE, NULL);
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanFdUpdateNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 does not support CAN FD */
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

#include "can_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static CAN_HandleTypeDef g_canHandle;
static CAN_TxHeaderTypeDef g_txHeader;
static CAN_RxHeaderTypeDef g_rxHeader;
static uint32_t g_txMailbox;
#endif

#ifdef PLATFORM_STM32F1
static CAN_HandleTypeDef g_canHandle;
static CAN_TxHeaderTypeDef g_txHeader;
static CAN_RxHeaderTypeDef g_rxHeader;
static uint32_t g_txMailbox;
#endif

#ifdef PLATFORM_ESP32
static twai_general_config_t g_generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
static twai_timing_config_t g_timingConfig = TWAI_TIMING_CONFIG_500KBITS();
static twai_filter_config_t g_filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#endif

#ifdef PLATFORM_LINUX
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>

static int g_canFd = -1;
#endif

/******************************************************************************
 * @brief     : Enable CAN clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 CAN clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure CAN GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* CAN1 GPIO Configuration: PA11(RX), PA12(TX) */
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* CAN1 TX (PA12) */
    gpioInit.Pin = GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* CAN1 RX (PA11) */
    gpioInit.Pin = GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
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
 * @brief     : Enable CAN module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanEnable(void)
{
#ifdef PLATFORM_STM32F4
    HAL_CAN_Start(&g_canHandle);
#elif defined(PLATFORM_STM32F1)
    HAL_CAN_Start(&g_canHandle);
#elif defined(PLATFORM_ESP32)
    twai_driver_install(&g_generalConfig, &g_timingConfig, &g_filterConfig);
    twai_start();
#elif defined(PLATFORM_LINUX)
    if (g_canFd < 0) {
        struct sockaddr_can addr;
        struct ifreq ifr;

        g_canFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (g_canFd >= 0) {
            if (strcpy_s(ifr.ifr_name, sizeof(ifr.ifr_name), "can0") != EOK) {
                close(g_canFd);
                g_canFd = -1;
                return;
            }
            if (ioctl(g_canFd, SIOCGIFINDEX, &ifr) < 0) {
                close(g_canFd);
                g_canFd = -1;
                return;
            }
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (bind(g_canFd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
                close(g_canFd);
                g_canFd = -1;
                return;
            }
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Check if CAN TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanTxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_CAN_GetTxMailboxesFreeLevel(&g_canHandle) > 0) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (HAL_CAN_GetTxMailboxesFreeLevel(&g_canHandle) > 0) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    twai_status_info_t status;
    twai_get_status_info(&status);
    return (status.msgs_to_tx == 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    return 1; /* Linux driver handles buffering */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write message to CAN
 * @param[in] : id - CAN message ID
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send (0-8)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanWriteMessage(uint32_t id, uint8_t* data, uint8_t length)
{
#ifdef PLATFORM_STM32F4
    if (length > 8) {
        return -1;
    }

    g_txHeader.StdId = id;
    g_txHeader.ExtId = 0;
    g_txHeader.RTR = CAN_RTR_DATA;
    g_txHeader.IDE = CAN_ID_STD;
    g_txHeader.DLC = length;
    g_txHeader.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&g_canHandle, &g_txHeader, data, &g_txMailbox) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    if (length > 8) {
        return -1;
    }

    g_txHeader.StdId = id;
    g_txHeader.ExtId = 0;
    g_txHeader.RTR = CAN_RTR_DATA;
    g_txHeader.IDE = CAN_ID_STD;
    g_txHeader.DLC = length;
    g_txHeader.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&g_canHandle, &g_txHeader, data, &g_txMailbox) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    if (length > 8) {
        return -1;
    }

    twai_message_t message;
    message.identifier = id;
    message.data_length_code = length;
    message.flags = TWAI_MSG_FLAG_NONE;
    for (int i = 0; i < length; i++) {
        message.data[i] = data[i];
    }

    return (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_canFd < 0 || length > 8) {
        return -1;
    }

    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = length;
    if (memcpy_s(frame.data, sizeof(frame.data), data, length) != EOK) {
        return -1;
    }

    return (write(g_canFd, &frame, sizeof(frame)) == sizeof(frame)) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if CAN RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanRxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_CAN_GetRxFifoFillLevel(&g_canHandle, CAN_RX_FIFO0) > 0) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (HAL_CAN_GetRxFifoFillLevel(&g_canHandle, CAN_RX_FIFO0) > 0) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    twai_status_info_t status;
    twai_get_status_info(&status);
    return (status.msgs_to_rx > 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_canFd >= 0) {
        int bytes = 0;
        if (ioctl(g_canFd, FIONREAD, &bytes) < 0) {
            return 0;
        }
        return (bytes > 0) ? 1 : 0;
    }
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Read message from CAN
 * @param[in] : id - Pointer to store received message ID
 * @param[in] : buffer - Pointer to buffer to store received data
 * @param[in] : maxLength - Maximum number of bytes to receive (should be 8)
 * @param[out]: id - Received message ID
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanReadMessage(uint32_t* id, uint8_t* buffer, uint8_t maxLength)
{
#ifdef PLATFORM_STM32F4
    if (maxLength < 8) {
        return -1;
    }

    if (HAL_CAN_GetRxMessage(&g_canHandle, CAN_RX_FIFO0, &g_rxHeader, buffer) == HAL_OK) {
        *id = (g_rxHeader.IDE == CAN_ID_STD) ? g_rxHeader.StdId : g_rxHeader.ExtId;
        return (int32_t)g_rxHeader.DLC;
    }
    return -1;
#elif defined(PLATFORM_STM32F1)
    if (maxLength < 8) {
        return -1;
    }

    if (HAL_CAN_GetRxMessage(&g_canHandle, CAN_RX_FIFO0, &g_rxHeader, buffer) == HAL_OK) {
        *id = (g_rxHeader.IDE == CAN_ID_STD) ? g_rxHeader.StdId : g_rxHeader.ExtId;
        return (int32_t)g_rxHeader.DLC;
    }
    return -1;
#elif defined(PLATFORM_ESP32)
    if (maxLength < 8) {
        return -1;
    }

    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        *id = message.identifier;
        for (int i = 0; i < message.data_length_code && i < maxLength; i++) {
            buffer[i] = message.data[i];
        }
        return (int32_t)message.data_length_code;
    }
    return -1;
#elif defined(PLATFORM_LINUX)
    if (g_canFd < 0 || maxLength < 8) {
        return -1;
    }

    struct can_frame frame;
    ssize_t nbytes = read(g_canFd, &frame, sizeof(frame));
    if (nbytes < 0) {
        return -1;
    }
    if (nbytes == sizeof(frame)) {
        *id = frame.can_id;
        if (memcpy_s(buffer, maxLength, frame.data, frame.can_dlc) != EOK) {
            return -1;
        }
        return (int32_t)frame.can_dlc;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure CAN baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t CanConfigureBaudRate(uint32_t baudRate)
{
#ifdef PLATFORM_STM32F4
    g_canHandle.Instance = CAN1;
    g_canHandle.Init.Prescaler = 6; /* For 500kbps with 42MHz APB1 */
    g_canHandle.Init.Mode = CAN_MODE_NORMAL;
    g_canHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_canHandle.Init.TimeSeg1 = CAN_BS1_11TQ;
    g_canHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
    g_canHandle.Init.TimeTriggeredMode = DISABLE;
    g_canHandle.Init.AutoBusOff = DISABLE;
    g_canHandle.Init.AutoWakeUp = DISABLE;
    g_canHandle.Init.AutoRetransmission = ENABLE;
    g_canHandle.Init.ReceiveFifoLocked = DISABLE;
    g_canHandle.Init.TransmitFifoPriority = DISABLE;

    if (baudRate == 125000) {
        g_canHandle.Init.Prescaler = 24;
    } else if (baudRate == 250000) {
        g_canHandle.Init.Prescaler = 12;
    } else if (baudRate == 500000) {
        g_canHandle.Init.Prescaler = 6;
    } else if (baudRate == 1000000) {
        g_canHandle.Init.Prescaler = 3;
    }

    return (HAL_CAN_Init(&g_canHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_canHandle.Instance = CAN1;
    g_canHandle.Init.Prescaler = 9; /* For 500kbps with 36MHz APB1 */
    g_canHandle.Init.Mode = CAN_MODE_NORMAL;
    g_canHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
    g_canHandle.Init.TimeSeg1 = CAN_BS1_6TQ;
    g_canHandle.Init.TimeSeg2 = CAN_BS2_1TQ;
    g_canHandle.Init.TimeTriggeredMode = DISABLE;
    g_canHandle.Init.AutoBusOff = DISABLE;
    g_canHandle.Init.AutoWakeUp = DISABLE;
    g_canHandle.Init.AutoRetransmission = ENABLE;
    g_canHandle.Init.ReceiveFifoLocked = DISABLE;
    g_canHandle.Init.TransmitFifoPriority = DISABLE;

    if (baudRate == 125000) {
        g_canHandle.Init.Prescaler = 36;
    } else if (baudRate == 250000) {
        g_canHandle.Init.Prescaler = 18;
    } else if (baudRate == 500000) {
        g_canHandle.Init.Prescaler = 9;
    } else if (baudRate == 1000000) {
        g_canHandle.Init.Prescaler = 4;
    }

    return (HAL_CAN_Init(&g_canHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    /* Update timing config based on baud rate */
    if (baudRate == 125000) {
        g_timingConfig = TWAI_TIMING_CONFIG_125KBITS();
    } else if (baudRate == 250000) {
        g_timingConfig = TWAI_TIMING_CONFIG_250KBITS();
    } else if (baudRate == 500000) {
        g_timingConfig = TWAI_TIMING_CONFIG_500KBITS();
    } else if (baudRate == 1000000) {
        g_timingConfig = TWAI_TIMING_CONFIG_1MBITS();
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux baud rate is configured via ip command: ip link set can0 type can bitrate <rate> */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable CAN RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanEnableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_CAN_ActivateNotification(&g_canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
#elif defined(PLATFORM_STM32F1)
    HAL_CAN_ActivateNotification(&g_canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts for notifications */
    twai_reconfigure_alerts(TWAI_ALERT_RX_DATA, NULL);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll mechanisms */
#endif
}

/******************************************************************************
 * @brief     : Enable CAN TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanEnableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_CAN_ActivateNotification(&g_canHandle, CAN_IT_TX_MAILBOX_EMPTY);
#elif defined(PLATFORM_STM32F1)
    HAL_CAN_ActivateNotification(&g_canHandle, CAN_IT_TX_MAILBOX_EMPTY);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts for notifications */
    twai_reconfigure_alerts(TWAI_ALERT_TX_IDLE, NULL);
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use TX interrupts in userspace */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for CAN
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

/******************************************************************************
 * @brief     : Disable CAN RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanDisableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_CAN_DeactivateNotification(&g_canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
#elif defined(PLATFORM_STM32F1)
    HAL_CAN_DeactivateNotification(&g_canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses alerts */
    twai_reconfigure_alerts(TWAI_ALERT_NONE, NULL);
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable CAN TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void CanDisableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    HAL_CAN_DeactivateNotification(&g_canHandle, CAN_IT_TX_MAILBOX_EMPTY);
#elif defined(PLATFORM_STM32F1)
    HAL_CAN_DeactivateNotification(&g_canHandle, CAN_IT_TX_MAILBOX_EMPTY);
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
void CanUpdateNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

#include "securec.h"
#include "flexray_hal.h"
#include "platform_config.h"
#include <string.h>

/* Platform-specific global variables and structures */
#ifdef PLATFORM_STM32F4
/* STM32F4 may have integrated FlexRay controller (e.g., STM32F407) or require external controller */
/* Using external E-Ray controller via SPI as an example */
static uint8_t g_flexrayEnabled = 0;
static uint16_t g_slotBufferStatus[FLEXRAY_MAX_SLOTS] = {0};
#endif

#ifdef PLATFORM_STM32F1
/* STM32F1 requires external FlexRay controller (E-Ray or TJA1080) */
static uint8_t g_flexrayEnabled = 0;
static uint16_t g_slotBufferStatus[FLEXRAY_MAX_SLOTS] = {0};
#endif

#ifdef PLATFORM_ESP32
/* ESP32 requires external FlexRay controller via SPI */
static uint8_t g_flexrayEnabled = 0;
static uint16_t g_slotBufferStatus[FLEXRAY_MAX_SLOTS] = {0};
#endif

#ifdef PLATFORM_LINUX
/* Linux FlexRay support via socketcan or character device */
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
static int g_flexrayFd = -1;
static uint16_t g_slotBufferStatus[FLEXRAY_MAX_SLOTS] = {0};
static uint8_t g_slotBuffers[FLEXRAY_MAX_SLOTS][FLEXRAY_MAX_PAYLOAD_LENGTH] = {0};
static uint16_t g_slotDataLengths[FLEXRAY_MAX_SLOTS] = {0};
#endif

/******************************************************************************
 * @brief     : Enable FlexRay clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    /* Enable clock for FlexRay controller or SPI interface */
    /* For external controller via SPI */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Enable clock for external controller interface */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 SPI clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure FlexRay GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* Configure SPI pins for external FlexRay controller */
    /* SPI2: SCK(PB13), MISO(PB14), MOSI(PB15), CS(PB12) */
    gpioInit.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    /* Chip Select pin */
    gpioInit.Pin = GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* Configure SPI pins for external FlexRay controller */
    /* SPI2: SCK(PB13), MISO(PB14), MOSI(PB15) */
    gpioInit.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    gpioInit.Pin = GPIO_PIN_14;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    /* Chip Select pin */
    gpioInit.Pin = GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by spi_bus_initialize */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses network interface or character device, no GPIO config */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Initialize FlexRay HAL
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalInit(void)
{
#ifdef PLATFORM_STM32F4
    /* Initialize SPI for external FlexRay controller communication */
    /* This is a simplified placeholder */
    g_flexrayEnabled = 0;
    if (memset_s(g_slotBufferStatus, sizeof(g_slotBufferStatus), 0, sizeof(g_slotBufferStatus)) != EOK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    g_flexrayEnabled = 0;
    if (memset_s(g_slotBufferStatus, sizeof(g_slotBufferStatus), 0, sizeof(g_slotBufferStatus)) != EOK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* Initialize SPI for external FlexRay controller */
    g_flexrayEnabled = 0;
    if (memset_s(g_slotBufferStatus, sizeof(g_slotBufferStatus), 0, sizeof(g_slotBufferStatus)) != EOK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Open FlexRay device */
    /* Try character device first (e.g., /dev/flexray0) */
    g_flexrayFd = open("/dev/flexray0", O_RDWR | O_NONBLOCK);
    if (g_flexrayFd < 0) {
        /* Fallback to socketcan-like interface if available */
        return -1;
    }
    if (memset_s(g_slotBufferStatus, sizeof(g_slotBufferStatus), 0, sizeof(g_slotBufferStatus)) != EOK) {
        return -1;
    }
    if (memset_s(g_slotBuffers, sizeof(g_slotBuffers), 0, sizeof(g_slotBuffers)) != EOK) {
        return -1;
    }
    if (memset_s(g_slotDataLengths, sizeof(g_slotDataLengths), 0, sizeof(g_slotDataLengths)) != EOK) {
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Deinitialize FlexRay HAL
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalDeinit(void)
{
#ifdef PLATFORM_STM32F4
    g_flexrayEnabled = 0;
#elif defined(PLATFORM_STM32F1)
    g_flexrayEnabled = 0;
#elif defined(PLATFORM_ESP32)
    g_flexrayEnabled = 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd >= 0) {
        close(g_flexrayFd);
        g_flexrayFd = -1;
    }
#endif
}

/******************************************************************************
 * @brief     : Configure FlexRay bit rate
 * @param[in] : bitRate - Desired bit rate in bps
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalConfigureBitRate(uint32_t bitRate)
{
#ifdef PLATFORM_STM32F4
    /* FlexRay standard bit rate is fixed at 10 Mbps */
    if (bitRate != FLEXRAY_DEFAULT_BITRATE) {
        return -1;
    }
    /* Configure external controller via SPI commands */
    /* Placeholder for actual SPI communication */
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (bitRate != FLEXRAY_DEFAULT_BITRATE) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    if (bitRate != FLEXRAY_DEFAULT_BITRATE) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Configure via ioctl if supported */
    if (g_flexrayFd >= 0) {
        /* Placeholder for ioctl configuration */
        return 0;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure FlexRay slot in hardware
 * @param[in] : slotId - Slot identifier
 * @param[in] : channel - Channel selection
 * @param[in] : payloadLength - Payload length in bytes
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalConfigureSlot(uint16_t slotId, uint8_t channel, uint16_t payloadLength)
{
#ifdef PLATFORM_STM32F4
    /* Configure message buffer in external controller */
    /* Send configuration commands via SPI */
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return -1;
    }
    g_slotBufferStatus[slotId] = 0x01;  /* Mark as configured */
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return -1;
    }
    g_slotBufferStatus[slotId] = 0x01;
    return 0;
#elif defined(PLATFORM_ESP32)
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return -1;
    }
    g_slotBufferStatus[slotId] = 0x01;
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Configure slot via ioctl or device-specific interface */
    if (g_flexrayFd >= 0 && slotId < FLEXRAY_MAX_SLOTS) {
        g_slotBufferStatus[slotId] = 0x01;
        return 0;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Send FlexRay frame
 * @param[in] : slotId - Slot identifier
 * @param[in] : frame - Pointer to frame structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalSendFrame(uint16_t slotId, FlexRayFrame_t* frame)
{
#ifdef PLATFORM_STM32F4
    /* Send frame to external controller via SPI */
    if (slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if ((g_slotBufferStatus[slotId] & 0x01) == 0) {
        return -1;  /* Slot not configured */
    }
    /* Placeholder: Write frame data to controller via SPI */
    /* In real implementation, this would involve:
     * 1. Select chip
     * 2. Send write command with slot ID
     * 3. Transfer frame header and payload
     * 4. Deselect chip
     */
    g_slotBufferStatus[slotId] |= 0x02;  /* Mark as pending TX */
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if ((g_slotBufferStatus[slotId] & 0x01) == 0) {
        return -1;
    }
    g_slotBufferStatus[slotId] |= 0x02;
    return 0;
#elif defined(PLATFORM_ESP32)
    if (slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if ((g_slotBufferStatus[slotId] & 0x01) == 0) {
        return -1;
    }
    g_slotBufferStatus[slotId] |= 0x02;
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd < 0 || slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    /* Write frame to device */
    /* Construct device-specific frame format */
    uint8_t txBuffer[FLEXRAY_MAX_FRAME_LENGTH];
    uint16_t offset = 0;

    /* Frame header */
    txBuffer[offset++] = (uint8_t)(slotId >> 8);
    txBuffer[offset++] = (uint8_t)(slotId & 0xFF);
    txBuffer[offset++] = frame->channel;
    txBuffer[offset++] = frame->payloadLength;
    txBuffer[offset++] = frame->cycleCount;

    /* Payload */
    uint16_t payloadBytes = frame->payloadLength * 2;
    if (payloadBytes > 0) {
        if (memcpy_s(&txBuffer[offset], sizeof(txBuffer) - offset, frame->payload, payloadBytes) != EOK) {
            return -1;
        }
        offset += payloadBytes;
    }

    /* Write to device */
    if (write(g_flexrayFd, txBuffer, offset) == offset) {
        return 0;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if RX slot has data
 * @param[in] : slotId - Slot identifier
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalRxSlotHasData(uint16_t slotId)
{
#ifdef PLATFORM_STM32F4
    /* Check if slot has received data */
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return 0;
    }
    /* Poll external controller status via SPI */
    /* Placeholder: In real implementation, read status register */
    return (g_slotBufferStatus[slotId] & 0x04) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return 0;
    }
    return (g_slotBufferStatus[slotId] & 0x04) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    if (slotId >= FLEXRAY_MAX_SLOTS) {
        return 0;
    }
    return (g_slotBufferStatus[slotId] & 0x04) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd < 0 || slotId >= FLEXRAY_MAX_SLOTS) {
        return 0;
    }
    /* Check if data available for this slot */
    return (g_slotDataLengths[slotId] > 0) ? 1 : 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Receive FlexRay frame
 * @param[in] : slotId - Slot identifier
 * @param[out]: frame - Pointer to frame structure to fill
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalReceiveFrame(uint16_t slotId, FlexRayFrame_t* frame)
{
#ifdef PLATFORM_STM32F4
    /* Read frame from external controller via SPI */
    if (slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if ((g_slotBufferStatus[slotId] & 0x04) == 0) {
        return -1;  /* No data available */
    }
    /* Placeholder: Read frame data from controller via SPI */
    /* In real implementation:
     * 1. Select chip
     * 2. Send read command with slot ID
     * 3. Read frame header and payload
     * 4. Deselect chip
     */
    g_slotBufferStatus[slotId] &= ~0x04;  /* Clear RX flag */
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if ((g_slotBufferStatus[slotId] & 0x04) == 0) {
        return -1;
    }
    g_slotBufferStatus[slotId] &= ~0x04;
    return 0;
#elif defined(PLATFORM_ESP32)
    if (slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if ((g_slotBufferStatus[slotId] & 0x04) == 0) {
        return -1;
    }
    g_slotBufferStatus[slotId] &= ~0x04;
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd < 0 || slotId >= FLEXRAY_MAX_SLOTS || frame == NULL) {
        return -1;
    }
    if (g_slotDataLengths[slotId] == 0) {
        return -1;  /* No data */
    }
    /* Copy stored data to frame */
    frame->slotId = slotId;
    uint16_t dataLen = g_slotDataLengths[slotId];
    if (dataLen > FLEXRAY_MAX_PAYLOAD_LENGTH) {
        dataLen = FLEXRAY_MAX_PAYLOAD_LENGTH;
    }
    if (memcpy_s(frame->payload, sizeof(frame->payload), g_slotBuffers[slotId], dataLen) != EOK) {
        return -1;
    }
    frame->payloadLength = (uint8_t)(dataLen / 2);

    /* Clear buffer */
    g_slotDataLengths[slotId] = 0;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Start FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalStartCommunication(void)
{
#ifdef PLATFORM_STM32F4
    /* Send start command to external controller */
    /* This initiates the FlexRay startup procedure */
    g_flexrayEnabled = 1;
    return 0;
#elif defined(PLATFORM_STM32F1)
    g_flexrayEnabled = 1;
    return 0;
#elif defined(PLATFORM_ESP32)
    g_flexrayEnabled = 1;
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd >= 0) {
        /* Start communication via ioctl */
        /* Placeholder for actual ioctl command */
        return 0;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Stop FlexRay communication
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalStopCommunication(void)
{
#ifdef PLATFORM_STM32F4
    /* Send halt command to external controller */
    g_flexrayEnabled = 0;
    return 0;
#elif defined(PLATFORM_STM32F1)
    g_flexrayEnabled = 0;
    return 0;
#elif defined(PLATFORM_ESP32)
    g_flexrayEnabled = 0;
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd >= 0) {
        /* Stop communication via ioctl */
        return 0;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Update FlexRay status from hardware
 * @param[in] : status - Pointer to status structure
 * @param[out]: status - Updated status structure
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalUpdateStatus(FlexRayStatus_t* status)
{
#ifdef PLATFORM_STM32F4
    /* Read status from external controller via SPI */
    if (status == NULL) {
        return;
    }
    /* Placeholder: In real implementation, read status registers */
    if (g_flexrayEnabled) {
        status->state = FLEXRAY_STATUS_ACTIVE;
        status->syncState = 1;
    }
#elif defined(PLATFORM_STM32F1)
    if (status == NULL) {
        return;
    }
    if (g_flexrayEnabled) {
        status->state = FLEXRAY_STATUS_ACTIVE;
        status->syncState = 1;
    }
#elif defined(PLATFORM_ESP32)
    if (status == NULL) {
        return;
    }
    if (g_flexrayEnabled) {
        status->state = FLEXRAY_STATUS_ACTIVE;
        status->syncState = 1;
    }
#elif defined(PLATFORM_LINUX)
    if (status == NULL || g_flexrayFd < 0) {
        return;
    }
    /* Read status via ioctl or from device */
    /* Placeholder for actual status query */
#endif
}

/******************************************************************************
 * @brief     : Configure FlexRay timing parameters
 * @param[in] : cycleLength - Cycle length in macroticks
 * @param[in] : staticSlots - Number of static slots
 * @param[in] : dynamicSlots - Number of dynamic slots
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t FlexRayHalConfigureTiming(uint16_t cycleLength, uint16_t staticSlots, uint16_t dynamicSlots)
{
#ifdef PLATFORM_STM32F4
    /* Configure timing in external controller */
    /* Send configuration commands via SPI */
    if (cycleLength == 0 || staticSlots + dynamicSlots > FLEXRAY_MAX_SLOTS) {
        return -1;
    }
    /* Placeholder for SPI configuration sequence */
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (cycleLength == 0 || staticSlots + dynamicSlots > FLEXRAY_MAX_SLOTS) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    if (cycleLength == 0 || staticSlots + dynamicSlots > FLEXRAY_MAX_SLOTS) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_flexrayFd >= 0) {
        /* Configure via ioctl */
        return 0;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable FlexRay slot interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalEnableSlotInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    /* Enable interrupt pin from external controller */
    /* Configure GPIO as interrupt input */
#elif defined(PLATFORM_STM32F1)
    /* Enable interrupt */
#elif defined(PLATFORM_ESP32)
    /* Configure ESP32 GPIO interrupt for external controller INT pin */
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or signals */
#endif
}

/******************************************************************************
 * @brief     : Enable FlexRay error interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalEnableErrorInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    /* Enable error interrupt from external controller */
#elif defined(PLATFORM_STM32F1)
    /* Enable error interrupt */
#elif defined(PLATFORM_ESP32)
    /* Configure error interrupt */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for FlexRay
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    /* Configure NVIC for external interrupt line */
    /* Example: External interrupt on EXTI line */
    /* HAL_NVIC_SetPriority(EXTIx_IRQn, 2, 0); */
    /* HAL_NVIC_EnableIRQ(EXTIx_IRQn); */
#elif defined(PLATFORM_STM32F1)
    /* Configure NVIC */
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable FlexRay slot interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalDisableSlotInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    /* Disable slot interrupt */
#elif defined(PLATFORM_STM32F1)
    /* Disable slot interrupt */
#elif defined(PLATFORM_ESP32)
    /* Disable interrupt */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable FlexRay error interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void FlexRayHalDisableErrorInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    /* Disable error interrupt */
#elif defined(PLATFORM_STM32F1)
    /* Disable error interrupt */
#elif defined(PLATFORM_ESP32)
    /* Disable error interrupt */
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
void FlexRayHalUpdateNvic(void)
{
#ifdef PLATFORM_STM32F4
    /* Disable NVIC interrupts */
    /* HAL_NVIC_DisableIRQ(EXTIx_IRQn); */
#elif defined(PLATFORM_STM32F1)
    /* Update NVIC */
#elif defined(PLATFORM_ESP32)
    /* Update interrupt configuration */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

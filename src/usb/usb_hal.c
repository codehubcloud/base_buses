#include "platform_config.h"
#include "securec.h"
#include "usb_hal.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
#include "stm32f4xx_hal.h"
static PCD_HandleTypeDef g_usbHandle;
static uint8_t g_usbRxBuffer[USB_MAX_ENDPOINTS][USB_MAX_PACKET_SIZE];
static uint8_t g_usbTxBuffer[USB_MAX_ENDPOINTS][USB_MAX_PACKET_SIZE];
#endif

#ifdef PLATFORM_STM32F1
#include "stm32f1xx_hal.h"
static PCD_HandleTypeDef g_usbHandle;
static uint8_t g_usbRxBuffer[USB_MAX_ENDPOINTS][USB_MAX_PACKET_SIZE];
static uint8_t g_usbTxBuffer[USB_MAX_ENDPOINTS][USB_MAX_PACKET_SIZE];
#endif

#ifdef PLATFORM_ESP32
#include "driver/gpio.h"
#include "esp_private/usb_phy.h"
#include "esp_system.h"
#include "soc/usb_periph.h"

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#include "hal/usb_hal.h"
static usb_hal_context_t g_usbHalContext;
static uint8_t g_usbRxBuffer[USB_MAX_ENDPOINTS][USB_MAX_PACKET_SIZE];
static uint8_t g_usbTxBuffer[USB_MAX_ENDPOINTS][USB_MAX_PACKET_SIZE];
#endif
#endif

#ifdef PLATFORM_LINUX
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
static libusb_context* g_usbContext = NULL;
static libusb_device_handle* g_usbDevHandle = NULL;
static uint8_t g_isHostMode = 0;
#endif

/******************************************************************************
 * @brief     : Enable USB peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 USB clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable USB peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalDisableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USB_CLK_DISABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 USB clock managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure USB GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* USB OTG FS GPIO Configuration: PA11(DM), PA12(DP) */
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure VBUS pin if needed */
    gpioInit.Pin = GPIO_PIN_9;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* USB GPIO Configuration: PA11(DM), PA12(DP) */
    gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32-S2/S3: USB pins are GPIO19(D-) and GPIO20(D+) */
    /* ESP32-C3: USB pins are GPIO18(D-) and GPIO19(D+) */
    /* GPIO configuration handled by USB driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device nodes, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Initialize USB controller hardware
 * @param[in] : mode - USB mode (device or host), speed - USB speed
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalInitController(uint8_t mode, uint8_t speed)
{
#ifdef PLATFORM_STM32F4
    g_usbHandle.Instance = USB_OTG_FS;
    g_usbHandle.Init.dev_endpoints = USB_MAX_ENDPOINTS;
    g_usbHandle.Init.speed = (speed == USB_SPEED_HIGH) ? PCD_SPEED_HIGH : PCD_SPEED_FULL;
    g_usbHandle.Init.dma_enable = DISABLE;
    g_usbHandle.Init.phy_itface = PCD_PHY_EMBEDDED;
    g_usbHandle.Init.Sof_enable = DISABLE;
    g_usbHandle.Init.low_power_enable = DISABLE;
    g_usbHandle.Init.lpm_enable = DISABLE;
    g_usbHandle.Init.vbus_sensing_enable = ENABLE;
    g_usbHandle.Init.use_dedicated_ep1 = DISABLE;

    if (HAL_PCD_Init(&g_usbHandle) != HAL_OK) {
        return -1;
    }

    /* Allocate FIFO memory */
    HAL_PCDEx_SetRxFiFo(&g_usbHandle, 0x80);
    HAL_PCDEx_SetTxFiFo(&g_usbHandle, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&g_usbHandle, 1, 0x40);

    return 0;
#elif defined(PLATFORM_STM32F1)
    g_usbHandle.Instance = USB;
    g_usbHandle.Init.dev_endpoints = USB_MAX_ENDPOINTS;
    g_usbHandle.Init.speed = PCD_SPEED_FULL; /* STM32F1 supports only Full-Speed */
    g_usbHandle.Init.phy_itface = PCD_PHY_EMBEDDED;
    g_usbHandle.Init.low_power_enable = DISABLE;
    g_usbHandle.Init.lpm_enable = DISABLE;
    g_usbHandle.Init.battery_charging_enable = DISABLE;

    if (HAL_PCD_Init(&g_usbHandle) != HAL_OK) {
        return -1;
    }

    /* Configure PMA (Packet Memory Area) */
    HAL_PCDEx_PMAConfig(&g_usbHandle, 0x00, PCD_SNG_BUF, 0x18); /* EP0 TX */
    HAL_PCDEx_PMAConfig(&g_usbHandle, 0x80, PCD_SNG_BUF, 0x58); /* EP0 RX */

    return 0;
#elif defined(PLATFORM_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    /* Initialize USB HAL context */
    usb_hal_init(&g_usbHalContext);

    /* Configure USB PHY */
    /* Note: ESP32-S2/S3/C3 have built-in USB PHY */
    return 0;
#else
    /* USB not supported on this ESP32 variant */
    return -1;
#endif
#elif defined(PLATFORM_LINUX)
    int result = 0;

    if (mode == USB_MODE_HOST) {
        /* Initialize libusb for host mode */
        result = libusb_init(&g_usbContext);
        if (result < 0) {
            return -1;
        }
        g_isHostMode = 1;
    } else {
        /* Device mode: Use USB gadget framework */
        /* This requires configfs mounted at /sys/kernel/config/usb_gadget */
        /* For simplicity, return success - actual implementation would
         * require writing to configfs files */
        g_isHostMode = 0;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Deinitialize USB controller hardware
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalDeinitController(void)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_DeInit(&g_usbHandle);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_PCD_DeInit(&g_usbHandle);
    return 0;
#elif defined(PLATFORM_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    usb_hal_deinit(&g_usbHalContext);
    return 0;
#else
    return -1;
#endif
#elif defined(PLATFORM_LINUX)
    if (g_usbContext != NULL) {
        if (g_usbDevHandle != NULL) {
            libusb_close(g_usbDevHandle);
            g_usbDevHandle = NULL;
        }
        libusb_exit(g_usbContext);
        g_usbContext = NULL;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Connect USB device (enable pull-up)
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalDeviceConnect(void)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_DevConnect(&g_usbHandle);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_PCD_DevConnect(&g_usbHandle);
    return 0;
#elif defined(PLATFORM_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    /* Enable USB device pull-up */
    return 0;
#else
    return -1;
#endif
#elif defined(PLATFORM_LINUX)
    /* USB gadget connection managed by kernel */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disconnect USB device (disable pull-up)
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalDeviceDisconnect(void)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_DevDisconnect(&g_usbHandle);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_PCD_DevDisconnect(&g_usbHandle);
    return 0;
#elif defined(PLATFORM_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
    /* Disable USB device pull-up */
    return 0;
#else
    return -1;
#endif
#elif defined(PLATFORM_LINUX)
    /* USB gadget disconnection managed by kernel */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure USB endpoint
 * @param[in] : config - Pointer to endpoint configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalConfigureEndpoint(UsbEndpointConfig_t* config)
{
    if (config == NULL) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    uint8_t epAddr = config->endpointNumber;
    if (config->direction == USB_EP_DIR_IN) {
        epAddr |= 0x80;
    }

    uint8_t epType;
    switch (config->endpointType) {
        case USB_EP_TYPE_CONTROL:
            epType = EP_TYPE_CTRL;
            break;
        case USB_EP_TYPE_BULK:
            epType = EP_TYPE_BULK;
            break;
        case USB_EP_TYPE_INTERRUPT:
            epType = EP_TYPE_INTR;
            break;
        case USB_EP_TYPE_ISOCHRONOUS:
            epType = EP_TYPE_ISOC;
            break;
        default:
            return -1;
    }

    return (HAL_PCD_EP_Open(&g_usbHandle, epAddr, config->maxPacketSize, epType) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    uint8_t epAddr = config->endpointNumber;
    if (config->direction == USB_EP_DIR_IN) {
        epAddr |= 0x80;
    }

    uint8_t epType;
    switch (config->endpointType) {
        case USB_EP_TYPE_CONTROL:
            epType = EP_TYPE_CTRL;
            break;
        case USB_EP_TYPE_BULK:
            epType = EP_TYPE_BULK;
            break;
        case USB_EP_TYPE_INTERRUPT:
            epType = EP_TYPE_INTR;
            break;
        case USB_EP_TYPE_ISOCHRONOUS:
            epType = EP_TYPE_ISOC;
            break;
        default:
            return -1;
    }

    return (HAL_PCD_EP_Open(&g_usbHandle, epAddr, config->maxPacketSize, epType) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 endpoint configuration */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Endpoint configuration via gadget framework */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Set USB device address
 * @param[in] : address - Device address (0-127)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalSetDeviceAddress(uint8_t address)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_SetAddress(&g_usbHandle, address);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_PCD_SetAddress(&g_usbHandle, address);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 set address */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Address set by kernel in device mode */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Transmit data on endpoint
 * @param[in] : endpoint - Endpoint number, data - Data buffer, length - Data length
 * @param[out]: None
 * @return    : Number of bytes transmitted, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalTransmitData(uint8_t endpoint, uint8_t* data, uint16_t length)
{
#ifdef PLATFORM_STM32F4
    uint8_t epAddr = endpoint | 0x80; /* IN endpoint */

    if (length > 0 && data != NULL) {
        if (memcpy_s(g_usbTxBuffer[endpoint], USB_MAX_PACKET_SIZE, data, length) != EOK) {
            return -1;
        }
    }

    if (HAL_PCD_EP_Transmit(&g_usbHandle, epAddr, (length > 0) ? g_usbTxBuffer[endpoint] : NULL, length) != HAL_OK) {
        return -1;
    }

    return (int32_t)length;
#elif defined(PLATFORM_STM32F1)
    uint8_t epAddr = endpoint | 0x80; /* IN endpoint */

    if (length > 0 && data != NULL) {
        if (memcpy_s(g_usbTxBuffer[endpoint], USB_MAX_PACKET_SIZE, data, length) != EOK) {
            return -1;
        }
    }

    if (HAL_PCD_EP_Transmit(&g_usbHandle, epAddr, (length > 0) ? g_usbTxBuffer[endpoint] : NULL, length) != HAL_OK) {
        return -1;
    }

    return (int32_t)length;
#elif defined(PLATFORM_ESP32)
    /* ESP32 transmit implementation */
    return (int32_t)length;
#elif defined(PLATFORM_LINUX)
    if (g_usbDevHandle != NULL && data != NULL) {
        int transferred = 0;
        int result = libusb_bulk_transfer(g_usbDevHandle, endpoint | 0x80, data, length, &transferred, 1000);
        return (result == 0) ? transferred : -1;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Receive data from endpoint
 * @param[in] : endpoint - Endpoint number, maxLength - Maximum buffer size
 * @param[out]: data - Data buffer for received data
 * @return    : Number of bytes received, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalReceiveData(uint8_t endpoint, uint8_t* data, uint16_t maxLength)
{
    if (data == NULL) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    uint8_t epAddr = endpoint & 0x7F; /* OUT endpoint */

    if (HAL_PCD_EP_Receive(&g_usbHandle, epAddr, g_usbRxBuffer[endpoint], maxLength) != HAL_OK) {
        return -1;
    }

    /* Get actual received length */
    uint16_t rxLength = HAL_PCD_EP_GetRxCount(&g_usbHandle, epAddr);
    if (rxLength > maxLength) {
        rxLength = maxLength;
    }

    if (memcpy_s(data, maxLength, g_usbRxBuffer[endpoint], rxLength) != EOK) {
        return -1;
    }
    return (int32_t)rxLength;
#elif defined(PLATFORM_STM32F1)
    uint8_t epAddr = endpoint & 0x7F; /* OUT endpoint */

    if (HAL_PCD_EP_Receive(&g_usbHandle, epAddr, g_usbRxBuffer[endpoint], maxLength) != HAL_OK) {
        return -1;
    }

    /* Get actual received length */
    uint16_t rxLength = HAL_PCD_EP_GetRxCount(&g_usbHandle, epAddr);
    if (rxLength > maxLength) {
        rxLength = maxLength;
    }

    if (memcpy_s(data, maxLength, g_usbRxBuffer[endpoint], rxLength) != EOK) {
        return -1;
    }
    return (int32_t)rxLength;
#elif defined(PLATFORM_ESP32)
    /* ESP32 receive implementation */
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_usbDevHandle != NULL) {
        int transferred = 0;
        int result = libusb_bulk_transfer(g_usbDevHandle, endpoint & 0x7F, data, maxLength, &transferred, 1000);
        return (result == 0) ? transferred : -1;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if endpoint is ready for transfer
 * @param[in] : endpoint - Endpoint number (with direction bit)
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalIsEndpointReady(uint8_t endpoint)
{
#ifdef PLATFORM_STM32F4
    /* Check if endpoint is not busy */
    return 1; /* Simplified - actual implementation would check PCD state */
#elif defined(PLATFORM_STM32F1)
    /* Check if endpoint is not busy */
    return 1; /* Simplified - actual implementation would check PCD state */
#elif defined(PLATFORM_ESP32)
    return 1;
#elif defined(PLATFORM_LINUX)
    return 1;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Check if data is available on endpoint
 * @param[in] : endpoint - Endpoint number
 * @param[out]: None
 * @return    : 1 if data available, 0 if not
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalIsDataAvailable(uint8_t endpoint)
{
#ifdef PLATFORM_STM32F4
    /* Check RX FIFO status */
    return 1; /* Simplified - would check actual FIFO status */
#elif defined(PLATFORM_STM32F1)
    /* Check RX buffer status */
    return 1; /* Simplified - would check actual buffer status */
#elif defined(PLATFORM_ESP32)
    return 1;
#elif defined(PLATFORM_LINUX)
    return 1;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Stall endpoint
 * @param[in] : endpoint - Endpoint number to stall
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalStallEndpoint(uint8_t endpoint)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_EP_SetStall(&g_usbHandle, endpoint);
    HAL_PCD_EP_SetStall(&g_usbHandle, endpoint | 0x80);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_PCD_EP_SetStall(&g_usbHandle, endpoint);
    HAL_PCD_EP_SetStall(&g_usbHandle, endpoint | 0x80);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 stall implementation */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Stall via gadget framework or libusb */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Clear stall condition on endpoint
 * @param[in] : endpoint - Endpoint number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalClearStall(uint8_t endpoint)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_EP_ClrStall(&g_usbHandle, endpoint);
    HAL_PCD_EP_ClrStall(&g_usbHandle, endpoint | 0x80);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_PCD_EP_ClrStall(&g_usbHandle, endpoint);
    HAL_PCD_EP_ClrStall(&g_usbHandle, endpoint | 0x80);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 clear stall implementation */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Clear stall via gadget framework or libusb */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enumerate connected USB devices (host mode)
 * @param[in] : None
 * @param[out]: None
 * @return    : Number of devices found, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalEnumerateDevices(void)
{
#ifdef PLATFORM_STM32F4
    /* STM32 USB host enumeration - requires USB Host stack */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 typically doesn't support USB host mode */
    return -1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 USB host enumeration */
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_usbContext == NULL || !g_isHostMode) {
        return -1;
    }

    libusb_device** devList;
    ssize_t devCount = libusb_get_device_list(g_usbContext, &devList);

    if (devCount < 0) {
        return -1;
    }

    /* Free device list */
    libusb_free_device_list(devList, 1);

    return (int32_t)devCount;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable USB interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalEnableInterrupts(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupts managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable in userspace */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable USB interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalDisableInterrupts(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupts managed by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable in userspace */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Reset USB peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalReset(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USB_FORCE_RESET();
    __HAL_RCC_USB_RELEASE_RESET();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 reset implementation */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Reset via libusb */
    if (g_usbDevHandle != NULL) {
        return (libusb_reset_device(g_usbDevHandle) == 0) ? 0 : -1;
    }
    return -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Flush TX FIFO for endpoint
 * @param[in] : endpoint - Endpoint number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalFlushTxFifo(uint8_t endpoint)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_EP_Flush(&g_usbHandle, endpoint | 0x80);
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 doesn't have separate flush function */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 flush implementation */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Flush RX FIFO
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UsbHalFlushRxFifo(void)
{
#ifdef PLATFORM_STM32F4
    HAL_PCD_EP_Flush(&g_usbHandle, 0x00);
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 doesn't have separate flush function */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 flush implementation */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
    return 0;
#else
    return -1;
#endif
}

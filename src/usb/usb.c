#include <string.h>
#include "securec.h"
#include "usb.h"
#include "usb_hal.h"


/* USB Module State */
static struct {
    uint8_t mode;
    uint8_t speed;
    uint8_t deviceState;
    uint8_t deviceAddress;
    uint8_t isInitialized;
    uint8_t isConnected;
} g_usbState = {0};

/* Device Descriptor - Default Configuration */
static UsbDeviceDescriptor_t g_deviceDescriptor = {.bLength = sizeof(UsbDeviceDescriptor_t),
                                                   .bDescriptorType = USB_DESC_TYPE_DEVICE,
                                                   .bcdUSB = 0x0200, /* USB 2.0 */
                                                   .bDeviceClass = 0x00,
                                                   .bDeviceSubClass = 0x00,
                                                   .bDeviceProtocol = 0x00,
                                                   .bMaxPacketSize0 = USB_MAX_PACKET_SIZE,
                                                   .idVendor = 0x1234,
                                                   .idProduct = 0x5678,
                                                   .bcdDevice = 0x0100,
                                                   .iManufacturer = 0x01,
                                                   .iProduct = 0x02,
                                                   .iSerialNumber = 0x03,
                                                   .bNumConfigurations = 0x01};

/******************************************************************************
 * @brief     : Initialize USB peripheral
 * @param[in] : mode --USB operating mode (USB_MODE_DEVICE or USB_MODE_HOST)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures USB hardware for specified mode
 *****************************************************************************/
int32_t UsbInit(uint8_t mode)
{
    int32_t result = 0;

    if (g_usbState.isInitialized) {
        return -1;
    }

    if ((mode != USB_MODE_DEVICE) && (mode != USB_MODE_HOST)) {
        return -1;
    }

    g_usbState.mode = mode;
    g_usbState.deviceState = USB_STATE_DEFAULT;
    g_usbState.deviceAddress = 0;
    g_usbState.isConnected = 0;

    result = UsbHalEnableClock();
    if (result != 0) {
        return -1;
    }

    result = UsbHalConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = UsbHalInitController(mode, g_usbState.speed);
    if (result != 0) {
        return -1;
    }

    /* Configure default control endpoint (EP0) */
    UsbEndpointConfig_t ep0Config = {.endpointNumber = 0,
                                     .endpointType = USB_EP_TYPE_CONTROL,
                                     .direction = USB_EP_DIR_OUT,
                                     .maxPacketSize = USB_MAX_PACKET_SIZE,
                                     .interval = 0};
    UsbHalConfigureEndpoint(&ep0Config);

    g_usbState.isInitialized = 1;

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize USB peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables USB hardware and releases resources
 *****************************************************************************/
int32_t UsbDeinit(void)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (g_usbState.isConnected) {
        UsbDeviceDisconnect();
    }

    UsbHalDeinitController();
    UsbHalDisableClock();

    g_usbState.isInitialized = 0;
    g_usbState.deviceState = USB_STATE_DEFAULT;
    g_usbState.deviceAddress = 0;

    return 0;
}

/******************************************************************************
 * @brief     : Connect USB device to host
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables USB pull-up resistor (device mode only)
 *****************************************************************************/
int32_t UsbDeviceConnect(void)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (g_usbState.mode != USB_MODE_DEVICE) {
        return -1;
    }

    if (g_usbState.isConnected) {
        return 0;
    }

    if (UsbHalDeviceConnect() != 0) {
        return -1;
    }

    g_usbState.isConnected = 1;
    g_usbState.deviceState = USB_STATE_DEFAULT;

    return 0;
}

/******************************************************************************
 * @brief     : Disconnect USB device from host
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables USB pull-up resistor (device mode only)
 *****************************************************************************/
int32_t UsbDeviceDisconnect(void)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (g_usbState.mode != USB_MODE_DEVICE) {
        return -1;
    }

    if (!g_usbState.isConnected) {
        return 0;
    }

    UsbHalDeviceDisconnect();

    g_usbState.isConnected = 0;
    g_usbState.deviceState = USB_STATE_DEFAULT;
    g_usbState.deviceAddress = 0;

    return 0;
}

/******************************************************************************
 * @brief     : Send data through USB endpoint
 * @param[in] : endpoint --Endpoint number (0-7), data - Pointer to data buffer, length - Number of bytes to send
 * @param[out]: None
 * @return    : Number of bytes sent, -1 if error
 * @note      : Blocking function for control/bulk transfers
 *****************************************************************************/
int32_t UsbSendData(uint8_t endpoint, uint8_t* data, uint16_t length)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (data == NULL) {
        return -1;
    }

    if (endpoint >= USB_MAX_ENDPOINTS) {
        return -1;
    }

    if (g_usbState.mode == USB_MODE_DEVICE && !g_usbState.isConnected) {
        return -1;
    }

    /* Wait for endpoint to be ready */
    uint32_t timeout = 10000;
    while (UsbHalIsEndpointReady(endpoint | 0x80) == 0 && timeout > 0) {
        timeout--;
    }

    if (timeout == 0) {
        return -1;
    }

    return UsbHalTransmitData(endpoint, data, length);
}

/******************************************************************************
 * @brief     : Receive data from USB endpoint
 * @param[in] : endpoint --Endpoint number (0-7), maxLength - Maximum number of bytes to receive
 * @param[out]: data --Pointer to buffer for received data
 * @return    : Number of bytes received, -1 if error
 * @note      : Blocking function with timeout
 *****************************************************************************/
int32_t UsbReceiveData(uint8_t endpoint, uint8_t* data, uint16_t maxLength)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (data == NULL) {
        return -1;
    }

    if (endpoint >= USB_MAX_ENDPOINTS) {
        return -1;
    }

    if (g_usbState.mode == USB_MODE_DEVICE && !g_usbState.isConnected) {
        return -1;
    }

    /* Check if data is available */
    if (UsbHalIsDataAvailable(endpoint) == 0) {
        return 0;
    }

    return UsbHalReceiveData(endpoint, data, maxLength);
}

/******************************************************************************
 * @brief     : Process USB control transfer setup packet
 * @param[in] : setupPacket --Pointer to 8-byte setup packet data
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Handles standard USB device requests
 *****************************************************************************/
int32_t UsbSetupControlTransfer(uint8_t* setupPacket)
{
    if (!g_usbState.isInitialized || setupPacket == NULL) {
        return -1;
    }

    UsbSetupPacket_t* setup = (UsbSetupPacket_t*)setupPacket;

    /* Process standard device requests */
    switch (setup->bRequest) {
        case USB_REQ_GET_DESCRIPTOR:
            if ((setup->wValue >> 8) == USB_DESC_TYPE_DEVICE) {
                /* Send device descriptor */
                UsbSendData(0, (uint8_t*)&g_deviceDescriptor,
                            (setup->wLength < sizeof(g_deviceDescriptor)) ? setup->wLength : sizeof(g_deviceDescriptor));
            } else {
                /* Unsupported descriptor type */
                UsbStallEndpoint(0);
            }
            break;

        case USB_REQ_SET_ADDRESS:
            g_usbState.deviceAddress = (uint8_t)(setup->wValue & 0x7F);
            UsbSetAddress(g_usbState.deviceAddress);
            g_usbState.deviceState = USB_STATE_ADDRESSED;
            /* Send zero-length status packet */
            UsbSendData(0, NULL, 0);
            break;

        case USB_REQ_SET_CONFIGURATION:
            if (setup->wValue != 0) {
                g_usbState.deviceState = USB_STATE_CONFIGURED;
            } else {
                g_usbState.deviceState = USB_STATE_ADDRESSED;
            }
            /* Send zero-length status packet */
            UsbSendData(0, NULL, 0);
            break;

        case USB_REQ_GET_CONFIGURATION: {
            uint8_t config = (g_usbState.deviceState == USB_STATE_CONFIGURED) ? 1 : 0;
            UsbSendData(0, &config, 1);
        } break;

        case USB_REQ_GET_STATUS: {
            uint16_t status = 0x0000; /* Self-powered, no remote wakeup */
            UsbSendData(0, (uint8_t*)&status, 2);
        } break;

        case USB_REQ_SET_FEATURE:
        case USB_REQ_CLEAR_FEATURE:
            /* Send zero-length status packet */
            UsbSendData(0, NULL, 0);
            break;

        default:
            /* Unsupported request */
            UsbStallEndpoint(0);
            return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Get USB device descriptor
 * @param[in] : maxLength --Maximum descriptor size
 * @param[out]: descriptor --Pointer to buffer for device descriptor
 * @return    : Actual descriptor size, -1 if error
 * @note      : Returns the device descriptor structure
 *****************************************************************************/
int32_t UsbGetDeviceDescriptor(uint8_t* descriptor, uint16_t maxLength)
{
    if (descriptor == NULL) {
        return -1;
    }

    uint16_t copySize = (maxLength < sizeof(g_deviceDescriptor)) ? maxLength : sizeof(g_deviceDescriptor);

    if (memcpy_s(descriptor, maxLength, &g_deviceDescriptor, copySize) != EOK) {
        return -1;
    }

    return (int32_t)copySize;
}

/******************************************************************************
 * @brief     : Set USB device address
 * @param[in] : address --Device address (0-127)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Used during enumeration process
 *****************************************************************************/
int32_t UsbSetAddress(uint8_t address)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (address > 127) {
        return -1;
    }

    if (UsbHalSetDeviceAddress(address) != 0) {
        return -1;
    }

    g_usbState.deviceAddress = address;

    return 0;
}

/******************************************************************************
 * @brief     : Configure USB endpoint
 * @param[in] : config --Pointer to endpoint configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures endpoint type, direction, and packet size
 *****************************************************************************/
int32_t UsbConfigureEndpoint(UsbEndpointConfig_t* config)
{
    if (!g_usbState.isInitialized || config == NULL) {
        return -1;
    }

    if (config->endpointNumber >= USB_MAX_ENDPOINTS) {
        return -1;
    }

    if (config->endpointType > USB_EP_TYPE_ISOCHRONOUS) {
        return -1;
    }

    return UsbHalConfigureEndpoint(config);
}

/******************************************************************************
 * @brief     : Get current USB device state
 * @param[in] : None
 * @param[out]: None
 * @return    : Current state (USB_STATE_*)
 * @note      : Returns device state in the enumeration process
 *****************************************************************************/
int32_t UsbGetDeviceState(void)
{
    return g_usbState.deviceState;
}

/******************************************************************************
 * @brief     : Enumerate connected USB devices
 * @param[in] : None
 * @param[out]: None
 * @return    : Number of devices found, -1 if error
 * @note      : Host mode only - scans for connected devices
 *****************************************************************************/
int32_t UsbEnumerateDevices(void)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (g_usbState.mode != USB_MODE_HOST) {
        return -1;
    }

    /* Host mode enumeration - implementation depends on platform */
    return UsbHalEnumerateDevices();
}

/******************************************************************************
 * @brief     : Set USB speed mode
 * @param[in] : speed --Speed mode (USB_SPEED_FULL or USB_SPEED_HIGH)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before UsbInit
 *****************************************************************************/
int32_t UsbSetSpeed(uint8_t speed)
{
    if (g_usbState.isInitialized) {
        return -1;
    }

    if ((speed != USB_SPEED_FULL) && (speed != USB_SPEED_HIGH)) {
        return -1;
    }

    g_usbState.speed = speed;

    return 0;
}

/******************************************************************************
 * @brief     : Stall USB endpoint
 * @param[in] : endpoint --Endpoint number to stall
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Used to signal unsupported requests
 *****************************************************************************/
int32_t UsbStallEndpoint(uint8_t endpoint)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (endpoint >= USB_MAX_ENDPOINTS) {
        return -1;
    }

    return UsbHalStallEndpoint(endpoint);
}

/******************************************************************************
 * @brief     : Clear stall condition on endpoint
 * @param[in] : endpoint --Endpoint number to clear
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Clears previously set stall condition
 *****************************************************************************/
int32_t UsbClearStall(uint8_t endpoint)
{
    if (!g_usbState.isInitialized) {
        return -1;
    }

    if (endpoint >= USB_MAX_ENDPOINTS) {
        return -1;
    }

    return UsbHalClearStall(endpoint);
}

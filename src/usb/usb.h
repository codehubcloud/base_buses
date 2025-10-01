#ifndef USB_H
#define USB_H

#include <stdint.h>

/* USB Speed Modes */
#define USB_SPEED_FULL 0 /* Full-Speed: 12 Mbps */
#define USB_SPEED_HIGH 1 /* High-Speed: 480 Mbps */

/* USB Operating Modes */
#define USB_MODE_DEVICE 0 /* Device mode */
#define USB_MODE_HOST 1   /* Host mode */

/* USB Endpoint Types */
#define USB_EP_TYPE_CONTROL 0     /* Control endpoint */
#define USB_EP_TYPE_BULK 1        /* Bulk endpoint */
#define USB_EP_TYPE_INTERRUPT 2   /* Interrupt endpoint */
#define USB_EP_TYPE_ISOCHRONOUS 3 /* Isochronous endpoint */

/* USB Endpoint Directions */
#define USB_EP_DIR_OUT 0 /* OUT endpoint (host to device) */
#define USB_EP_DIR_IN 1  /* IN endpoint (device to host) */

/* USB Standard Request Codes */
#define USB_REQ_GET_STATUS 0x00
#define USB_REQ_CLEAR_FEATURE 0x01
#define USB_REQ_SET_FEATURE 0x03
#define USB_REQ_SET_ADDRESS 0x05
#define USB_REQ_GET_DESCRIPTOR 0x06
#define USB_REQ_SET_DESCRIPTOR 0x07
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_REQ_GET_INTERFACE 0x0A
#define USB_REQ_SET_INTERFACE 0x0B
#define USB_REQ_SYNCH_FRAME 0x0C

/* USB Descriptor Types */
#define USB_DESC_TYPE_DEVICE 0x01
#define USB_DESC_TYPE_CONFIGURATION 0x02
#define USB_DESC_TYPE_STRING 0x03
#define USB_DESC_TYPE_INTERFACE 0x04
#define USB_DESC_TYPE_ENDPOINT 0x05

/* USB Device States */
#define USB_STATE_DEFAULT 0
#define USB_STATE_ADDRESSED 1
#define USB_STATE_CONFIGURED 2
#define USB_STATE_SUSPENDED 3

/* USB Transfer Status */
#define USB_TRANSFER_IDLE 0
#define USB_TRANSFER_PENDING 1
#define USB_TRANSFER_COMPLETE 2
#define USB_TRANSFER_ERROR 3

/* USB Default Configuration */
#define USB_MAX_PACKET_SIZE 64
#define USB_MAX_ENDPOINTS 8
#define USB_SETUP_PACKET_SIZE 8

/* USB Setup Packet Structure */
typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} UsbSetupPacket_t;

/* USB Device Descriptor Structure */
typedef struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} UsbDeviceDescriptor_t;

/* USB Endpoint Configuration */
typedef struct {
    uint8_t endpointNumber;
    uint8_t endpointType;
    uint8_t direction;
    uint16_t maxPacketSize;
    uint8_t interval;
} UsbEndpointConfig_t;

/******************************************************************************
 * @brief     : Initialize USB peripheral
 * @param[in] : mode - USB operating mode (USB_MODE_DEVICE or USB_MODE_HOST)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures USB hardware for specified mode
 *****************************************************************************/
int32_t UsbInit(uint8_t mode);

/******************************************************************************
 * @brief     : Deinitialize USB peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables USB hardware and releases resources
 *****************************************************************************/
int32_t UsbDeinit(void);

/******************************************************************************
 * @brief     : Connect USB device to host
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables USB pull-up resistor (device mode only)
 *****************************************************************************/
int32_t UsbDeviceConnect(void);

/******************************************************************************
 * @brief     : Disconnect USB device from host
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables USB pull-up resistor (device mode only)
 *****************************************************************************/
int32_t UsbDeviceDisconnect(void);

/******************************************************************************
 * @brief     : Send data through USB endpoint
 * @param[in] : endpoint - Endpoint number (0-7), data - Pointer to data buffer, length - Number of bytes to send
 * @param[out]: None
 * @return    : Number of bytes sent, -1 if error
 * @note      : Blocking function for control/bulk transfers
 *****************************************************************************/
int32_t UsbSendData(uint8_t endpoint, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from USB endpoint
 * @param[in] : endpoint - Endpoint number (0-7), maxLength - Maximum number of bytes to receive
 * @param[out]: data - Pointer to buffer for received data
 * @return    : Number of bytes received, -1 if error
 * @note      : Blocking function with timeout
 *****************************************************************************/
int32_t UsbReceiveData(uint8_t endpoint, uint8_t* data, uint16_t maxLength);

/******************************************************************************
 * @brief     : Process USB control transfer setup packet
 * @param[in] : setupPacket - Pointer to 8-byte setup packet data
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Handles standard USB device requests
 *****************************************************************************/
int32_t UsbSetupControlTransfer(uint8_t* setupPacket);

/******************************************************************************
 * @brief     : Get USB device descriptor
 * @param[in] : maxLength - Maximum descriptor size
 * @param[out]: descriptor - Pointer to buffer for device descriptor
 * @return    : Actual descriptor size, -1 if error
 * @note      : Returns the device descriptor structure
 *****************************************************************************/
int32_t UsbGetDeviceDescriptor(uint8_t* descriptor, uint16_t maxLength);

/******************************************************************************
 * @brief     : Set USB device address
 * @param[in] : address - Device address (0-127)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Used during enumeration process
 *****************************************************************************/
int32_t UsbSetAddress(uint8_t address);

/******************************************************************************
 * @brief     : Configure USB endpoint
 * @param[in] : config - Pointer to endpoint configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures endpoint type, direction, and packet size
 *****************************************************************************/
int32_t UsbConfigureEndpoint(UsbEndpointConfig_t* config);

/******************************************************************************
 * @brief     : Get current USB device state
 * @param[in] : None
 * @param[out]: None
 * @return    : Current state (USB_STATE_*)
 * @note      : Returns device state in the enumeration process
 *****************************************************************************/
int32_t UsbGetDeviceState(void);

/******************************************************************************
 * @brief     : Enumerate connected USB devices
 * @param[in] : None
 * @param[out]: None
 * @return    : Number of devices found, -1 if error
 * @note      : Host mode only - scans for connected devices
 *****************************************************************************/
int32_t UsbEnumerateDevices(void);

/******************************************************************************
 * @brief     : Set USB speed mode
 * @param[in] : speed - Speed mode (USB_SPEED_FULL or USB_SPEED_HIGH)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before UsbInit
 *****************************************************************************/
int32_t UsbSetSpeed(uint8_t speed);

/******************************************************************************
 * @brief     : Stall USB endpoint
 * @param[in] : endpoint - Endpoint number to stall
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Used to signal unsupported requests
 *****************************************************************************/
int32_t UsbStallEndpoint(uint8_t endpoint);

/******************************************************************************
 * @brief     : Clear stall condition on endpoint
 * @param[in] : endpoint - Endpoint number to clear
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Clears previously set stall condition
 *****************************************************************************/
int32_t UsbClearStall(uint8_t endpoint);

#endif // USB_H

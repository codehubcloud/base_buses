#ifndef USB_HAL_H
#define USB_HAL_H

#include <stdint.h>
#include "usb.h"

/******************************************************************************
 * @brief     : Enable USB peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalEnableClock(void);

/******************************************************************************
 * @brief     : Disable USB peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalDisableClock(void);

/******************************************************************************
 * @brief     : Configure USB GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalConfigureGpio(void);

/******************************************************************************
 * @brief     : Initialize USB controller hardware
 * @param[in] : mode - USB mode (device or host), speed - USB speed
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalInitController(uint8_t mode, uint8_t speed);

/******************************************************************************
 * @brief     : Deinitialize USB controller hardware
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalDeinitController(void);

/******************************************************************************
 * @brief     : Connect USB device (enable pull-up)
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalDeviceConnect(void);

/******************************************************************************
 * @brief     : Disconnect USB device (disable pull-up)
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalDeviceDisconnect(void);

/******************************************************************************
 * @brief     : Configure USB endpoint
 * @param[in] : config - Pointer to endpoint configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalConfigureEndpoint(UsbEndpointConfig_t* config);

/******************************************************************************
 * @brief     : Set USB device address
 * @param[in] : address - Device address (0-127)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalSetDeviceAddress(uint8_t address);

/******************************************************************************
 * @brief     : Transmit data on endpoint
 * @param[in] : endpoint - Endpoint number, data - Data buffer, length - Data length
 * @param[out]: None
 * @return    : Number of bytes transmitted, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalTransmitData(uint8_t endpoint, uint8_t* data, uint16_t length);

/******************************************************************************
 * @brief     : Receive data from endpoint
 * @param[in] : endpoint - Endpoint number, maxLength - Maximum buffer size
 * @param[out]: data - Data buffer for received data
 * @return    : Number of bytes received, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalReceiveData(uint8_t endpoint, uint8_t* data, uint16_t maxLength);

/******************************************************************************
 * @brief     : Check if endpoint is ready for transfer
 * @param[in] : endpoint - Endpoint number (with direction bit)
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalIsEndpointReady(uint8_t endpoint);

/******************************************************************************
 * @brief     : Check if data is available on endpoint
 * @param[in] : endpoint - Endpoint number
 * @param[out]: None
 * @return    : 1 if data available, 0 if not
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalIsDataAvailable(uint8_t endpoint);

/******************************************************************************
 * @brief     : Stall endpoint
 * @param[in] : endpoint - Endpoint number to stall
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalStallEndpoint(uint8_t endpoint);

/******************************************************************************
 * @brief     : Clear stall condition on endpoint
 * @param[in] : endpoint - Endpoint number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalClearStall(uint8_t endpoint);

/******************************************************************************
 * @brief     : Enumerate connected USB devices (host mode)
 * @param[in] : None
 * @param[out]: None
 * @return    : Number of devices found, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalEnumerateDevices(void);

/******************************************************************************
 * @brief     : Enable USB interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalEnableInterrupts(void);

/******************************************************************************
 * @brief     : Disable USB interrupts
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalDisableInterrupts(void);

/******************************************************************************
 * @brief     : Reset USB peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalReset(void);

/******************************************************************************
 * @brief     : Flush TX FIFO for endpoint
 * @param[in] : endpoint - Endpoint number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalFlushTxFifo(uint8_t endpoint);

/******************************************************************************
 * @brief     : Flush RX FIFO
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *****************************************************************************/
int32_t UsbHalFlushRxFifo(void);

#endif // USB_HAL_H

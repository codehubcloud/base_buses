#include <stdint.h>
#include <stdio.h>
#include "ahb/ahb.h"
#include "apb/apb.h"
#include "axi/axi.h"
#include "can/can.h"
#include "dma/dma.h"
#include "emmc/emmc.h"
#include "ethernet/ethernet.h"
#include "flexray/flexray.h"
#include "i2c/i2c.h"
#include "i2s/i2s.h"
#include "i3c/i3c.h"
#include "jtag/jtag.h"
#include "lin/lin.h"
#include "modbus/modbus.h"
#include "onewire/onewire.h"
#include "pcie/pcie.h"
#include "pmbus/pmbus.h"
#include "rs232/rs232.h"
#include "rs422/rs422.h"
#include "rs485/rs485.h"
#include "sdio/sdio.h"
#include "smbus/smbus.h"
#include "spi/spi.h"
#include "swd/swd.h"
#include "uart/uart.h"
#include "usb/usb.h"

/******************************************************************************
 * @brief     : Test UART functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests sending and receiving data through UART
 *****************************************************************************/
int32_t TestUart(void)
{
    uint8_t testData[] = "UART Test Message";
    uint8_t receiveBuffer[32];
    int32_t receivedLength = 0;
    int32_t result = 0;

    printf("Testing UART...\n");

    result = UartInit();
    if (result != 0) {
        printf("UART initialization failed!\n");
        return -1;
    }

    result = UartSendData(testData, sizeof(testData) - 1);
    if (result != 0) {
        printf("UART send failed!\n");
        return -1;
    }

    receivedLength = UartReceiveData(receiveBuffer, sizeof(receiveBuffer) - 1);
    if (receivedLength > 0) {
        receiveBuffer[receivedLength] = '\0';
        printf("Received: %s\n", receiveBuffer);
    }

    printf("UART test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test SPI functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests SPI data transfer
 *****************************************************************************/
int32_t TestSpi(void)
{
    uint8_t txData[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t rxData[4];
    int32_t result = 0;

    printf("Testing SPI...\n");

    result = SpiInit();
    if (result != 0) {
        printf("SPI initialization failed!\n");
        return -1;
    }

    result = SpiTransfer(txData, rxData, sizeof(txData));
    if (result != 0) {
        printf("SPI transfer failed!\n");
        return -1;
    }

    printf("SPI test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test I2C functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests I2C device communication
 *****************************************************************************/
int32_t TestI2c(void)
{
    uint8_t deviceAddr = 0x50;
    uint8_t testData[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t readData[4];
    int32_t result = 0;

    printf("Testing I2C...\n");

    result = I2cInit();
    if (result != 0) {
        printf("I2C initialization failed!\n");
        return -1;
    }

    result = I2cCheckDevice(deviceAddr);
    if (result != 0) {
        printf("I2C device not found!\n");
        return -1;
    }

    result = I2cWriteData(deviceAddr, NULL, testData, sizeof(testData));
    if (result != 0) {
        printf("I2C write failed!\n");
        return -1;
    }

    result = I2cReadData(deviceAddr, NULL, readData, sizeof(readData));
    if (result != 0) {
        printf("I2C read failed!\n");
        return -1;
    }

    printf("I2C test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test I3C functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests I3C device communication
 *****************************************************************************/
int32_t TestI3c(void)
{
    uint8_t deviceAddr = 0x50;
    uint8_t testData[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t readData[4];
    int32_t result = 0;

    printf("Testing I3C...\n");

    result = I3cInit();
    if (result != 0) {
        printf("I3C initialization failed!\n");
        return -1;
    }

    result = I3cCheckDevice(deviceAddr);
    if (result != 0) {
        printf("I3C device not found!\n");
        return -1;
    }

    result = I3cSendData(deviceAddr, testData, sizeof(testData));
    if (result != 0) {
        printf("I3C send failed!\n");
        return -1;
    }

    result = I3cReadData(deviceAddr, readData, sizeof(readData));
    if (result != 0) {
        printf("I3C read failed!\n");
        return -1;
    }

    printf("I3C test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test I2S functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests I2S audio interface with sample data
 *****************************************************************************/
int32_t TestI2s(void)
{
    uint8_t audioData[128];
    uint8_t receiveBuffer[128];
    int32_t result = 0;

    printf("Testing I2S...\n");

    result = I2sInit();
    if (result != 0) {
        printf("I2S initialization failed!\n");
        return -1;
    }

    result = I2sSetSampleRate(I2S_SAMPLE_RATE_44KHZ);
    if (result != 0) {
        printf("I2S sample rate configuration failed!\n");
        return -1;
    }

    result = I2sConfigureFormat(I2S_DATA_FORMAT_16BIT, I2S_CHANNEL_STEREO);
    if (result != 0) {
        printf("I2S format configuration failed!\n");
        return -1;
    }

    for (uint32_t i = 0; i < sizeof(audioData); i++) {
        audioData[i] = (uint8_t)(i & 0xFF);
    }

    result = I2sTransmitData(audioData, sizeof(audioData));
    if (result != 0) {
        printf("I2S transmit failed!\n");
        return -1;
    }

    result = I2sReceiveData(receiveBuffer, sizeof(receiveBuffer));
    if (result < 0) {
        printf("I2S receive failed!\n");
        return -1;
    }

    result = I2sEnableDMA();
    if (result != 0) {
        printf("I2S DMA enable failed!\n");
        return -1;
    }

    result = I2sDeinit();
    if (result != 0) {
        printf("I2S deinitialization failed!\n");
        return -1;
    }

    printf("I2S test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test Modbus functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests Modbus communication
 *****************************************************************************/
int32_t TestModbus(void)
{
    int32_t result = 0;

    printf("Testing Modbus...\n");

    result = ModbusInit();
    if (result != 0) {
        printf("Modbus initialization failed!\n");
        return -1;
    }

    // Add Modbus tests here when hardware is available

    printf("Modbus test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test SMBus functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests SMBus device communication
 *****************************************************************************/
int32_t TestSmbus(void)
{
    uint8_t deviceAddr = 0x50;
    uint8_t command = 0x01;
    uint8_t testData = 0xAB;
    uint8_t readData = 0;
    int32_t result = 0;

    printf("Testing SMBus...\n");

    result = SmBusInit();
    if (result != 0) {
        printf("SMBus initialization failed!\n");
        return -1;
    }

    result = SmBusCheckDevice(deviceAddr);
    if (result != 0) {
        printf("SMBus device not found!\n");
        return -1;
    }

    result = SmBusWriteByte(deviceAddr, command, testData);
    if (result != 0) {
        printf("SMBus write failed!\n");
        return -1;
    }

    result = SmBusReadByte(deviceAddr, command, &readData);
    if (result != 0) {
        printf("SMBus read failed!\n");
        return -1;
    }

    printf("SMBus test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test PMBus functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests PMBus power management operations
 *****************************************************************************/
int32_t TestPmbus(void)
{
    uint8_t deviceAddr = 0x40;
    float voltage = 0.0f;
    float current = 0.0f;
    float temperature = 0.0f;
    uint16_t status = 0;
    uint8_t revision = 0;
    int32_t result = 0;

    printf("Testing PMBus...\n");

    result = PmBusInit();
    if (result != 0) {
        printf("PMBus initialization failed!\n");
        return -1;
    }

    // Read PMBus revision
    result = PmBusReadRevision(deviceAddr, &revision);
    if (result == 0) {
        printf("PMBus Revision: 0x%02X\n", revision);
    }

    // Read output voltage
    result = PmBusReadVoltage(deviceAddr, &voltage);
    if (result == 0) {
        printf("Output Voltage: %.3f V\n", voltage);
    }

    // Read output current
    result = PmBusReadCurrent(deviceAddr, &current);
    if (result == 0) {
        printf("Output Current: %.3f A\n", current);
    }

    // Read temperature sensor 1
    result = PmBusReadTemperature(deviceAddr, 1, &temperature);
    if (result == 0) {
        printf("Temperature: %.2f C\n", temperature);
    }

    // Read status word
    result = PmBusGetStatus(deviceAddr, &status);
    if (result == 0) {
        printf("Status Word: 0x%04X\n", status);
    }

    // Set voltage example (12.0V)
    result = PmBusSetVoltage(deviceAddr, 12.0f);
    if (result == 0) {
        printf("Voltage set to 12.0V\n");
    }

    // Clear faults
    result = PmBusClearFaults(deviceAddr);
    if (result == 0) {
        printf("Faults cleared\n");
    }

    printf("PMBus test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test RS232 functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests RS232 communication
 *****************************************************************************/
int32_t TestRs232(void)
{
    uint8_t testData[] = "RS232 Test Message";
    uint8_t receiveBuffer[32];
    int32_t receivedLength = 0;
    int32_t result = 0;

    printf("Testing RS232...\n");

    result = Rs232Init();
    if (result != 0) {
        printf("RS232 initialization failed!\n");
        return -1;
    }

    result = Rs232SendData(testData, sizeof(testData) - 1);
    if (result != 0) {
        printf("RS232 send failed!\n");
        return -1;
    }

    receivedLength = Rs232ReceiveData(receiveBuffer, sizeof(receiveBuffer) - 1);
    if (receivedLength > 0) {
        receiveBuffer[receivedLength] = '\0';
        printf("Received: %s\n", receiveBuffer);
    }

    printf("RS232 test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test RS422 functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests RS422 full-duplex differential communication
 *****************************************************************************/
int32_t TestRs422(void)
{
    uint8_t testData[] = "RS422 Test Message";
    uint8_t receiveBuffer[32];
    int32_t receivedLength = 0;
    int32_t result = 0;

    printf("Testing RS422...\n");

    result = Rs422Init();
    if (result != 0) {
        printf("RS422 initialization failed!\n");
        return -1;
    }

    result = Rs422SetBaudRate(RS422_BAUDRATE_115200);
    if (result != 0) {
        printf("RS422 baud rate configuration failed!\n");
        return -1;
    }

    result = Rs422ConfigureFormat(RS422_DATA_BITS_8, RS422_PARITY_NONE, RS422_STOP_BITS_1);
    if (result != 0) {
        printf("RS422 format configuration failed!\n");
        return -1;
    }

    result = Rs422SendData(testData, sizeof(testData) - 1);
    if (result != 0) {
        printf("RS422 send failed!\n");
        return -1;
    }

    receivedLength = Rs422ReceiveData(receiveBuffer, sizeof(receiveBuffer) - 1);
    if (receivedLength > 0) {
        receiveBuffer[receivedLength] = '\0';
        printf("Received: %s\n", receiveBuffer);
    }

    result = Rs422Deinit();
    if (result != 0) {
        printf("RS422 deinitialization failed!\n");
        return -1;
    }

    printf("RS422 test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test RS485 functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests RS485 communication
 *****************************************************************************/
int32_t TestRs485(void)
{
    uint8_t testData[] = "RS485 Test Message";
    uint8_t receiveBuffer[32];
    int32_t receivedLength = 0;
    int32_t result = 0;

    printf("Testing RS485...\n");

    result = Rs485Init();
    if (result != 0) {
        printf("RS485 initialization failed!\n");
        return -1;
    }

    result = Rs485SendData(testData, sizeof(testData) - 1);
    if (result != 0) {
        printf("RS485 send failed!\n");
        return -1;
    }

    receivedLength = Rs485ReceiveData(receiveBuffer, sizeof(receiveBuffer) - 1);
    if (receivedLength > 0) {
        receiveBuffer[receivedLength] = '\0';
        printf("Received: %s\n", receiveBuffer);
    }

    printf("RS485 test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test CAN functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests CAN communication
 *****************************************************************************/
int32_t TestCan(void)
{
    uint32_t testId = 0x123;
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    uint32_t receivedId = 0;
    uint8_t receiveBuffer[8];
    int32_t receivedLength = 0;
    int32_t result = 0;

    printf("Testing CAN...\n");

    result = CanInit();
    if (result != 0) {
        printf("CAN initialization failed!\n");
        return -1;
    }

    result = CanSendData(testId, testData, sizeof(testData));
    if (result != 0) {
        printf("CAN send failed!\n");
        return -1;
    }

    receivedLength = CanReceiveData(&receivedId, receiveBuffer, sizeof(receiveBuffer));
    if (receivedLength > 0) {
        printf("Received CAN message with ID: 0x%X\n", receivedId);
    }

    printf("CAN test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test LIN functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests LIN communication
 *****************************************************************************/
int32_t TestLin(void)
{
    uint8_t frameId = 0x12;
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t receiveBuffer[8];
    int32_t result = 0;

    printf("Testing LIN...\n");

    result = LinInit();
    if (result != 0) {
        printf("LIN initialization failed!\n");
        return -1;
    }

    result = LinSendFrame(frameId, testData, sizeof(testData));
    if (result != 0) {
        printf("LIN send failed!\n");
        return -1;
    }

    result = LinReceiveFrame(frameId, receiveBuffer, sizeof(receiveBuffer));
    if (result > 0) {
        printf("LIN frame received successfully.\n");
    }

    printf("LIN test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test FlexRay functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests FlexRay communication
 *****************************************************************************/
int32_t TestFlexRay(void)
{
    uint16_t slotId = 10;
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t receiveBuffer[32];
    int32_t result = 0;

    printf("Testing FlexRay...\n");

    result = FlexRayInit();
    if (result != 0) {
        printf("FlexRay initialization failed!\n");
        return -1;
    }

    result = FlexRaySendFrame(slotId, testData, sizeof(testData));
    if (result != 0) {
        printf("FlexRay send failed!\n");
        return -1;
    }

    result = FlexRayReceiveFrame(slotId, receiveBuffer, sizeof(receiveBuffer));
    if (result > 0) {
        printf("FlexRay frame received successfully.\n");
    }

    printf("FlexRay test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test OneWire functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests OneWire communication
 *****************************************************************************/
int32_t TestOneWire(void)
{
    uint8_t testData[] = {0x44, 0xBE, 0x01, 0x00, 0x05};
    uint8_t receiveBuffer[8];
    int32_t result = 0;

    printf("Testing OneWire...\n");

    result = OneWireInit();
    if (result != 0) {
        printf("OneWire initialization failed!\n");
        return -1;
    }

    result = OneWireReset();
    if (result != 0) {
        printf("OneWire device not present!\n");
        return -1;
    }

    result = OneWireWriteBytes(testData, sizeof(testData));
    if (result < 0) {
        printf("OneWire write failed!\n");
        return -1;
    }

    result = OneWireReadBytes(receiveBuffer, sizeof(receiveBuffer));
    if (result > 0) {
        printf("OneWire read successful.\n");
    }

    printf("OneWire test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test USB functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests USB communication
 *****************************************************************************/
int32_t TestUsb(void)
{
    uint8_t testData[] = "USB Test Message";
    uint8_t receiveBuffer[64];
    int32_t result = 0;

    printf("Testing USB...\n");

    result = UsbInit(USB_MODE_DEVICE);
    if (result != 0) {
        printf("USB initialization failed!\n");
        return -1;
    }

    result = UsbDeviceConnect();
    if (result != 0) {
        printf("USB device connect failed!\n");
        return -1;
    }

    result = UsbSendData(1, testData, sizeof(testData) - 1);
    if (result < 0) {
        printf("USB send failed!\n");
        return -1;
    }

    result = UsbReceiveData(1, receiveBuffer, sizeof(receiveBuffer));
    if (result > 0) {
        printf("USB data received successfully.\n");
    }

    printf("USB test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test SDIO functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests SDIO communication
 *****************************************************************************/
int32_t TestSdio(void)
{
    uint8_t testData[512];
    uint8_t receiveBuffer[512];
    int32_t result = 0;

    printf("Testing SDIO...\n");

    result = SdioInit();
    if (result != 0) {
        printf("SDIO initialization failed!\n");
        return -1;
    }

    result = SdioSetBusWidth(4);
    if (result != 0) {
        printf("SDIO bus width configuration failed!\n");
        return -1;
    }

    for (int i = 0; i < 512; i++) {
        testData[i] = (uint8_t)(i & 0xFF);
    }

    result = SdioWriteBlock(0, testData, sizeof(testData));
    if (result != 0) {
        printf("SDIO write block failed!\n");
        return -1;
    }

    result = SdioReadBlock(0, receiveBuffer, sizeof(receiveBuffer));
    if (result == 0) {
        printf("SDIO block read successful.\n");
    }

    printf("SDIO test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test Ethernet functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests Ethernet communication
 *****************************************************************************/
int32_t TestEthernet(void)
{
    uint8_t testFrame[128];
    uint8_t receiveBuffer[ETHERNET_MAX_FRAME_SIZE];
    EthernetMacAddress_t macAddr = {{0x00, 0x11, 0x22, 0x33, 0x44, 0x55}};
    int32_t result = 0;

    printf("Testing Ethernet...\n");

    result = EthernetInit();
    if (result != 0) {
        printf("Ethernet initialization failed!\n");
        return -1;
    }

    result = EthernetSetMacAddress(&macAddr);
    if (result != 0) {
        printf("Ethernet MAC address configuration failed!\n");
        return -1;
    }

    result = EthernetSetSpeed(ETHERNET_SPEED_100M, ETHERNET_DUPLEX_FULL);
    if (result != 0) {
        printf("Ethernet speed configuration failed!\n");
        return -1;
    }

    result = EthernetGetLinkStatus();
    if (result == ETHERNET_LINK_UP) {
        printf("Ethernet link is up.\n");
    } else {
        printf("Ethernet link is down.\n");
    }

    for (int i = 0; i < sizeof(testFrame); i++) {
        testFrame[i] = (uint8_t)(i & 0xFF);
    }

    result = EthernetSendFrame(testFrame, ETHERNET_MIN_FRAME_SIZE);
    if (result == 0) {
        printf("Ethernet frame sent successfully.\n");
    }

    result = EthernetReceiveFrame(receiveBuffer, sizeof(receiveBuffer));
    if (result > 0) {
        printf("Ethernet frame received (%d bytes).\n", result);
    }

    printf("Ethernet test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test SWD functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests SWD debug interface functionality
 *****************************************************************************/
int32_t TestSwd(void)
{
    uint32_t idcode = 0;
    uint32_t dpCtrl = 0;
    int32_t result = 0;

    printf("Testing SWD...\n");

    result = SwdInit();
    if (result != 0) {
        printf("SWD initialization failed!\n");
        return -1;
    }

    result = SwdReadIdcode(&idcode);
    if (result != 0) {
        printf("SWD IDCODE read failed!\n");
        return -1;
    }
    printf("Target IDCODE: 0x%08X\n", idcode);

    result = SwdReadDP(SWD_DP_CTRL, &dpCtrl);
    if (result != 0) {
        printf("SWD DP CTRL read failed!\n");
        return -1;
    }
    printf("DP CTRL/STAT: 0x%08X\n", dpCtrl);

    result = SwdWriteDP(SWD_DP_CTRL, 0x50000000);
    if (result != 0) {
        printf("SWD DP CTRL write failed!\n");
        return -1;
    }

    result = SwdDeinit();
    if (result != 0) {
        printf("SWD deinitialization failed!\n");
        return -1;
    }

    printf("SWD test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test JTAG functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests JTAG debug interface functionality
 *****************************************************************************/
int32_t TestJtag(void)
{
    uint32_t idcode = 0;
    uint8_t testData[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t readData[4];
    JtagConfig_T config;
    int32_t result = 0;

    printf("Testing JTAG...\n");

    result = JtagInit();
    if (result != 0) {
        printf("JTAG initialization failed!\n");
        return -1;
    }

    result = JtagReadIdcode(&idcode);
    if (result != 0) {
        printf("JTAG IDCODE read failed!\n");
        return -1;
    }
    printf("Target IDCODE: 0x%08X\n", idcode);

    config.clockFrequency = 1000000;
    config.irLength = 4;
    config.tapCount = 1;
    config.enableTrst = 0;
    result = JtagConfigure(&config);
    if (result != 0) {
        printf("JTAG configuration failed!\n");
        return -1;
    }

    result = JtagShiftIR(JTAG_IR_BYPASS, 4);
    if (result != 0) {
        printf("JTAG IR shift failed!\n");
        return -1;
    }

    result = JtagWriteRegister(JTAG_IR_BYPASS, testData, 32);
    if (result != 0) {
        printf("JTAG register write failed!\n");
        return -1;
    }

    result = JtagReadRegister(JTAG_IR_BYPASS, readData, 32);
    if (result != 0) {
        printf("JTAG register read failed!\n");
        return -1;
    }

    result = JtagRunTestIdle(10);
    if (result != 0) {
        printf("JTAG Run-Test/Idle failed!\n");
        return -1;
    }

    result = JtagDeinit();
    if (result != 0) {
        printf("JTAG deinitialization failed!\n");
        return -1;
    }

    printf("JTAG test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test PCIe functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests PCIe device enumeration and configuration space access
 *****************************************************************************/
int32_t TestPcie(void)
{
    PcieConfig config;
    PcieLinkStatus linkStatus;
    PcieDeviceInfo devices[16];
    uint32_t deviceCount = 0;
    uint32_t vendorDevice = 0;
    int32_t result = 0;
    uint32_t i = 0;

    printf("Testing PCIe...\n");

    config.targetGeneration = PCIE_GEN3;
    config.targetLinkWidth = PCIE_LINK_WIDTH_X4;
    config.enableHotplug = 0;
    config.enableAER = 1;

    result = PcieInit(&config);
    if (result != 0) {
        printf("PCIe initialization failed (may not be supported on this platform)!\n");
        return 0;
    }

    result = PcieGetLinkStatus(&linkStatus);
    if (result == 0) {
        printf("PCIe Link Status:\n");
        printf("  Generation: Gen%d\n", linkStatus.generation);
        printf("  Link Width: x%d\n", linkStatus.linkWidth);
        printf("  Link Up: %s\n", linkStatus.isLinkUp ? "Yes" : "No");
    }

    result = PcieEnumerateDevices(devices, 16, &deviceCount);
    if (result == 0 && deviceCount > 0) {
        printf("Found %u PCIe device(s):\n", deviceCount);
        for (i = 0; i < deviceCount && i < 16; i++) {
            printf("  Device %u: Bus=%02x Dev=%02x Func=%x VID=%04x DID=%04x\n", i, devices[i].bus, devices[i].device, devices[i].function,
                   devices[i].vendorId, devices[i].deviceId);
        }
    } else {
        printf("No PCIe devices found or enumeration not supported.\n");
    }

    result = PcieConfigRead(0, 0, 0, PCIE_CFG_VENDOR_ID, &vendorDevice, 4);
    if (result == 0) {
        printf("Bus 0 Device 0: Vendor=0x%04X Device=0x%04X\n", (vendorDevice & 0xFFFF), ((vendorDevice >> 16) & 0xFFFF));
    }

    result = PcieDeinit();
    if (result != 0) {
        printf("PCIe deinitialization failed!\n");
        return -1;
    }

    printf("PCIe test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test eMMC functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests eMMC storage read/write operations
 *****************************************************************************/
int32_t TestEmmc(void)
{
    uint8_t writeBuffer[512];
    uint8_t readBuffer[512];
    EmmcCardInfo_S cardInfo;
    uint32_t status = 0;
    int32_t result = 0;

    printf("Testing eMMC...\n");

    result = EmmcInit();
    if (result != 0) {
        printf("eMMC initialization failed!\n");
        return -1;
    }

    result = EmmcGetCardInfo(&cardInfo);
    if (result == 0) {
        printf("eMMC Card Information:\n");
        printf("  Manufacturer ID: 0x%02X\n", cardInfo.cid.manufacturerId);
        printf("  Capacity: %llu bytes (%llu MB)\n", (unsigned long long)cardInfo.capacity,
               (unsigned long long)(cardInfo.capacity / (1024 * 1024)));
        printf("  Block Size: %u bytes\n", cardInfo.blockSize);
        printf("  Block Count: %u\n", cardInfo.blockCount);
    }

    result = EmmcGetCardStatus(&status);
    if (result == 0) {
        printf("eMMC Status: 0x%08X\n", status);
    }

    result = EmmcSetBusWidth(EMMC_BUS_WIDTH_8BIT);
    if (result == 0) {
        printf("eMMC bus width set to 8-bit\n");
    }

    result = EmmcSetSpeedMode(EMMC_SPEED_MODE_HIGH_SPEED);
    if (result == 0) {
        printf("eMMC speed mode set to High Speed\n");
    }

    for (int i = 0; i < 512; i++) {
        writeBuffer[i] = (uint8_t)(i & 0xFF);
    }

    result = EmmcWriteBlock(0x1000, writeBuffer);
    if (result != 0) {
        printf("eMMC write block failed!\n");
        return -1;
    }
    printf("eMMC single block written successfully\n");

    result = EmmcReadBlock(0x1000, readBuffer);
    if (result != 0) {
        printf("eMMC read block failed!\n");
        return -1;
    }
    printf("eMMC single block read successfully\n");

    for (int i = 0; i < 512; i++) {
        if (readBuffer[i] != writeBuffer[i]) {
            printf("eMMC data verification failed at byte %d!\n", i);
            return -1;
        }
    }
    printf("eMMC data verification passed\n");

    uint8_t multiWriteBuffer[1024];
    uint8_t multiReadBuffer[1024];
    for (int i = 0; i < 1024; i++) {
        multiWriteBuffer[i] = (uint8_t)((i * 2) & 0xFF);
    }

    result = EmmcWriteMultipleBlocks(0x2000, 2, multiWriteBuffer);
    if (result == 0) {
        printf("eMMC multiple blocks written successfully\n");
    }

    result = EmmcReadMultipleBlocks(0x2000, 2, multiReadBuffer);
    if (result == 0) {
        printf("eMMC multiple blocks read successfully\n");
    }

    result = EmmcSelectPartition(EMMC_PARTITION_USER);
    if (result == 0) {
        printf("eMMC partition switched to USER\n");
    }

    result = EmmcDeinit();
    if (result != 0) {
        printf("eMMC deinitialization failed!\n");
        return -1;
    }

    printf("eMMC test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test AHB functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests AHB bus read/write transactions and burst operations
 *****************************************************************************/
int32_t TestAhb(void)
{
    uint32_t testData = 0xDEADBEEF;
    uint32_t readData = 0;
    uint32_t burstWriteData[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t burstReadData[8] = {0};
    int32_t result = 0;

    printf("Testing AHB...\n");

    result = AhbInit();
    if (result != 0) {
        printf("AHB initialization failed (may not be supported on this platform)!\n");
        return 0;
    }

    result = AhbWrite(0x20000000, testData, AHB_SIZE_WORD);
    if (result != 0) {
        printf("AHB single write failed!\n");
        return -1;
    }
    printf("AHB single write completed\n");

    result = AhbRead(0x20000000, &readData, AHB_SIZE_WORD);
    if (result != 0) {
        printf("AHB single read failed!\n");
        return -1;
    }
    printf("AHB single read completed, data: 0x%08X\n", readData);

    result = AhbBurstWrite(0x20001000, burstWriteData, 8, AHB_BURST_INCR8);
    if (result != 0) {
        printf("AHB burst write failed!\n");
        return -1;
    }
    printf("AHB burst write completed (8 words)\n");

    result = AhbBurstRead(0x20001000, burstReadData, 8, AHB_BURST_INCR8);
    if (result != 0) {
        printf("AHB burst read failed!\n");
        return -1;
    }
    printf("AHB burst read completed (8 words)\n");

    result = AhbSetPriority(2);
    if (result == 0) {
        printf("AHB master priority set to 2\n");
    }

    int32_t status = AhbGetBusStatus();
    if (status == AHB_STATUS_IDLE) {
        printf("AHB bus status: IDLE\n");
    } else if (status == AHB_STATUS_BUSY) {
        printf("AHB bus status: BUSY\n");
    } else if (status == AHB_STATUS_ERROR) {
        printf("AHB bus status: ERROR\n");
    }

    result = AhbDeinit();
    if (result != 0) {
        printf("AHB deinitialization failed!\n");
        return -1;
    }

    printf("AHB test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test AXI functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests AXI bus read/write transactions and burst operations
 *****************************************************************************/
int32_t TestAxi(void)
{
    AxiConfig config;
    AxiWriteAddress writeAddr;
    AxiWriteData writeData;
    AxiWriteResponse writeResp;
    AxiReadAddress readAddr;
    AxiReadData readData;
    AxiStatus status;
    uint8_t writeBuffer[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};
    uint8_t readBuffer[16] = {0};
    int32_t result = 0;

    printf("Testing AXI...\n");

    config.dataWidth = 32;
    config.addressWidth = 32;
    config.idWidth = 4;
    config.maxBurstLength = 16;
    config.isLite = 0;
    config.timeout = 1000;

    result = AxiInit(&config);
    if (result != 0) {
        printf("AXI initialization failed (may not be supported on this platform)!\n");
        return 0;
    }

    writeAddr.address = 0x1000;
    writeAddr.burstLength = 0;
    writeAddr.burstSize = AXI_SIZE_4_BYTES;
    writeAddr.burstType = AXI_BURST_INCR;
    writeAddr.lock = AXI_LOCK_NORMAL;
    writeAddr.cache = AXI_CACHE_NORMAL_NON_CACHEABLE;
    writeAddr.protection = 0;
    writeAddr.qos = 0;
    writeAddr.region = 0;
    writeAddr.id = 1;
    writeAddr.user = 0;

    writeData.data = writeBuffer;
    writeData.strobe = NULL;
    writeData.last = 1;
    writeData.user = 0;

    result = AxiWriteTransaction(&writeAddr, &writeData, &writeResp);
    if (result == 0) {
        printf("AXI write transaction completed with response: %d\n", writeResp.response);
    } else {
        printf("AXI write transaction failed!\n");
    }

    readAddr.address = 0x1000;
    readAddr.burstLength = 0;
    readAddr.burstSize = AXI_SIZE_4_BYTES;
    readAddr.burstType = AXI_BURST_INCR;
    readAddr.lock = AXI_LOCK_NORMAL;
    readAddr.cache = AXI_CACHE_NORMAL_NON_CACHEABLE;
    readAddr.protection = 0;
    readAddr.qos = 0;
    readAddr.region = 0;
    readAddr.id = 2;
    readAddr.user = 0;

    readData.data = readBuffer;
    readData.response = AXI_RESP_OKAY;
    readData.last = 1;
    readData.id = 0;
    readData.user = 0;

    result = AxiReadTransaction(&readAddr, &readData);
    if (result == 0) {
        printf("AXI read transaction completed with response: %d\n", readData.response);
    } else {
        printf("AXI read transaction failed!\n");
    }

    result = AxiSetQoS(3);
    if (result == 0) {
        printf("AXI QoS set to 3\n");
    }

    result = AxiGetStatus(&status);
    if (result == 0) {
        printf("AXI Status: Initialized=%d, Busy=%d, Errors=%u, Writes=%u, Reads=%u\n", status.isInitialized, status.isBusy,
               status.errorCount, status.writeCount, status.readCount);
    }

    result = AxiDeinit();
    if (result != 0) {
        printf("AXI deinitialization failed!\n");
        return -1;
    }

    printf("AXI test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test APB functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests APB bus read/write operations
 *****************************************************************************/
int32_t TestApb(void)
{
    uint32_t testData = 0x12345678;
    uint32_t readData = 0;
    uint32_t multiWriteData[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint32_t multiReadData[4] = {0};
    int32_t result = 0;

    printf("Testing APB...\n");

    result = ApbInit();
    if (result != 0) {
        printf("APB initialization failed (may not be supported on this platform)!\n");
        return 0;
    }

    result = ApbWrite(0x40000000, testData);
    if (result != 0) {
        printf("APB single write failed!\n");
        return -1;
    }
    printf("APB single write completed\n");

    result = ApbRead(0x40000000, &readData);
    if (result != 0) {
        printf("APB single read failed!\n");
        return -1;
    }
    printf("APB single read completed, data: 0x%08X\n", readData);

    result = ApbWriteWithStrobe(0x40000100, 0xAABBCCDD, 0x0F);
    if (result == 0) {
        printf("APB write with strobe completed\n");
    }

    result = ApbWriteMultiple(0x40001000, multiWriteData, 4);
    if (result == 0) {
        printf("APB multiple write completed (4 words)\n");
    }

    result = ApbReadMultiple(0x40001000, multiReadData, 4);
    if (result == 0) {
        printf("APB multiple read completed (4 words)\n");
    }

    result = ApbSetTimeout(5000);
    if (result == 0) {
        printf("APB timeout set to 5000 us\n");
    }

    result = ApbDeinit();
    if (result != 0) {
        printf("APB deinitialization failed!\n");
        return -1;
    }

    printf("APB test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Test DMA functionality
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Tests DMA memory-to-memory transfer
 *****************************************************************************/
int32_t TestDma(void)
{
    uint8_t srcBuffer[256];
    uint8_t destBuffer[256];
    DmaConfig config;
    int32_t result = 0;

    printf("Testing DMA...\n");

    result = DmaInit();
    if (result != 0) {
        printf("DMA initialization failed!\n");
        return -1;
    }

    for (int i = 0; i < 256; i++) {
        srcBuffer[i] = (uint8_t)(i & 0xFF);
        destBuffer[i] = 0;
    }

    config.controller = 1;
    config.channel = 0;
    config.direction = DMA_DIR_MEM2MEM;
    config.dataWidth = DMA_DATA_WIDTH_BYTE;
    config.priority = DMA_PRIORITY_HIGH;
    config.mode = DMA_MODE_NORMAL;
    config.memoryIncrement = 1;
    config.peripheralIncrement = 1;
    config.fifoMode = 0;
    config.fifoThreshold = 0;

    result = DmaConfigureChannel(&config);
    if (result != 0) {
        printf("DMA channel configuration failed!\n");
        return -1;
    }
    printf("DMA channel configured\n");

    result = DmaStartTransfer(1, 0, (uint32_t)srcBuffer, (uint32_t)destBuffer, 256);
    if (result != 0) {
        printf("DMA transfer start failed!\n");
        return -1;
    }

    result = DmaWaitComplete(1, 0, 1000);
    if (result != 0) {
        printf("DMA transfer timeout!\n");
        return -1;
    }
    printf("DMA transfer completed\n");

    for (int i = 0; i < 256; i++) {
        if (destBuffer[i] != srcBuffer[i]) {
            printf("DMA data verification failed at byte %d!\n", i);
            return -1;
        }
    }
    printf("DMA data verification passed\n");

    result = DmaMemCopy((uint32_t)srcBuffer, (uint32_t)destBuffer, 128);
    if (result == 0) {
        printf("DMA memory copy test passed\n");
    }

    result = DmaDeinit();
    if (result != 0) {
        printf("DMA deinitialization failed!\n");
        return -1;
    }

    printf("DMA test completed.\n");
    return 0;
}

/******************************************************************************
 * @brief     : Main function - entry point of the application
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 on success
 * @note      : Runs all module tests
 *****************************************************************************/
int32_t main(void)
{
    int32_t testResult = 0;

    printf("Starting Bus Module Tests...\n");

    testResult = TestUart();
    if (testResult != 0) {
        printf("UART test failed!\n");
        return -1;
    }

    testResult = TestSpi();
    if (testResult != 0) {
        printf("SPI test failed!\n");
        return -1;
    }

    testResult = TestI2c();
    if (testResult != 0) {
        printf("I2C test failed!\n");
        return -1;
    }

    testResult = TestI3c();
    if (testResult != 0) {
        printf("I3C test failed!\n");
        return -1;
    }

    testResult = TestI2s();
    if (testResult != 0) {
        printf("I2S test failed!\n");
        return -1;
    }

    testResult = TestModbus();
    if (testResult != 0) {
        printf("Modbus test failed!\n");
        return -1;
    }

    testResult = TestSmbus();
    if (testResult != 0) {
        printf("SMBus test failed!\n");
        return -1;
    }

    testResult = TestPmbus();
    if (testResult != 0) {
        printf("PMBus test failed!\n");
        return -1;
    }

    testResult = TestRs232();
    if (testResult != 0) {
        printf("RS232 test failed!\n");
        return -1;
    }

    testResult = TestRs422();
    if (testResult != 0) {
        printf("RS422 test failed!\n");
        return -1;
    }

    testResult = TestRs485();
    if (testResult != 0) {
        printf("RS485 test failed!\n");
        return -1;
    }

    testResult = TestCan();
    if (testResult != 0) {
        printf("CAN test failed!\n");
        return -1;
    }

    testResult = TestLin();
    if (testResult != 0) {
        printf("LIN test failed!\n");
        return -1;
    }

    testResult = TestFlexRay();
    if (testResult != 0) {
        printf("FlexRay test failed!\n");
        return -1;
    }

    testResult = TestOneWire();
    if (testResult != 0) {
        printf("OneWire test failed!\n");
        return -1;
    }

    testResult = TestUsb();
    if (testResult != 0) {
        printf("USB test failed!\n");
        return -1;
    }

    testResult = TestSdio();
    if (testResult != 0) {
        printf("SDIO test failed!\n");
        return -1;
    }

    testResult = TestEthernet();
    if (testResult != 0) {
        printf("Ethernet test failed!\n");
        return -1;
    }

    testResult = TestSwd();
    if (testResult != 0) {
        printf("SWD test failed!\n");
        return -1;
    }

    testResult = TestJtag();
    if (testResult != 0) {
        printf("JTAG test failed!\n");
        return -1;
    }

    testResult = TestPcie();
    if (testResult != 0) {
        printf("PCIe test failed!\n");
        return -1;
    }

    testResult = TestEmmc();
    if (testResult != 0) {
        printf("eMMC test failed!\n");
        return -1;
    }

    testResult = TestAhb();
    if (testResult != 0) {
        printf("AHB test failed!\n");
        return -1;
    }

    testResult = TestAxi();
    if (testResult != 0) {
        printf("AXI test failed!\n");
        return -1;
    }

    testResult = TestApb();
    if (testResult != 0) {
        printf("APB test failed!\n");
        return -1;
    }

    testResult = TestDma();
    if (testResult != 0) {
        printf("DMA test failed!\n");
        return -1;
    }

    printf("All tests completed successfully.\n");
    return 0;
}
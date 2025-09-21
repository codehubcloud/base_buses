#include <stdint.h>
#include <stdio.h>
#include "can/can.h"
#include "i2c/i2c.h"
#include "i3c/i3c.h"
#include "modbus/modbus.h"
#include "rs232/rs232.h"
#include "rs485/rs485.h"
#include "smbus/smbus.h"
#include "spi/spi.h"
#include "uart/uart.h"

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

    testResult = TestRs232();
    if (testResult != 0) {
        printf("RS232 test failed!\n");
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

    printf("All tests completed successfully.\n");
    return 0;
}
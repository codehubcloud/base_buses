#include <string.h>
#include "modbus.h"
#include "modbus_hal.h"
#include "securec.h"

/******************************************************************************
 * @brief     : Initialize Modbus communication with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures Modbus parameters and initializes UART
 *****************************************************************************/
int32_t ModbusInit(void)
{
    int32_t result = 0;

    result = ModbusUartInit();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Send Modbus RTU request and receive response
 * @param[in] : request - Pointer to Modbus request structure
 * @param[out]: response - Pointer to response buffer
 * @return    : Number of bytes received, -1 if error
 * @note      : Blocking function that sends request and waits for response
 *****************************************************************************/
int32_t ModbusSendReceive(ModbusRequest_S* request, uint8_t* response)
{
    uint8_t requestBuffer[256];
    uint16_t requestLength = 0;
    int32_t result = 0;
    int32_t responseLength = 0;

    if ((request == NULL) || (request->data == NULL) || (response == NULL) || (request->dataLength == 0)
        || (request->responseMaxLength == 0)) {
        return -1;
    }

    if ((request->dataLength + 4) > (sizeof(requestBuffer) / sizeof(requestBuffer[0]))) {
        return -1;
    }

    requestBuffer[0] = request->slaveAddr;
    requestBuffer[1] = request->functionCode;

    // Copy data to request
    for (uint16_t i = 0; i < request->dataLength; i++) {
        requestBuffer[2 + i] = request->data[i];
    }

    requestLength = 2 + request->dataLength;

    // Calculate and append CRC
    uint16_t crc = ModbusCalculateCRC(requestBuffer, requestLength);
    requestBuffer[requestLength] = (uint8_t)(crc & 0xFF);
    requestBuffer[requestLength + 1] = (uint8_t)((crc >> 8) & 0xFF);
    requestLength += 2;

    // Send request
    result = ModbusUartSendData(requestBuffer, requestLength);
    if (result != 0) {
        return -1;
    }

    // Receive response
    responseLength = ModbusUartReceiveData(response, request->responseMaxLength);
    if (responseLength <= 0) {
        return -1;
    }

    // Verify CRC
    crc = ModbusCalculateCRC(response, responseLength - 2);
    if ((response[responseLength - 2] != (uint8_t)(crc & 0xFF)) || (response[responseLength - 1] != (uint8_t)((crc >> 8) & 0xFF))) {
        return -1;
    }

    return responseLength - 2; // Exclude CRC bytes
}

/******************************************************************************
 * @brief     : Read Modbus coils (function code 0x01)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of coils to read
 * @param[out]: coilStatus - Pointer to coil status buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads discrete outputs (coils)
 *****************************************************************************/
int32_t ModbusReadCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t* coilStatus)
{
    uint8_t data[4];
    uint8_t response[256];
    int32_t responseLength = 0;
    int32_t result = 0;

    if ((coilStatus == NULL) || (quantity == 0) || (quantity > 2000)) {
        return -1;
    }

    data[0] = (uint8_t)(startAddr >> 8);
    data[1] = (uint8_t)(startAddr & 0xFF);
    data[2] = (uint8_t)(quantity >> 8);
    data[3] = (uint8_t)(quantity & 0xFF);

    responseLength = ModbusSendReceive(slaveAddr, 0x01, data, 4, response, sizeof(response));
    if (responseLength <= 0) {
        return -1;
    }

    // Check for exception
    if ((responseLength >= 2) && (response[1] & 0x80)) {
        return -1;
    }

    // Copy coil status
    uint8_t byteCount = response[2];
    for (uint8_t i = 0; i < byteCount; i++) {
        coilStatus[i] = response[3 + i];
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read Modbus input discretes (function code 0x02)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of inputs to read
 * @param[out]: inputStatus - Pointer to input status buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads discrete inputs
 *****************************************************************************/
int32_t ModbusReadDiscreteInputs(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t* inputStatus)
{
    uint8_t data[4];
    uint8_t response[256];
    int32_t responseLength = 0;

    if ((inputStatus == NULL) || (quantity == 0) || (quantity > 2000)) {
        return -1;
    }

    data[0] = (uint8_t)(startAddr >> 8);
    data[1] = (uint8_t)(startAddr & 0xFF);
    data[2] = (uint8_t)(quantity >> 8);
    data[3] = (uint8_t)(quantity & 0xFF);

    responseLength = ModbusSendReceive(slaveAddr, 0x02, data, 4, response, sizeof(response));
    if (responseLength <= 0) {
        return -1;
    }

    // Check for exception
    if ((responseLength >= 2) && (response[1] & 0x80)) {
        return -1;
    }

    // Copy input status
    uint8_t byteCount = response[2];
    for (uint8_t i = 0; i < byteCount; i++) {
        inputStatus[i] = response[3 + i];
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read Modbus holding registers (function code 0x03)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of registers to read
 * @param[out]: registerValues - Pointer to register values buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads holding registers
 *****************************************************************************/
int32_t ModbusReadHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t* registerValues)
{
    uint8_t data[4];
    uint8_t response[256];
    int32_t responseLength = 0;

    if ((registerValues == NULL) || (quantity == 0) || (quantity > 125)) {
        return -1;
    }

    data[0] = (uint8_t)(startAddr >> 8);
    data[1] = (uint8_t)(startAddr & 0xFF);
    data[2] = (uint8_t)(quantity >> 8);
    data[3] = (uint8_t)(quantity & 0xFF);

    responseLength = ModbusSendReceive(slaveAddr, 0x03, data, 4, response, sizeof(response));
    if (responseLength <= 0) {
        return -1;
    }

    // Check for exception
    if ((responseLength >= 2) && (response[1] & 0x80)) {
        return -1;
    }

    // Copy register values
    uint8_t byteCount = response[2];
    for (uint8_t i = 0; i < byteCount; i += 2) {
        registerValues[i / 2] = (uint16_t)(response[3 + i] << 8) | response[4 + i];
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read Modbus input registers (function code 0x04)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of registers to read
 * @param[out]: inputRegisters - Pointer to input registers buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads input registers
 *****************************************************************************/
int32_t ModbusReadInputRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t* inputRegisters)
{
    uint8_t data[4];
    uint8_t response[256];
    int32_t responseLength = 0;

    if ((inputRegisters == NULL) || (quantity == 0) || (quantity > 125)) {
        return -1;
    }

    data[0] = (uint8_t)(startAddr >> 8);
    data[1] = (uint8_t)(startAddr & 0xFF);
    data[2] = (uint8_t)(quantity >> 8);
    data[3] = (uint8_t)(quantity & 0xFF);

    responseLength = ModbusSendReceive(slaveAddr, 0x04, data, 4, response, sizeof(response));
    if (responseLength <= 0) {
        return -1;
    }

    // Check for exception
    if ((responseLength >= 2) && (response[1] & 0x80)) {
        return -1;
    }

    // Copy input registers
    uint8_t byteCount = response[2];
    for (uint8_t i = 0; i < byteCount; i += 2) {
        inputRegisters[i / 2] = (uint16_t)(response[3 + i] << 8) | response[4 + i];
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write single Modbus coil (function code 0x05)
 * @param[in] : slaveAddr - Slave device address, coilAddr - Coil address, coilValue - Coil value (0x0000 or 0xFF00)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes single coil
 *****************************************************************************/
int32_t ModbusWriteSingleCoil(uint8_t slaveAddr, uint16_t coilAddr, uint16_t coilValue)
{
    uint8_t data[4];
    uint8_t response[256];
    int32_t responseLength = 0;

    data[0] = (uint8_t)(coilAddr >> 8);
    data[1] = (uint8_t)(coilAddr & 0xFF);
    data[2] = (uint8_t)(coilValue >> 8);
    data[3] = (uint8_t)(coilValue & 0xFF);

    responseLength = ModbusSendReceive(slaveAddr, 0x05, data, 4, response, sizeof(response));
    if (responseLength <= 0) {
        return -1;
    }

    // Check for exception
    if ((responseLength >= 2) && (response[1] & 0x80)) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write single Modbus register (function code 0x06)
 * @param[in] : slaveAddr - Slave device address, registerAddr - Register address, registerValue - Register value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes single register
 *****************************************************************************/
int32_t ModbusWriteSingleRegister(uint8_t slaveAddr, uint16_t registerAddr, uint16_t registerValue)
{
    uint8_t data[4];
    uint8_t response[256];
    int32_t responseLength = 0;

    data[0] = (uint8_t)(registerAddr >> 8);
    data[1] = (uint8_t)(registerAddr & 0xFF);
    data[2] = (uint8_t)(registerValue >> 8);
    data[3] = (uint8_t)(registerValue & 0xFF);

    responseLength = ModbusSendReceive(slaveAddr, 0x06, data, 4, response, sizeof(response));
    if (responseLength <= 0) {
        return -1;
    }

    // Check for exception
    if ((responseLength >= 2) && (response[1] & 0x80)) {
        return -1;
    }

    return 0;
}
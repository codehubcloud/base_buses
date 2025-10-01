#ifndef MODBUS_H
#define MODBUS_H

#include <stdint.h>

#define MODBUS_DEFAULT_BAUDRATE 9600
#define MODBUS_DEFAULT_TIMEOUT 1000

typedef struct {
    uint8_t slaveAddr;
    uint8_t functionCode;
    uint8_t* data;
    uint16_t dataLength;
    uint16_t responseMaxLength;
} ModbusRequest_S;

/******************************************************************************
 * @brief     : Initialize Modbus communication with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures Modbus parameters and initializes UART
 *****************************************************************************/
int32_t ModbusInit(void);

/******************************************************************************
 * @brief     : Send Modbus RTU request and receive response
 * @param[in] : request - Pointer to Modbus request structure
 * @param[out]: response - Pointer to response buffer
 * @return    : Number of bytes received, -1 if error
 * @note      : Blocking function that sends request and waits for response
 *****************************************************************************/
int32_t ModbusSendReceive(ModbusRequest_S* request, uint8_t* response);

/******************************************************************************
 * @brief     : Read Modbus coils (function code 0x01)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of coils to read
 * @param[out]: coilStatus - Pointer to coil status buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads discrete outputs (coils)
 *****************************************************************************/
int32_t ModbusReadCoils(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t* coilStatus);

/******************************************************************************
 * @brief     : Read Modbus input discretes (function code 0x02)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of inputs to read
 * @param[out]: inputStatus - Pointer to input status buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads discrete inputs
 *****************************************************************************/
int32_t ModbusReadDiscreteInputs(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint8_t* inputStatus);

/******************************************************************************
 * @brief     : Read Modbus holding registers (function code 0x03)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of registers to read
 * @param[out]: registerValues - Pointer to register values buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads holding registers
 *****************************************************************************/
int32_t ModbusReadHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t* registerValues);

/******************************************************************************
 * @brief     : Read Modbus input registers (function code 0x04)
 * @param[in] : slaveAddr - Slave device address, startAddr - Starting address, quantity - Number of registers to read
 * @param[out]: inputRegisters - Pointer to input registers buffer
 * @return    : 0 if success, -1 if error
 * @note      : Reads input registers
 *****************************************************************************/
int32_t ModbusReadInputRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t* inputRegisters);

/******************************************************************************
 * @brief     : Write single Modbus coil (function code 0x05)
 * @param[in] : slaveAddr - Slave device address, coilAddr - Coil address, coilValue - Coil value (0x0000 or 0xFF00)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes single coil
 *****************************************************************************/
int32_t ModbusWriteSingleCoil(uint8_t slaveAddr, uint16_t coilAddr, uint16_t coilValue);

/******************************************************************************
 * @brief     : Write single Modbus register (function code 0x06)
 * @param[in] : slaveAddr - Slave device address, registerAddr - Register address, registerValue - Register value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Writes single register
 *****************************************************************************/
int32_t ModbusWriteSingleRegister(uint8_t slaveAddr, uint16_t registerAddr, uint16_t registerValue);

#endif // MODBUS_H
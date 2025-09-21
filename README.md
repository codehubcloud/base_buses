# 嵌入式总线通信协议库

## 项目概述

这是一个用于嵌入式系统的总线通信协议库，提供了多种常用通信协议的实现，包括UART、SPI、I2C、I3C、CAN、Modbus、SMBus、RS232和RS485。该项目采用模块化设计，每个协议独立为一个模块，便于集成和使用（**HAL部分由用户根据自己的硬件进行适配**）。
[持续更新中...](https://github.com/codehubcloud/base_buss.git)
## 目录结构

```
base_buss/
├── 3rd_party/              # 第三方库
│   └── securelib/          # 安全库
├── src/                    # 源代码目录
│   ├── uart/               # UART模块
│   ├── spi/                # SPI模块
│   ├── i2c/                # I2C模块
│   ├── i3c/                # I3C模块
│   ├── can/                # CAN模块
│   ├── modbus/             # Modbus模块
│   ├── smbus/              # SMBus模块
│   ├── rs232/              # RS232模块
│   ├── rs485/              # RS485模块
│   ├── main.c              # 测试主函数
│   └── CMakeLists.txt      # 源码根目录CMake配置
├── CMakeLists.txt          # 项目根CMake配置
└── README.md               # 项目说明文档
```

## 模块介绍

### UART (通用异步收发传输器)

UART是一种异步串行通信协议，常用于微控制器与其它设备之间的短距离通信。

**主要功能：**
- 初始化UART外设
- 发送数据
- 接收数据
- 设置波特率
- 启用/禁用中断

**API接口：**
- `UartInit()` - 初始化UART
- `UartSendData()` - 发送数据
- `UartReceiveData()` - 接收数据
- `UartSetBaudRate()` - 设置波特率
- `UartEnableInterrupts()` - 启用中断
- `UartDisableInterrupts()` - 禁用中断

### SPI (串行外设接口)

SPI是一种同步串行通信协议，支持全双工通信，常用于微控制器与传感器、存储器等外设的通信。

**主要功能：**
- 初始化SPI外设
- 全双工数据传输
- 单向发送数据
- 单向接收数据
- 设置时钟速度
- 设置主从模式

**API接口：**
- `SpiInit()` - 初始化SPI
- `SpiTransfer()` - 全双工传输
- `SpiSendData()` - 发送数据
- `SpiReceiveData()` - 接收数据
- `SpiSetClockSpeed()` - 设置时钟速度
- `SpiSetMode()` - 设置主从模式

### I2C (Inter-Integrated Circuit)

I2C是一种多主机、多从机的串行通信协议，只需要两根线(SDA和SCL)即可实现设备间通信。

**主要功能：**
- 初始化I2C外设
- 向设备写入数据
- 从设备读取数据
- 设置时钟速度
- 检查设备是否存在

**API接口：**
- `I2cInit()` - 初始化I2C
- `I2cWriteData()` - 写入数据
- `I2cReadData()` - 读取数据
- `I2cSetClockSpeed()` - 设置时钟速度
- `I2cCheckDevice()` - 检查设备

### I3C (Improved Inter-Integrated Circuit)

I3C是I2C的改进版本，提供了更高的数据传输速率和更丰富的功能。

**主要功能：**
- 初始化I3C外设
- 向设备发送数据
- 从设备读取数据
- 设置时钟速度
- 检查设备是否存在

**API接口：**
- `I3cInit()` - 初始化I3C
- `I3cSendData()` - 发送数据
- `I3cReadData()` - 读取数据
- `I3cSetClockSpeed()` - 设置时钟速度
- `I3cCheckDevice()` - 检查设备

### CAN (Controller Area Network)

CAN是一种广泛应用于汽车和工业控制领域的串行通信协议，具有高可靠性和抗干扰能力。

**主要功能：**
- 初始化CAN外设
- 发送CAN消息
- 接收CAN消息
- 设置波特率
- 启用/禁用中断

**API接口：**
- `CanInit()` - 初始化CAN
- `CanSendData()` - 发送数据
- `CanReceiveData()` - 接收数据
- `CanSetBaudRate()` - 设置波特率
- `CanEnableInterrupts()` - 启用中断
- `CanDisableInterrupts()` - 禁用中断

### Modbus

Modbus是一种串行通信协议，广泛应用于工业电子设备之间。

**主要功能：**
- 初始化Modbus通信
- 发送请求并接收响应
- 读取线圈状态
- 读取离散输入
- 读取保持寄存器
- 读取输入寄存器
- 写入单个线圈
- 写入单个寄存器

**API接口：**
- `ModbusInit()` - 初始化Modbus
- `ModbusSendReceive()` - 发送请求并接收响应
- `ModbusReadCoils()` - 读取线圈
- `ModbusReadDiscreteInputs()` - 读取离散输入
- `ModbusReadHoldingRegisters()` - 读取保持寄存器
- `ModbusReadInputRegisters()` - 读取输入寄存器
- `ModbusWriteSingleCoil()` - 写入单个线圈
- `ModbusWriteSingleRegister()` - 写入单个寄存器

### SMBus (System Management Bus)

SMBus是基于I2C的两线制总线，主要用于系统管理和通信。

**主要功能：**
- 初始化SMBus外设
- 向设备发送数据
- 从设备读取数据
- 读写字节数据
- 设置时钟速度
- 检查设备是否存在

**API接口：**
- `SmBusInit()` - 初始化SMBus
- `SmBusSendData()` - 发送数据
- `SmBusReadData()` - 读取数据
- `SmBusReadByte()` - 读取字节
- `SmBusWriteByte()` - 写入字节
- `SmBusSetClockSpeed()` - 设置时钟速度
- `SmBusCheckDevice()` - 检查设备

### RS232

RS232是一种标准的串行通信协议，广泛用于计算机与外设之间的通信。

**主要功能：**
- 初始化RS232外设
- 发送数据
- 接收数据
- 设置波特率
- 启用/禁用中断

**API接口：**
- `Rs232Init()` - 初始化RS232
- `Rs232SendData()` - 发送数据
- `Rs232ReceiveData()` - 接收数据
- `Rs232SetBaudRate()` - 设置波特率
- `Rs232EnableInterrupts()` - 启用中断
- `Rs232DisableInterrupts()` - 禁用中断

### RS485

RS485是一种差分串行通信协议，支持多点通信，传输距离远，抗干扰能力强。

**主要功能：**
- 初始化RS485外设
- 发送数据
- 接收数据
- 设置波特率
- 启用发送/接收模式

**API接口：**
- `Rs485Init()` - 初始化RS485
- `Rs485SendData()` - 发送数据
- `Rs485ReceiveData()` - 接收数据
- `Rs485SetBaudRate()` - 设置波特率
- `Rs485EnableTransmit()` - 启用发送模式
- `Rs485EnableReceive()` - 启用接收模式

## 硬件抽象层(HAL)

每个模块都采用了硬件抽象层设计，将硬件相关操作与协议逻辑分离。硬件相关函数位于`xxx_hal.c`和`xxx_hal.h`文件中，便于移植到不同的硬件平台。

## 编译构建

项目使用CMake构建系统，支持跨平台编译。

### 构建步骤

```bash
mkdir build
cd build
cmake ..
make
```

### 构建配置

- 支持Debug和Release两种构建模式
- 默认使用C99标准
- Debug模式启用调试信息和额外警告
- Release模式启用优化

## 使用示例

每个模块都提供了测试函数，可在`main.c`中找到使用示例。

## 编码规范

- 采用大驼峰命名法命名函数
- 局部变量和参数采用小驼峰命名法
- 所有控制语句都使用花括号包裹
- 严格校验函数返回值
- 使用第三方安全库函数
- 每个函数都有完整注释
- 每个函数非空非注释行数不超过50行

## 仓库

[GitHub](# 嵌入式总线通信协议库

## 项目概述

这是一个用于嵌入式系统的总线通信协议库，提供了多种常用通信协议的实现，包括UART、SPI、I2C、I3C、CAN、Modbus、SMBus、RS232和RS485。该项目采用模块化设计，每个协议独立为一个模块，便于集成和使用。

## 目录结构

```
base_buss/
├── 3rd_party/              # 第三方库
│   └── securelib/          # 安全库
├── src/                    # 源代码目录
│   ├── uart/               # UART模块
│   ├── spi/                # SPI模块
│   ├── i2c/                # I2C模块
│   ├── i3c/                # I3C模块
│   ├── can/                # CAN模块
│   ├── modbus/             # Modbus模块
│   ├── smbus/              # SMBus模块
│   ├── rs232/              # RS232模块
│   ├── rs485/              # RS485模块
│   ├── main.c              # 测试主函数
│   └── CMakeLists.txt      # 源码根目录CMake配置
├── CMakeLists.txt          # 项目根CMake配置
└── README.md               # 项目说明文档
```

## 模块介绍

### UART (通用异步收发传输器)

UART是一种异步串行通信协议，常用于微控制器与其它设备之间的短距离通信。

**主要功能：**
- 初始化UART外设
- 发送数据
- 接收数据
- 设置波特率
- 启用/禁用中断

**API接口：**
- `UartInit()` - 初始化UART
- `UartSendData()` - 发送数据
- `UartReceiveData()` - 接收数据
- `UartSetBaudRate()` - 设置波特率
- `UartEnableInterrupts()` - 启用中断
- `UartDisableInterrupts()` - 禁用中断

### SPI (串行外设接口)

SPI是一种同步串行通信协议，支持全双工通信，常用于微控制器与传感器、存储器等外设的通信。

**主要功能：**
- 初始化SPI外设
- 全双工数据传输
- 单向发送数据
- 单向接收数据
- 设置时钟速度
- 设置主从模式

**API接口：**
- `SpiInit()` - 初始化SPI
- `SpiTransfer()` - 全双工传输
- `SpiSendData()` - 发送数据
- `SpiReceiveData()` - 接收数据
- `SpiSetClockSpeed()` - 设置时钟速度
- `SpiSetMode()` - 设置主从模式

### I2C (Inter-Integrated Circuit)

I2C是一种多主机、多从机的串行通信协议，只需要两根线(SDA和SCL)即可实现设备间通信。

**主要功能：**
- 初始化I2C外设
- 向设备写入数据
- 从设备读取数据
- 设置时钟速度
- 检查设备是否存在

**API接口：**
- `I2cInit()` - 初始化I2C
- `I2cWriteData()` - 写入数据
- `I2cReadData()` - 读取数据
- `I2cSetClockSpeed()` - 设置时钟速度
- `I2cCheckDevice()` - 检查设备

### I3C (Improved Inter-Integrated Circuit)

I3C是I2C的改进版本，提供了更高的数据传输速率和更丰富的功能。

**主要功能：**
- 初始化I3C外设
- 向设备发送数据
- 从设备读取数据
- 设置时钟速度
- 检查设备是否存在

**API接口：**
- `I3cInit()` - 初始化I3C
- `I3cSendData()` - 发送数据
- `I3cReadData()` - 读取数据
- `I3cSetClockSpeed()` - 设置时钟速度
- `I3cCheckDevice()` - 检查设备

### CAN (Controller Area Network)

CAN是一种广泛应用于汽车和工业控制领域的串行通信协议，具有高可靠性和抗干扰能力。

**主要功能：**
- 初始化CAN外设
- 发送CAN消息
- 接收CAN消息
- 设置波特率
- 启用/禁用中断

**API接口：**
- `CanInit()` - 初始化CAN
- `CanSendData()` - 发送数据
- `CanReceiveData()` - 接收数据
- `CanSetBaudRate()` - 设置波特率
- `CanEnableInterrupts()` - 启用中断
- `CanDisableInterrupts()` - 禁用中断

### Modbus

Modbus是一种串行通信协议，广泛应用于工业电子设备之间。

**主要功能：**
- 初始化Modbus通信
- 发送请求并接收响应
- 读取线圈状态
- 读取离散输入
- 读取保持寄存器
- 读取输入寄存器
- 写入单个线圈
- 写入单个寄存器

**API接口：**
- `ModbusInit()` - 初始化Modbus
- `ModbusSendReceive()` - 发送请求并接收响应
- `ModbusReadCoils()` - 读取线圈
- `ModbusReadDiscreteInputs()` - 读取离散输入
- `ModbusReadHoldingRegisters()` - 读取保持寄存器
- `ModbusReadInputRegisters()` - 读取输入寄存器
- `ModbusWriteSingleCoil()` - 写入单个线圈
- `ModbusWriteSingleRegister()` - 写入单个寄存器

### SMBus (System Management Bus)

SMBus是基于I2C的两线制总线，主要用于系统管理和通信。

**主要功能：**
- 初始化SMBus外设
- 向设备发送数据
- 从设备读取数据
- 读写字节数据
- 设置时钟速度
- 检查设备是否存在

**API接口：**
- `SmBusInit()` - 初始化SMBus
- `SmBusSendData()` - 发送数据
- `SmBusReadData()` - 读取数据
- `SmBusReadByte()` - 读取字节
- `SmBusWriteByte()` - 写入字节
- `SmBusSetClockSpeed()` - 设置时钟速度
- `SmBusCheckDevice()` - 检查设备

### RS232

RS232是一种标准的串行通信协议，广泛用于计算机与外设之间的通信。

**主要功能：**
- 初始化RS232外设
- 发送数据
- 接收数据
- 设置波特率
- 启用/禁用中断

**API接口：**
- `Rs232Init()` - 初始化RS232
- `Rs232SendData()` - 发送数据
- `Rs232ReceiveData()` - 接收数据
- `Rs232SetBaudRate()` - 设置波特率
- `Rs232EnableInterrupts()` - 启用中断
- `Rs232DisableInterrupts()` - 禁用中断

### RS485

RS485是一种差分串行通信协议，支持多点通信，传输距离远，抗干扰能力强。

**主要功能：**
- 初始化RS485外设
- 发送数据
- 接收数据
- 设置波特率
- 启用发送/接收模式

**API接口：**
- `Rs485Init()` - 初始化RS485
- `Rs485SendData()` - 发送数据
- `Rs485ReceiveData()` - 接收数据
- `Rs485SetBaudRate()` - 设置波特率
- `Rs485EnableTransmit()` - 启用发送模式
- `Rs485EnableReceive()` - 启用接收模式

## 硬件抽象层(HAL)

每个模块都采用了硬件抽象层设计，将硬件相关操作与协议逻辑分离。硬件相关函数位于`xxx_hal.c`和`xxx_hal.h`文件中，便于移植到不同的硬件平台。

## 编译构建

项目使用CMake构建系统，支持跨平台编译。

### 构建步骤

```bash
mkdir build
cd build
cmake ..
make
```

### 构建配置

- 支持Debug和Release两种构建模式
- 默认使用C99标准
- Debug模式启用调试信息和额外警告
- Release模式启用优化

## 使用示例

每个模块都提供了测试函数，可在`main.c`中找到使用示例。

## 编码规范

- 采用大驼峰命名法命名函数
- 局部变量和参数采用小驼峰命名法
- 所有控制语句都使用花括号包裹
- 严格校验函数返回值
- 使用第三方安全库函数
- 每个函数都有完整注释
- 每个函数非空非注释行数不超过50行

## 仓库

[Github](https://github.com/codehubcloud/base_buss.git)
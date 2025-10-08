# 嵌入式总线通信协议库 (Embedded Bus Communication Protocol Library)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()
[![Platform](https://img.shields.io/badge/platform-STM32%20%7C%20ESP32%20%7C%20Linux-blue.svg)]()

## 项目概述 (Project Overview)

这是一个功能完整的嵌入式系统总线通信协议库，提供了 27 种常用通信协议和总线接口的实现。该项目采用**硬件抽象层(HAL)模式**设计，支持多平台移植，每个协议独立为一个模块，便于集成和使用。

**主要特性:**

- ✅ 27 个独立通信协议模块
- ✅ 支持 4 大平台: STM32F4、STM32F1、ESP32、Linux
- ✅ 完整的硬件抽象层(HAL)设计
- ✅ 使用 securec 安全函数库
- ✅ 使用业界的代码规范，结合Google的代码规范,进行一些修改
- ✅ 每个模块独立编译为静态库
- ✅ 完整的测试用例和使用示例

[持续更新中...](https://github.com/codehubcloud/base_buses.git)

---

## 目录结构 (Project Structure)

```
base_buses/
├── 3rd_party/              # 第三方库
│   └── securelib/          # securec安全库
├── src/                    # 源代码目录 (协议实现)
│   ├── uart/               # UART - 通用异步收发器
│   ├── spi/                # SPI - 串行外设接口
│   ├── i2c/                # I2C - 两线制总线
│   ├── i3c/                # I3C - 改进型I3C
│   ├── i2s/                # I2S - 音频数据接口
│   ├── can/                # CAN - 控制器局域网
│   ├── canfd/              # CAN FD - 灵活数据速率CAN
│   ├── lin/                # LIN - 局域互连网络
│   ├── flexray/            # FlexRay - 高速汽车总线
│   ├── modbus/             # Modbus - 工业协议
│   ├── smbus/              # SMBus - 系统管理总线
│   ├── pmbus/              # PMBus - 电源管理总线
│   ├── rs232/              # RS232 - 标准串口
│   ├── rs422/              # RS422 - 差分串口
│   ├── rs485/              # RS485 - 多点通信总线
│   ├── usb/                # USB - 通用串行总线
│   ├── sdio/               # SDIO - SD卡接口
│   ├── onewire/            # 1-Wire - 单总线协议
│   ├── ethernet/           # Ethernet - 以太网
│   ├── pcie/               # PCIe - 高速串行总线
│   ├── emmc/               # eMMC - 嵌入式存储
│   ├── jtag/               # JTAG - 调试接口
│   ├── swd/                # SWD - 串行调试接口
│   ├── axi/                # AXI - ARM高级总线
│   ├── ahb/                # AHB - 高性能总线
│   ├── apb/                # APB - 外设总线
│   ├── dma/                # DMA - 直接内存访问
│   ├── platform_config.h   # 平台配置头文件
│   └── CMakeLists.txt      # 源码CMake配置
├── test/                   # 测试目录
│   ├── main.c              # 测试主函数 (所有模块测试)
│   ├── CMakeLists.txt      # 测试CMake配置
│   └── README.md           # 测试说明文档
├── CMakeLists.txt          # 项目根CMake配置
└── README.md               # 项目说明文档
```

---

## 模块分类 (Module Categories)

### 🔌 串行通信协议 (Serial Communication)

| 模块       | 说明           | 应用场景        |
| ---------- | -------------- | --------------- |
| **UART**   | 通用异步收发器 | MCU 串口通信    |
| **RS232**  | 标准串口协议   | 计算机外设通信  |
| **RS422**  | 差分全双工串口 | 工业长距离通信  |
| **RS485**  | 差分半双工多点 | 工业总线网络    |
| **Modbus** | 工业通信协议   | PLC、传感器网络 |

### 🚌 同步总线协议 (Synchronous Bus)

| 模块      | 说明         | 应用场景       |
| --------- | ------------ | -------------- |
| **SPI**   | 串行外设接口 | 传感器、存储器 |
| **I2C**   | 两线制总线   | 传感器、EEPROM |
| **I3C**   | 改进型 I3C   | 高速传感器网络 |
| **SMBus** | 系统管理总线 | 电池管理、温控 |
| **PMBus** | 电源管理总线 | 智能电源管理   |
| **I2S**   | 音频数据接口 | 音频编解码器   |

### 🚗 汽车总线协议 (Automotive Bus)

| 模块        | 说明             | 应用场景       |
| ----------- | ---------------- | -------------- |
| **CAN**     | 控制器局域网     | 汽车 ECU 通信  |
| **CAN FD**  | 灵活数据速率 CAN | 高速汽车网络   |
| **LIN**     | 局域互连网络     | 低速汽车子系统 |
| **FlexRay** | 时间触发总线     | 高端汽车底盘   |

### 💾 存储与高速接口 (Storage & High-Speed)

| 模块     | 说明         | 应用场景     |
| -------- | ------------ | ------------ |
| **USB**  | 通用串行总线 | USB 设备通信 |
| **SDIO** | SD 卡接口    | SD 卡读写    |
| **eMMC** | 嵌入式存储   | 板载闪存     |
| **PCIe** | 高速串行总线 | 高速外设扩展 |

### 🌐 网络协议 (Networking)

| 模块         | 说明       | 应用场景   |
| ------------ | ---------- | ---------- |
| **Ethernet** | 以太网协议 | 网络通信   |
| **1-Wire**   | 单总线协议 | 温度传感器 |

### 🔧 调试与系统总线 (Debug & System Bus)

| 模块     | 说明             | 应用场景         |
| -------- | ---------------- | ---------------- |
| **JTAG** | 边界扫描调试     | 芯片调试下载     |
| **SWD**  | 串行调试接口     | ARM 调试接口     |
| **AXI**  | ARM 高级扩展总线 | SoC 片内高速总线 |
| **AHB**  | ARM 高性能总线   | SoC 高速外设     |
| **APB**  | ARM 外设总线     | SoC 低速外设     |
| **DMA**  | 直接内存访问     | 高速数据传输     |

---

## 平台支持 (Platform Support)

项目通过条件编译支持多种硬件平台:

| 平台        | 宏定义             | 处理器架构    | 状态        |
| ----------- | ------------------ | ------------- | ----------- |
| **STM32F4** | `PLATFORM_STM32F4` | ARM Cortex-M4 | ✅ 完全支持 |
| **STM32F1** | `PLATFORM_STM32F1` | ARM Cortex-M3 | ✅ 完全支持 |
| **ESP32**   | `PLATFORM_ESP32`   | Xtensa/RISC-V | ✅ 完全支持 |
| **Linux**   | `PLATFORM_LINUX`   | x86/x64/ARM   | ✅ 完全支持 |

### 平台配置文件

所有平台配置集中在 `src/platform_config.h`，通过 CMake 传递平台宏定义。

---

## 编译构建 (Build Instructions)

### 前置要求 (Prerequisites)

- CMake >= 3.10
- GCC/Clang (Linux) 或 ARM-GCC (嵌入式)
- Make 或 Ninja

### 标准构建 (Standard Build)

```bash
# Linux平台 (默认)
mkdir build && cd build
cmake ..
make
./bus_test
```

### 平台选择构建 (Platform-Specific Build)

```bash
# STM32F4平台
cmake -DPLATFORM=STM32F4 ..
make

# STM32F1平台
cmake -DPLATFORM=STM32F1 ..
make

# ESP32平台
cmake -DPLATFORM=ESP32 ..
make

# Linux平台 (显式指定)
cmake -DPLATFORM=LINUX ..
make
```

### 构建类型 (Build Types)

```bash
# Debug构建 (调试信息 + 额外警告)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release构建 (优化)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

### 构建配置说明

- **C 标准**: C99
- **编译器选项**:
  - Debug: `-g -Wall -Wextra`
  - Release: `-O2 -DNDEBUG`
- **依赖库**: securec (自动编译)
- **输出**: 静态库 + 可执行文件

---

## 硬件抽象层(HAL)设计 (HAL Architecture)

### 设计原则

项目严格遵循**协议层与 HAL 层分离**的原则:

```
┌─────────────────────────────────────┐
│   应用层 (Application Layer)        │
├─────────────────────────────────────┤
│   协议层 (Protocol Layer)           │
│   xxx.c / xxx.h                     │
│   - 协议逻辑                        │
│   - 状态机                          │
│   - 数据封装                        │
├─────────────────────────────────────┤
│   HAL层 (Hardware Abstraction)      │
│   xxx_hal.c / xxx_hal.h             │
│   - 寄存器操作                      │
│   - GPIO控制                        │
│   - 中断处理                        │
├─────────────────────────────────────┤
│   硬件层 (Hardware)                 │
│   STM32 / ESP32 / Linux             │
└─────────────────────────────────────┘
```

### HAL 函数命名规范

```c
// 协议层 (平台无关)
int32_t UartInit(void);              // UpperCamelCase
int32_t UartSendData(uint8_t data);

// HAL层 (平台相关)
void UartEnableClock(void);          // UpperCamelCase
void UartConfigureGpio(void);
uint8_t UartReadRegister(void);
```

### 移植新平台步骤

1. 在 `src/platform_config.h` 添加平台检测
2. 在各模块的 `xxx_hal.c` 添加平台条件编译分支
3. 实现平台特定的 HAL 函数
4. 更新 `CMakeLists.txt` 支持新平台
5. **协议层代码无需修改**

---

## API 接口文档 (API Reference)

### 通用函数模式 (Common Function Patterns)

所有模块遵循一致的 API 设计:

| 函数类型  | 命名模式                             | 返回值                     | 示例                |
| --------- | ------------------------------------ | -------------------------- | ------------------- |
| 初始化    | `XxxInit()`                          | `int32_t` (0 成功/-1 失败) | `UartInit()`        |
| 去初始化  | `XxxDeinit()`                        | `int32_t`                  | `UartDeinit()`      |
| 发送数据  | `XxxSendData()` / `XxxWriteData()`   | `int32_t`                  | `SpiSendData()`     |
| 接收数据  | `XxxReceiveData()` / `XxxReadData()` | `int32_t`                  | `CanReceiveData()`  |
| 配置参数  | `XxxSetXxx()`                        | `int32_t`                  | `UartSetBaudRate()` |
| 使能/禁用 | `XxxEnable()` / `XxxDisable()`       | `void`                     | `I2cEnable()`       |
| 状态查询  | `XxxIsXxx()` / `XxxGetXxx()`         | `int32_t` / 具体类型       | `SpiIsBusy()`       |

### 示例: UART 模块

```c
#include "uart.h"

// 初始化UART
int32_t ret = UartInit();
if (ret != 0) {
    // 错误处理
}

// 设置波特率
UartSetBaudRate(115200);

// 发送数据
uint8_t data = 0x55;
UartSendData(data);

// 接收数据
uint8_t rxData = 0;
ret = UartReceiveData(&rxData, 1);

// 启用中断
UartEnableInterrupts();
```

### 示例: CAN 模块

```c
#include "can.h"

// 初始化CAN
CanInit();

// 设置波特率为500Kbps
CanSetBaudRate(500000);

// 发送CAN消息
uint32_t canId = 0x123;
uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
CanSendData(canId, data, 8);

// 接收CAN消息
uint8_t rxBuffer[8];
uint32_t rxId = 0;
int32_t len = CanReceiveData(&rxId, rxBuffer, 8);
```

### 示例: PMBus 模块

```c
#include "pmbus.h"

// 初始化PMBus
PmBusInit();

// 选择电源轨0
PmBusSetPage(0);

// 读取输出电压
float voltage = PmBusReadVoltage();

// 读取输出电流
float current = PmBusReadCurrent();

// 设置输出电压为3.3V
PmBusSetVoltage(3.3f);

// 读取状态
uint16_t status = PmBusGetStatus();

// 清除故障
PmBusClearFaults();
```

---

## 使用示例 (Usage Examples)

每个模块都在 `test/main.c` 中提供了完整的测试函数。

### 测试函数列表

```c
void TestUart(void);        // UART测试
void TestSpi(void);         // SPI测试
void TestI2c(void);         // I2C测试
void TestI3c(void);         // I3C测试
void TestI2s(void);         // I2S测试
void TestCan(void);         // CAN测试
void TestCanFd(void);       // CAN FD测试
void TestLin(void);         // LIN测试
void TestFlexRay(void);     // FlexRay测试
void TestModbus(void);      // Modbus测试
void TestSmBus(void);       // SMBus测试
void TestPmBus(void);       // PMBus测试
void TestRs232(void);       // RS232测试
void TestRs422(void);       // RS422测试
void TestRs485(void);       // RS485测试
void TestUsb(void);         // USB测试
void TestSdio(void);        // SDIO测试
void TestOneWire(void);     // 1-Wire测试
void TestEthernet(void);    // Ethernet测试
void TestPcie(void);        // PCIe测试
void TestEmmc(void);        // eMMC测试
void TestJtag(void);        // JTAG测试
void TestSwd(void);         // SWD测试
void TestAxi(void);         // AXI测试
void TestAhb(void);         // AHB测试
void TestApb(void);         // APB测试
void TestDma(void);         // DMA测试
```

### 运行测试

```bash
cd build
./bus_test
```

详细测试说明请参考: `test/README.md`

---

## 编码规范 (Coding Standards)

### 命名规范

- **函数名**: `UpperCamelCase` (大驼峰)
  - 示例: `UartInit()`, `SpiSendData()`
- **变量名/参数名**: `lowerCamelCase` (小驼峰)
  - 示例: `int32_t dataLength`, `uint8_t* buffer`
- **宏定义**: `UPPER_CASE_WITH_UNDERSCORES`
  - 示例: `#define MAX_BUFFER_SIZE 256`
- **结构体/枚举**: `UpperCamelCase_E/S`
  - 示例: `typedef enum { ... } Status_E;`

### 代码规范

- ✅ 所有控制语句必须使用花括号
- ✅ 每个函数最多 50 行非空非注释代码
- ✅ 每个函数都有完整 Doxygen 注释
- ✅ **所有函数返回值必须检查**
- ✅ 使用 securec 安全函数 (memcpy_s, strcpy_s 等)
- ✅ 返回值: 0 表示成功, -1 表示失败
- ✅ 错误处理: 检查返回值并适当清理资源

### 错误处理示例

```c
// ✅ 正确: 检查返回值
if (memcpy_s(dest, destSize, src, srcSize) != EOK) {
    return -1;
}

// ✅ 正确: 检查系统调用
int fd = open("/dev/i2c-1", O_RDWR);
if (fd < 0) {
    // 错误处理
    return -1;
}

// ✅ 正确: 资源清理
if (ioctl(fd, I2C_SLAVE, addr) < 0) {
    close(fd);  // 清理已分配资源
    return -1;
}
```

### 注释规范

```c
/******************************************************************************
 * @brief     : 初始化UART外设
 * @param[in] : baudRate - 波特率
 * @param[out]: None
 * @return    : 0 - 成功, -1 - 失败
 * @note      : 调用此函数前需要先使能时钟
 *****************************************************************************/
int32_t UartInit(uint32_t baudRate);
```

---

## 项目统计 (Project Statistics)

- **总模块数**: 27 个
- **源文件数**: 110+ (54 .c 文件 + 56 .h 文件)
- **代码行数**: 约 45,000 行
- **支持平台**: 4 个 (STM32F4, STM32F1, ESP32, Linux)
- **依赖库**: securec (安全字符串库)
- **C 标准**: C99
- **构建系统**: CMake

---

## 代码质量保证 (Quality Assurance)

### ✅ 已完成的质量改进

1. **所有 securec 函数返回值检查** (54+ 处修复)

   - memcpy_s, memset_s, strcpy_s, strncpy_s, sprintf_s

2. **所有系统调用返回值检查** (47+ 处修复)

   - open(), ioctl(), read(), write(), bind()

3. **错误处理模式统一**

   - 关键操作: 检查返回值并清理资源
   - 非关键操作: 显式标记 `(void)` 忽略返回值

4. **资源管理优化**
   - 多资源分配时的逆序清理
   - 错误路径的资源释放
   - 文件描述符的正确关闭

---

## 文档资源 (Documentation)

- **README.md** - 项目概述和使用说明 (本文档)
- **CLAUDE.md** - 开发者指南和项目配置
- **test/README.md** - 测试说明和使用指南
- **src/platform_config.h** - 平台配置参考

---

## 常见问题 (FAQ)

### Q1: 如何选择合适的通信协议?

- **短距离低速**: UART, RS232
- **短距离传感器**: I2C, I3C, SMBus
- **高速外设**: SPI
- **工业控制**: RS485, Modbus
- **汽车应用**: CAN, CAN FD, LIN, FlexRay
- **音频应用**: I2S
- **网络通信**: Ethernet
- **电源管理**: PMBus
- **片内总线**: AXI, AHB, APB

### Q2: 如何移植到新平台?

参见 `多平台HAL实现说明.md`，基本步骤:

1. 添加平台宏定义
2. 实现 HAL 层函数
3. 更新 CMakeLists.txt
4. 协议层代码无需修改

### Q3: 如何调试?

- 使用 Debug 构建: `cmake -DCMAKE_BUILD_TYPE=Debug ..`
- 检查返回值
- 启用日志输出
- 使用 JTAG/SWD 调试接口

### Q4: 性能优化建议?

- 使用 Release 构建
- 启用 DMA 传输
- 配置合适的波特率/时钟速度
- 使用中断而非轮询
- 批量传输数据

---

## 贡献指南 (Contributing)

欢迎贡献代码! 请遵循以下规范:

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/new-protocol`)
3. 遵循项目编码规范
4. 添加测试用例
5. 提交代码 (`git commit -m 'Add new protocol'`)
6. 推送分支 (`git push origin feature/new-protocol`)
7. 创建 Pull Request

---

## 许可证 (License)

MIT License - 详见 LICENSE 文件

---

## 联系方式 (Contact)

- **GitHub**: [https://github.com/codehubcloud/base_buses.git](https://github.com/codehubcloud/base_buses.git)
- **Issues**: 欢迎提交问题和建议

---

## 更新日志 (Changelog)

### v2.0.0 (2025-10-01)

- ✅ 新增 17 个通信协议模块
- ✅ 完成 4 平台 HAL 层适配
- ✅ 全面的返回值检查和错误处理
- ✅ 代码规范统一化
- ✅ 完善的文档和示例

### v1.0.0 (2025-09-24)

- ✅ 初始版本
- ✅ 实现基础 10 个通信协议
- ✅ CMake 构建系统
- ✅ securec 安全库集成

---

**持续更新中... 欢迎 Star 和 Fork!** ⭐

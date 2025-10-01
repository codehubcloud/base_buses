# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded bus communication protocol library (嵌入式总线通信协议库) that provides modular implementations of common communication protocols for embedded systems. The project supports UART, SPI, I2C, I3C, CAN, Modbus, SMBus, RS232, and RS485.

## Build Commands

### Platform Selection

The project supports multiple hardware platforms through conditional compilation:
- **LINUX** (default) - For development and testing
- **STM32F4** - STM32F4xx series
- **STM32F1** - STM32F1xx series
- **ESP32** - ESP32 series

### Standard Build (Linux)
```bash
mkdir build && cd build
cmake ..
make
./bus_app
```

### Platform-Specific Builds
```bash
# STM32F4
cmake -DPLATFORM=STM32F4 ..

# STM32F1
cmake -DPLATFORM=STM32F1 ..

# ESP32
cmake -DPLATFORM=ESP32 ..

# Linux (explicit)
cmake -DPLATFORM=LINUX ..
```

### Build Type
```bash
# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build
cmake -DCMAKE_BUILD_TYPE=Release ..
```

## Architecture

### Hardware Abstraction Layer (HAL) Pattern

The codebase uses a strict separation between protocol logic and hardware-specific code:

- **Protocol Layer**: `xxx.c` and `xxx.h` - Contains protocol-agnostic business logic
- **HAL Layer**: `xxx_hal.c` and `xxx_hal.h` - Contains platform-specific hardware implementations

**Multi-Platform HAL Implementation:**
All HAL files now support 4 platforms through conditional compilation:
- STM32F4 (Cortex-M4)
- STM32F1 (Cortex-M3)
- ESP32 (Xtensa/RISC-V)
- Linux (for testing/development)

Each module follows this pattern consistently. The HAL automatically selects the correct platform implementation based on `PLATFORM_XXX` macros.

### Module Structure

Each protocol module is self-contained in `src/<protocol>/`:
- Static library build (e.g., `uart`, `spi`, `i2c`)
- Linked via `bus_modules` interface library
- Independent compilation and testing

### CMake Organization

1. Root `CMakeLists.txt` - Project configuration, includes securelib and src
2. `src/CMakeLists.txt` - Creates `bus_modules` interface library linking all protocol modules
3. Individual module `CMakeLists.txt` - Each builds a static library with HAL dependencies

### Dependency Management

- **securelib**: Third-party security library in `3rd_party/securelib/`
- All modules link against `securec` for safe string operations
- Include paths automatically propagated through interface library

## Coding Standards

- **Naming**: UpperCamelCase for functions (e.g., `UartInit`), lowerCamelCase for variables/parameters
- **Error Handling**: Always check function return values (0 = success, -1 = error)
- **Braces**: Required for all control statements
- **Comments**: Full Doxygen-style documentation for every function
- **Function Size**: Maximum 50 non-empty, non-comment lines per function
- **C Standard**: C99

## Key Implementation Notes

### Function Signatures Pattern

All modules follow consistent API patterns:
- `XxxInit()` - Initialize the peripheral (calls HAL clock/GPIO setup)
- `XxxSendData()` / `XxxWriteData()` - Transmit operations
- `XxxReceiveData()` / `XxxReadData()` - Receive operations
- `XxxSetBaudRate()` / `XxxSetClockSpeed()` - Configuration
- `XxxEnableInterrupts()` / `XxxDisableInterrupts()` - Interrupt control

### Testing

Each protocol has a corresponding `TestXxx()` function in `src/main.c` demonstrating typical usage patterns. The main function runs all tests sequentially.

### Multi-Platform Support Details

**Implemented Modules (9 total):**
1. UART - Universal Asynchronous Receiver-Transmitter
2. SPI - Serial Peripheral Interface
3. I2C - Inter-Integrated Circuit
4. CAN - Controller Area Network
5. I3C - Improved I3C (I2C compatibility mode)
6. RS232 - Standard serial communication
7. RS485 - Differential serial (with DE control)
8. Modbus - Industrial protocol (with CRC16)
9. SMBus - System Management Bus (with PEC/CRC8)

**Platform Configuration File:**
- `src/platform_config.h` - Central platform detection and configuration

**Documentation:**
- `多平台HAL实现说明.md` - Comprehensive platform porting guide
- `HAL实现完成总结.md` - Complete implementation summary
- `快速开始指南.md` - Quick start guide

### Porting to New Hardware

To add a new platform:
1. Add platform detection in `src/platform_config.h`
2. Add conditional compilation branches in each `xxx_hal.c` file
3. Update `CMakeLists.txt` to support the new platform
4. Keep protocol layer (`xxx.c`) unchanged

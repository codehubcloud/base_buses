# 测试目录 (Test Directory)

本目录包含所有27个通信协议模块的测试代码和示例程序。

## 文件说明

### main.c
包含所有模块的测试函数，用于验证各个通信协议模块的功能。

**测试函数列表:**

| 测试函数 | 模块 | 说明 |
|---------|------|------|
| `TestUart()` | UART | 通用异步收发器测试 |
| `TestSpi()` | SPI | 串行外设接口测试 |
| `TestI2c()` | I2C | 两线制总线测试 |
| `TestI3c()` | I3C | 改进型I3C测试 |
| `TestI2s()` | I2S | 音频数据接口测试 |
| `TestCan()` | CAN | 控制器局域网测试 |
| `TestCanFd()` | CAN FD | 灵活数据速率CAN测试 |
| `TestLin()` | LIN | 局域互连网络测试 |
| `TestFlexRay()` | FlexRay | 高速汽车总线测试 |
| `TestModbus()` | Modbus | 工业协议测试 |
| `TestSmBus()` | SMBus | 系统管理总线测试 |
| `TestPmBus()` | PMBus | 电源管理总线测试 |
| `TestRs232()` | RS232 | 标准串口测试 |
| `TestRs422()` | RS422 | 差分串口测试 |
| `TestRs485()` | RS485 | 多点通信总线测试 |
| `TestUsb()` | USB | 通用串行总线测试 |
| `TestSdio()` | SDIO | SD卡接口测试 |
| `TestOneWire()` | 1-Wire | 单总线协议测试 |
| `TestEthernet()` | Ethernet | 以太网测试 |
| `TestPcie()` | PCIe | 高速串行总线测试 |
| `TestEmmc()` | eMMC | 嵌入式存储测试 |
| `TestJtag()` | JTAG | 调试接口测试 |
| `TestSwd()` | SWD | 串行调试接口测试 |
| `TestAxi()` | AXI | ARM高级总线测试 |
| `TestAhb()` | AHB | 高性能总线测试 |
| `TestApb()` | APB | 外设总线测试 |
| `TestDma()` | DMA | 直接内存访问测试 |

## 编译运行

### 编译测试程序

```bash
# 进入构建目录
cd build

# 配置CMake
cmake ..

# 编译
make

# 运行测试
./bus_test
```

### 平台选择

```bash
# Linux平台 (默认)
cmake -DPLATFORM=LINUX ..

# STM32F4平台
cmake -DPLATFORM=STM32F4 ..

# STM32F1平台
cmake -DPLATFORM=STM32F1 ..

# ESP32平台
cmake -DPLATFORM=ESP32 ..
```

### 构建类型

```bash
# Debug构建 (包含调试信息)
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release构建 (优化)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

## 测试输出

运行测试程序后，会看到每个模块的测试输出:

```
=== Bus Communication Protocol Test ===

Testing UART...
UART initialized
UART test completed

Testing SPI...
SPI initialized
SPI test completed

Testing I2C...
I2C initialized
I2C test completed

...

All tests completed!
```

## 添加新测试

如果需要添加新的测试用例，请按照以下步骤:

1. 在 `main.c` 中添加测试函数:
   ```c
   void TestNewModule(void)
   {
       printf("Testing NewModule...\n");

       // 初始化
       NewModuleInit();

       // 测试功能
       // ...

       printf("NewModule test completed\n\n");
   }
   ```

2. 在 `main()` 函数中调用新测试:
   ```c
   TestNewModule();
   ```

3. 重新编译并运行:
   ```bash
   make
   ./bus_test
   ```

## 注意事项

1. **硬件依赖**: 某些测试可能需要实际的硬件设备才能完全运行
2. **平台限制**: 不同平台支持的功能可能不同
3. **权限要求**: Linux平台某些设备文件(如 `/dev/i2c-1`) 可能需要root权限
4. **设备可用性**: 确保设备文件存在且可访问

## 调试技巧

### 启用详细输出
修改测试函数，增加详细的状态输出:
```c
int32_t ret = UartInit();
printf("UartInit() returned: %d\n", ret);
```

### 单独测试某个模块
注释掉 `main()` 函数中的其他测试调用，只保留需要测试的模块:
```c
int main(void)
{
    printf("=== Bus Communication Protocol Test ===\n\n");

    // 只测试UART
    TestUart();

    printf("Test completed!\n");
    return 0;
}
```

### 使用调试器
```bash
# 使用gdb调试
gdb ./bus_test

# 设置断点
(gdb) break TestUart

# 运行
(gdb) run

# 单步执行
(gdb) step
```

## 测试覆盖率

当前测试覆盖了以下功能:

- ✅ 模块初始化/去初始化
- ✅ 数据发送/接收
- ✅ 参数配置 (波特率、时钟速度等)
- ✅ 状态查询
- ✅ 错误处理
- ✅ 平台适配验证

## 性能测试

如需进行性能测试，可以添加计时代码:

```c
#include <time.h>

void TestPerformance(void)
{
    clock_t start, end;
    double cpu_time_used;

    start = clock();

    // 执行测试操作
    for (int i = 0; i < 1000; i++) {
        UartSendData(0x55);
    }

    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

    printf("Time taken: %f seconds\n", cpu_time_used);
}
```

## 相关文档

- 主项目README: `../README.md`
- 开发指南: `../CLAUDE.md`
- 各模块源码: `../src/`

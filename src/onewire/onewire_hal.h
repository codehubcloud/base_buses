#ifndef ONEWIRE_HAL_H
#define ONEWIRE_HAL_H

#include <stdint.h>

/******************************************************************************
 * @brief     : 初始化1-Wire GPIO引脚 / Initialize 1-Wire GPIO pin
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation required
 *              配置GPIO为开漏模式（STM32）或带上拉的GPIO模式（ESP32/Linux）
 *              Configure GPIO as open-drain mode (STM32) or GPIO with pull-up (ESP32/Linux)
 *****************************************************************************/
int32_t OnewireInitGpio(void);

/******************************************************************************
 * @brief     : 设置1-Wire引脚为输出模式 / Set 1-Wire pin to output mode
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *              将GPIO配置为输出模式以发送数据
 *              Configure GPIO as output mode to send data
 *****************************************************************************/
void OnewireSetPinOutput(void);

/******************************************************************************
 * @brief     : 设置1-Wire引脚为输入模式 / Set 1-Wire pin to input mode
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *              将GPIO配置为输入模式以读取数据
 *              Configure GPIO as input mode to read data
 *****************************************************************************/
void OnewireSetPinInput(void);

/******************************************************************************
 * @brief     : 设置1-Wire引脚为高电平 / Set 1-Wire pin to high level
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *              在开漏模式下释放总线（由上拉电阻拉高）
 *              Release bus in open-drain mode (pulled high by pull-up resistor)
 *****************************************************************************/
void OnewireSetPinHigh(void);

/******************************************************************************
 * @brief     : 设置1-Wire引脚为低电平 / Set 1-Wire pin to low level
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *              拉低总线以发送逻辑0或复位信号
 *              Pull bus low to send logic 0 or reset signal
 *****************************************************************************/
void OnewireSetPinLow(void);

/******************************************************************************
 * @brief     : 读取1-Wire引脚电平 / Read 1-Wire pin level
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if high, 0 if low
 * @note      : Platform-specific implementation required
 *              读取总线当前电平状态
 *              Read current bus level state
 *****************************************************************************/
int32_t OnewireReadPin(void);

/******************************************************************************
 * @brief     : 微秒级延时 / Microsecond delay
 * @param[in] : us - 延时时间（微秒）/ Delay time in microseconds
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation required
 *              1-Wire协议需要精确的微秒级时序控制
 *              1-Wire protocol requires precise microsecond timing control
 *              标准模式时序要求：
 *              Standard mode timing requirements:
 *              - 复位脉冲: 480us / Reset pulse: 480us
 *              - 存在脉冲检测: 60-240us / Presence detect: 60-240us
 *              - 写0时间槽: 60us / Write 0 time slot: 60us
 *              - 写1时间槽: 1-15us / Write 1 time slot: 1-15us
 *              - 读时间槽: 1us拉低 + 14us采样 / Read slot: 1us low + 14us sample
 *****************************************************************************/
void OnewireDelayUs(uint32_t us);

#endif // ONEWIRE_HAL_H

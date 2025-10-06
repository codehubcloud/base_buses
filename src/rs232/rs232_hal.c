#include "platform_config.h"
#include "rs232_hal.h"
#include "securec.h"


/* NOTE: RS232 is implemented using UART physical layer
 * All platforms use their respective UART hardware
 */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static UART_HandleTypeDef g_rs232Handle;
#endif

#ifdef PLATFORM_STM32F1
static UART_HandleTypeDef g_rs232Handle;
#endif

#ifdef PLATFORM_ESP32
static const int g_rs232UartNum = UART_NUM_2; /* Using UART2 for RS232 */
#endif

#ifdef PLATFORM_LINUX
static int g_rs232Fd = -1;
#endif

/******************************************************************************
 * @brief     : Enable RS232 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs232EnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 UART clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure RS232 GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs232ConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* USART2 GPIO Configuration: PA2(TX), PA3(RX) */
    gpioInit.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* USART2 TX (PA2) */
    gpioInit.Pin = GPIO_PIN_2;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* USART2 RX (PA3) */
    gpioInit.Pin = GPIO_PIN_3;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by uart_param_config */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device files, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable RS232 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232Enable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE(&g_rs232Handle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE(&g_rs232Handle);
#elif defined(PLATFORM_ESP32)
    uart_driver_install(g_rs232UartNum, 256, 0, 0, NULL, 0);
#elif defined(PLATFORM_LINUX)
    if (g_rs232Fd < 0) {
        g_rs232Fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (g_rs232Fd >= 0) {
            fcntl(g_rs232Fd, F_SETFL, 0);
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Check if RS232 TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs232TxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_rs232Handle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_rs232Handle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1; /* ESP32 driver handles buffering */
#elif defined(PLATFORM_LINUX)
    return 1; /* Linux driver handles buffering */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write byte to RS232
 * @param[in] : data --Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232WriteByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    g_rs232Handle.Instance->DR = data;
#elif defined(PLATFORM_STM32F1)
    g_rs232Handle.Instance->DR = data;
#elif defined(PLATFORM_ESP32)
    uart_write_bytes(g_rs232UartNum, (const char*)&data, 1);
#elif defined(PLATFORM_LINUX)
    if (g_rs232Fd >= 0) {
        (void)write(g_rs232Fd, &data, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Check if RS232 RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs232RxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_rs232Handle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_rs232Handle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    size_t length = 0;
    uart_get_buffered_data_len(g_rs232UartNum, &length);
    return (length > 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_rs232Fd >= 0) {
        int bytes = 0;
        if (ioctl(g_rs232Fd, FIONREAD, &bytes) < 0) {
            return 0;
        }
        return (bytes > 0) ? 1 : 0;
    }
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Read byte from RS232
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from RS232
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t Rs232ReadByte(void)
{
#ifdef PLATFORM_STM32F4
    return (uint8_t)(g_rs232Handle.Instance->DR & 0xFF);
#elif defined(PLATFORM_STM32F1)
    return (uint8_t)(g_rs232Handle.Instance->DR & 0xFF);
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    uart_read_bytes(g_rs232UartNum, &data, 1, 0);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_rs232Fd >= 0) {
        (void)read(g_rs232Fd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Configure RS232 baud rate
 * @param[in] : baudRate --Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs232ConfigureBaudRate(uint32_t baudRate)
{
#ifdef PLATFORM_STM32F4
    g_rs232Handle.Instance = USART2;
    g_rs232Handle.Init.BaudRate = baudRate;
    g_rs232Handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_rs232Handle.Init.StopBits = UART_STOPBITS_1;
    g_rs232Handle.Init.Parity = UART_PARITY_NONE;
    g_rs232Handle.Init.Mode = UART_MODE_TX_RX;
    g_rs232Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_rs232Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    return (HAL_UART_Init(&g_rs232Handle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_rs232Handle.Instance = USART2;
    g_rs232Handle.Init.BaudRate = baudRate;
    g_rs232Handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_rs232Handle.Init.StopBits = UART_STOPBITS_1;
    g_rs232Handle.Init.Parity = UART_PARITY_NONE;
    g_rs232Handle.Init.Mode = UART_MODE_TX_RX;
    g_rs232Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    return (HAL_UART_Init(&g_rs232Handle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    uart_config_t uartConfig = {.baud_rate = (int)baudRate,
                                .data_bits = UART_DATA_8_BITS,
                                .parity = UART_PARITY_DISABLE,
                                .stop_bits = UART_STOP_BITS_1,
                                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(g_rs232UartNum, &uartConfig);
    uart_set_pin(g_rs232UartNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_rs232Fd < 0) {
        return -1;
    }

    struct termios options;
    tcgetattr(g_rs232Fd, &options);

    speed_t speed = B115200;
    if (baudRate == 9600) {
        speed = B9600;
    } else if (baudRate == 19200) {
        speed = B19200;
    } else if (baudRate == 38400) {
        speed = B38400;
    } else if (baudRate == 57600) {
        speed = B57600;
    } else if (baudRate == 115200) {
        speed = B115200;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    tcsetattr(g_rs232Fd, TCSANOW, &options);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable RS232 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232EnableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_rs232Handle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_rs232Handle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue for interrupts */
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll mechanisms */
#endif
}

/******************************************************************************
 * @brief     : Enable RS232 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232EnableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_rs232Handle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_rs232Handle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue for interrupts */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use TX interrupts in userspace */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for RS232
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232ConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

/******************************************************************************
 * @brief     : Disable RS232 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232DisableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_rs232Handle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_rs232Handle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable RS232 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232DisableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_rs232Handle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_rs232Handle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs232UpdateNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_DisableIRQ(USART2_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_DisableIRQ(USART2_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}
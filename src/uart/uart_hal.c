#include "securec.h"
#include "uart_hal.h"
#include "platform_config.h"

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static UART_HandleTypeDef g_uartHandle;
#endif

#ifdef PLATFORM_STM32F1
static UART_HandleTypeDef g_uartHandle;
#endif

#ifdef PLATFORM_ESP32
static const int g_uartNum = UART_NUM_DEFAULT;
#endif

#ifdef PLATFORM_LINUX
static int g_uartFd = -1;
#endif

/******************************************************************************
 * @brief     : Enable UART clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UartEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USART1_CLK_ENABLE();
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
 * @brief     : Configure UART GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UartConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* USART1 GPIO Configuration: PA9(TX), PA10(RX) */
    gpioInit.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* USART1 TX (PA9) */
    gpioInit.Pin = GPIO_PIN_9;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* USART1 RX (PA10) */
    gpioInit.Pin = GPIO_PIN_10;
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
 * @brief     : Enable UART module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartEnable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE(&g_uartHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE(&g_uartHandle);
#elif defined(PLATFORM_ESP32)
    uart_driver_install(g_uartNum, 256, 0, 0, NULL, 0);
#elif defined(PLATFORM_LINUX)
    if (g_uartFd < 0) {
        g_uartFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (g_uartFd >= 0) {
            fcntl(g_uartFd, F_SETFL, 0);
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Check if UART TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UartTxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_uartHandle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_uartHandle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1;  /* ESP32 driver handles buffering */
#elif defined(PLATFORM_LINUX)
    return 1;  /* Linux driver handles buffering */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write byte to UART
 * @param[in] : data - Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartWriteByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    g_uartHandle.Instance->DR = data;
#elif defined(PLATFORM_STM32F1)
    g_uartHandle.Instance->DR = data;
#elif defined(PLATFORM_ESP32)
    uart_write_bytes(g_uartNum, (const char*)&data, 1);
#elif defined(PLATFORM_LINUX)
    if (g_uartFd >= 0) {
        write(g_uartFd, &data, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Check if UART RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UartRxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_uartHandle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_uartHandle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    size_t length = 0;
    uart_get_buffered_data_len(g_uartNum, &length);
    return (length > 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_uartFd >= 0) {
        int bytes = 0;
        ioctl(g_uartFd, FIONREAD, &bytes);
        return (bytes > 0) ? 1 : 0;
    }
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Read byte from UART
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from UART
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t UartReadByte(void)
{
#ifdef PLATFORM_STM32F4
    return (uint8_t)(g_uartHandle.Instance->DR & 0xFF);
#elif defined(PLATFORM_STM32F1)
    return (uint8_t)(g_uartHandle.Instance->DR & 0xFF);
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    uart_read_bytes(g_uartNum, &data, 1, 0);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_uartFd >= 0) {
        (void)read(g_uartFd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Configure UART baud rate
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t UartConfigureBaudRate(uint32_t baudRate)
{
#ifdef PLATFORM_STM32F4
    g_uartHandle.Instance = USART1;
    g_uartHandle.Init.BaudRate = baudRate;
    g_uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uartHandle.Init.StopBits = UART_STOPBITS_1;
    g_uartHandle.Init.Parity = UART_PARITY_NONE;
    g_uartHandle.Init.Mode = UART_MODE_TX_RX;
    g_uartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_uartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
    return (HAL_UART_Init(&g_uartHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_uartHandle.Instance = USART1;
    g_uartHandle.Init.BaudRate = baudRate;
    g_uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    g_uartHandle.Init.StopBits = UART_STOPBITS_1;
    g_uartHandle.Init.Parity = UART_PARITY_NONE;
    g_uartHandle.Init.Mode = UART_MODE_TX_RX;
    g_uartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    return (HAL_UART_Init(&g_uartHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    uart_config_t uartConfig = {
        .baud_rate = (int)baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(g_uartNum, &uartConfig);
    uart_set_pin(g_uartNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_uartFd < 0) {
        return -1;
    }

    struct termios options;
    tcgetattr(g_uartFd, &options);

    speed_t speed = B115200;
    if (baudRate == 9600) { speed = B9600; }
    else if (baudRate == 19200) { speed = B19200; }
    else if (baudRate == 38400) { speed = B38400; }
    else if (baudRate == 57600) { speed = B57600; }
    else if (baudRate == 115200) { speed = B115200; }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    tcsetattr(g_uartFd, TCSANOW, &options);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable UART RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartEnableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_uartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_uartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue for interrupts */
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll mechanisms */
#endif
}

/******************************************************************************
 * @brief     : Enable UART TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartEnableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_uartHandle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_uartHandle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue for interrupts */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use TX interrupts in userspace */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for UART
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

/******************************************************************************
 * @brief     : Disable UART RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartDisableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_uartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_uartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    /* ESP32 uses event queue */
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable UART TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void UartDisableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_uartHandle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_uartHandle, UART_IT_TXE);
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
void UartUpdateNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_DisableIRQ(USART1_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_DisableIRQ(USART1_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

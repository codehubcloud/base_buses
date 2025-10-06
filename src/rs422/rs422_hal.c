#include "platform_config.h"
#include "rs422_hal.h"
#include "securec.h"


/* NOTE: RS422 uses differential signaling (TX+/TX-, RX+/RX-) for full-duplex communication
 * Unlike RS485, RS422 does NOT require DE (Driver Enable) control
 * All platforms use USART/UART with external RS422 transceivers
 */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static UART_HandleTypeDef g_rs422Handle;
#endif

#ifdef PLATFORM_STM32F1
static UART_HandleTypeDef g_rs422Handle;
#endif

#ifdef PLATFORM_ESP32
static const int g_rs422UartNum = UART_NUM_1; /* Using UART1 for RS422 */
#endif

#ifdef PLATFORM_LINUX
static int g_rs422Fd = -1;
static struct termios g_rs422OldTermios;
#endif

/******************************************************************************
 * @brief     : Enable RS422 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs422EnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
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
 * @brief     : Disable RS422 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422DisableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USART3_CLK_DISABLE();
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USART3_CLK_DISABLE();
#elif defined(PLATFORM_ESP32)
    /* ESP32 clock managed by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock disabling */
#endif
}

/******************************************************************************
 * @brief     : Configure RS422 GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs422ConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* USART3 GPIO Configuration: PB10(TX), PB11(RX) */
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* USART3 TX (PB10) */
    gpioInit.Pin = GPIO_PIN_10;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    /* USART3 RX (PB11) */
    gpioInit.Pin = GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by uart_param_config and uart_set_pin */
    uart_config_t uartConfig = {.baud_rate = 9600,
                                .data_bits = UART_DATA_8_BITS,
                                .parity = UART_PARITY_DISABLE,
                                .stop_bits = UART_STOP_BITS_1,
                                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(g_rs422UartNum, &uartConfig);
    /* Configure GPIO pins for RS422: TX=GPIO17, RX=GPIO16 */
    uart_set_pin(g_rs422UartNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device files, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable RS422 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422Enable(void)
{
#ifdef PLATFORM_STM32F4
    g_rs422Handle.Instance = USART3;
    g_rs422Handle.Init.BaudRate = 9600;
    g_rs422Handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_rs422Handle.Init.StopBits = UART_STOPBITS_1;
    g_rs422Handle.Init.Parity = UART_PARITY_NONE;
    g_rs422Handle.Init.Mode = UART_MODE_TX_RX;
    g_rs422Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_rs422Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&g_rs422Handle);
    __HAL_UART_ENABLE(&g_rs422Handle);
#elif defined(PLATFORM_STM32F1)
    g_rs422Handle.Instance = USART3;
    g_rs422Handle.Init.BaudRate = 9600;
    g_rs422Handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_rs422Handle.Init.StopBits = UART_STOPBITS_1;
    g_rs422Handle.Init.Parity = UART_PARITY_NONE;
    g_rs422Handle.Init.Mode = UART_MODE_TX_RX;
    g_rs422Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&g_rs422Handle);
    __HAL_UART_ENABLE(&g_rs422Handle);
#elif defined(PLATFORM_ESP32)
    uart_driver_install(g_rs422UartNum, 256, 256, 0, NULL, 0);
#elif defined(PLATFORM_LINUX)
    struct termios newTermios;

    if (g_rs422Fd < 0) {
        g_rs422Fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (g_rs422Fd < 0) {
            return;
        }
    }

    tcgetattr(g_rs422Fd, &g_rs422OldTermios);
    if (memset_s(&newTermios, sizeof(newTermios), 0, sizeof(newTermios)) != EOK) {
        return;
    }

    /* Configure for raw mode with RS422 settings */
    newTermios.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    newTermios.c_iflag = IGNPAR;
    newTermios.c_oflag = 0;
    newTermios.c_lflag = 0;
    newTermios.c_cc[VTIME] = 0;
    newTermios.c_cc[VMIN] = 0;

    tcflush(g_rs422Fd, TCIFLUSH);
    tcsetattr(g_rs422Fd, TCSANOW, &newTermios);

/* Set RS422 mode using ioctl (if supported by driver) */
#ifdef TIOCGRS485
    struct serial_rs485 rs422Config;
    if (memset_s(&rs422Config, sizeof(rs422Config), 0, sizeof(rs422Config)) != EOK) {
        return;
    }
    /* RS422 is full-duplex, no special flags needed */
    rs422Config.flags = 0;
    (void)ioctl(g_rs422Fd, TIOCSRS485, &rs422Config);
#endif

    fcntl(g_rs422Fd, F_SETFL, FNDELAY);
#endif
}

/******************************************************************************
 * @brief     : Disable RS422 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422Disable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE(&g_rs422Handle);
    HAL_UART_DeInit(&g_rs422Handle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE(&g_rs422Handle);
    HAL_UART_DeInit(&g_rs422Handle);
#elif defined(PLATFORM_ESP32)
    uart_driver_delete(g_rs422UartNum);
#elif defined(PLATFORM_LINUX)
    if (g_rs422Fd >= 0) {
        tcsetattr(g_rs422Fd, TCSANOW, &g_rs422OldTermios);
        close(g_rs422Fd);
        g_rs422Fd = -1;
    }
#endif
}

/******************************************************************************
 * @brief     : Check if RS422 TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs422TxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_rs422Handle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_rs422Handle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1; /* ESP32 driver handles buffering */
#elif defined(PLATFORM_LINUX)
    return 1; /* Linux driver handles buffering */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write byte to RS422
 * @param[in] : data --Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422WriteByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    g_rs422Handle.Instance->DR = data;
#elif defined(PLATFORM_STM32F1)
    g_rs422Handle.Instance->DR = data;
#elif defined(PLATFORM_ESP32)
    uart_write_bytes(g_rs422UartNum, (const char*)&data, 1);
#elif defined(PLATFORM_LINUX)
    if (g_rs422Fd >= 0) {
        (void)write(g_rs422Fd, &data, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Check if RS422 RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs422RxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_rs422Handle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_rs422Handle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    size_t availableBytes = 0;
    uart_get_buffered_data_len(g_rs422UartNum, &availableBytes);
    return (availableBytes > 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_rs422Fd >= 0) {
        int bytesAvailable = 0;
        if (ioctl(g_rs422Fd, FIONREAD, &bytesAvailable) < 0) {
            return 0;
        }
        return (bytesAvailable > 0) ? 1 : 0;
    }
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Read byte from RS422
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from RS422
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t Rs422ReadByte(void)
{
#ifdef PLATFORM_STM32F4
    return (uint8_t)(g_rs422Handle.Instance->DR & 0xFF);
#elif defined(PLATFORM_STM32F1)
    return (uint8_t)(g_rs422Handle.Instance->DR & 0xFF);
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    uart_read_bytes(g_rs422UartNum, &data, 1, 0);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_rs422Fd >= 0) {
        (void)read(g_rs422Fd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Configure RS422 baud rate
 * @param[in] : baudRate --Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs422ConfigureBaudRate(uint32_t baudRate)
{
#ifdef PLATFORM_STM32F4
    g_rs422Handle.Init.BaudRate = baudRate;
    HAL_UART_Init(&g_rs422Handle);
    return 0;
#elif defined(PLATFORM_STM32F1)
    g_rs422Handle.Init.BaudRate = baudRate;
    HAL_UART_Init(&g_rs422Handle);
    return 0;
#elif defined(PLATFORM_ESP32)
    uart_set_baudrate(g_rs422UartNum, baudRate);
    return 0;
#elif defined(PLATFORM_LINUX)
    struct termios options;
    speed_t speed;

    if (g_rs422Fd < 0) {
        return -1;
    }

    /* Map baud rate to speed_t */
    switch (baudRate) {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            return -1;
    }

    tcgetattr(g_rs422Fd, &options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    tcsetattr(g_rs422Fd, TCSANOW, &options);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure RS422 data format
 * @param[in] : dataBits --Data bits (7 or 8), parity - Parity mode, stopBits - Stop bits (1 or 2)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs422ConfigureDataFormat(uint8_t dataBits, uint8_t parity, uint8_t stopBits)
{
#ifdef PLATFORM_STM32F4
    /* Configure data bits */
    if (dataBits == 8) {
        g_rs422Handle.Init.WordLength = (parity == 0) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
    } else if (dataBits == 7) {
        g_rs422Handle.Init.WordLength = (parity == 0) ? UART_WORDLENGTH_7B : UART_WORDLENGTH_8B;
    } else {
        return -1;
    }

    /* Configure parity */
    if (parity == 0) {
        g_rs422Handle.Init.Parity = UART_PARITY_NONE;
    } else if (parity == 1) {
        g_rs422Handle.Init.Parity = UART_PARITY_ODD;
    } else if (parity == 2) {
        g_rs422Handle.Init.Parity = UART_PARITY_EVEN;
    } else {
        return -1;
    }

    /* Configure stop bits */
    if (stopBits == 1) {
        g_rs422Handle.Init.StopBits = UART_STOPBITS_1;
    } else if (stopBits == 2) {
        g_rs422Handle.Init.StopBits = UART_STOPBITS_2;
    } else {
        return -1;
    }

    HAL_UART_Init(&g_rs422Handle);
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Configure data bits */
    if (dataBits == 8) {
        g_rs422Handle.Init.WordLength = (parity == 0) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
    } else if (dataBits == 7) {
        g_rs422Handle.Init.WordLength = UART_WORDLENGTH_8B;
    } else {
        return -1;
    }

    /* Configure parity */
    if (parity == 0) {
        g_rs422Handle.Init.Parity = UART_PARITY_NONE;
    } else if (parity == 1) {
        g_rs422Handle.Init.Parity = UART_PARITY_ODD;
    } else if (parity == 2) {
        g_rs422Handle.Init.Parity = UART_PARITY_EVEN;
    } else {
        return -1;
    }

    /* Configure stop bits */
    if (stopBits == 1) {
        g_rs422Handle.Init.StopBits = UART_STOPBITS_1;
    } else if (stopBits == 2) {
        g_rs422Handle.Init.StopBits = UART_STOPBITS_2;
    } else {
        return -1;
    }

    HAL_UART_Init(&g_rs422Handle);
    return 0;
#elif defined(PLATFORM_ESP32)
    uart_word_length_t wordLength;
    uart_parity_t parityMode;
    uart_stop_bits_t stopBitsMode;

    /* Configure data bits */
    if (dataBits == 8) {
        wordLength = UART_DATA_8_BITS;
    } else if (dataBits == 7) {
        wordLength = UART_DATA_7_BITS;
    } else {
        return -1;
    }

    /* Configure parity */
    if (parity == 0) {
        parityMode = UART_PARITY_DISABLE;
    } else if (parity == 1) {
        parityMode = UART_PARITY_ODD;
    } else if (parity == 2) {
        parityMode = UART_PARITY_EVEN;
    } else {
        return -1;
    }

    /* Configure stop bits */
    if (stopBits == 1) {
        stopBitsMode = UART_STOP_BITS_1;
    } else if (stopBits == 2) {
        stopBitsMode = UART_STOP_BITS_2;
    } else {
        return -1;
    }

    uart_set_word_length(g_rs422UartNum, wordLength);
    uart_set_parity(g_rs422UartNum, parityMode);
    uart_set_stop_bits(g_rs422UartNum, stopBitsMode);
    return 0;
#elif defined(PLATFORM_LINUX)
    struct termios options;

    if (g_rs422Fd < 0) {
        return -1;
    }

    tcgetattr(g_rs422Fd, &options);

    /* Configure data bits */
    options.c_cflag &= ~CSIZE;
    if (dataBits == 8) {
        options.c_cflag |= CS8;
    } else if (dataBits == 7) {
        options.c_cflag |= CS7;
    } else {
        return -1;
    }

    /* Configure parity */
    if (parity == 0) {
        options.c_cflag &= ~PARENB;
    } else if (parity == 1) {
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
    } else if (parity == 2) {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
    } else {
        return -1;
    }

    /* Configure stop bits */
    if (stopBits == 1) {
        options.c_cflag &= ~CSTOPB;
    } else if (stopBits == 2) {
        options.c_cflag |= CSTOPB;
    } else {
        return -1;
    }

    tcsetattr(g_rs422Fd, TCSANOW, &options);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable RS422 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422EnableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_rs422Handle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_rs422Handle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    uart_enable_rx_intr(g_rs422UartNum);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll for interrupt-like behavior */
#endif
}

/******************************************************************************
 * @brief     : Enable RS422 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422EnableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_rs422Handle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_rs422Handle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    uart_enable_tx_intr(g_rs422UartNum, 1, 0);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll for interrupt-like behavior */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for RS422
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422ConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is managed by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use NVIC */
#endif
}

/******************************************************************************
 * @brief     : Disable RS422 RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422DisableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_rs422Handle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_rs422Handle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    uart_disable_rx_intr(g_rs422UartNum);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll */
#endif
}

/******************************************************************************
 * @brief     : Disable RS422 TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422DisableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_rs422Handle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_rs422Handle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    uart_disable_tx_intr(g_rs422UartNum);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll */
#endif
}

/******************************************************************************
 * @brief     : Update NVIC configuration
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs422UpdateNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_DisableIRQ(USART3_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_DisableIRQ(USART3_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is managed by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use NVIC */
#endif
}

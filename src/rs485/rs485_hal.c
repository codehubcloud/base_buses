#include "platform_config.h"
#include "rs485_hal.h"
#include "securec.h"


/* NOTE: RS485 is implemented using UART with DE (Driver Enable) pin control
 * DE pin HIGH = Transmit mode, DE pin LOW = Receive mode
 * Using PA1 as DE pin for STM32, GPIO_NUM_4 for ESP32
 */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static UART_HandleTypeDef g_rs485Handle;
#define RS485_DE_PIN GPIO_PIN_1
#define RS485_DE_PORT GPIOA
#endif

#ifdef PLATFORM_STM32F1
static UART_HandleTypeDef g_rs485Handle;
#define RS485_DE_PIN GPIO_PIN_1
#define RS485_DE_PORT GPIOA
#endif

#ifdef PLATFORM_ESP32
static const int g_rs485UartNum = UART_NUM_2;
#define RS485_DE_PIN GPIO_NUM_4
#endif

#ifdef PLATFORM_LINUX
static int g_rs485Fd = -1;
#endif

/******************************************************************************
 * @brief     : Enable RS485 clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs485EnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
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
 * @brief     : Configure RS485 GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs485ConfigureGpio(void)
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

    /* DE pin configuration (PA1) */
    gpioInit.Pin = RS485_DE_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_PORT, &gpioInit);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET); /* Default to RX mode */
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

    /* DE pin configuration (PA1) */
    gpioInit.Pin = RS485_DE_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_PORT, &gpioInit);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET); /* Default to RX mode */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by uart_param_config */
    gpio_config_t io_conf = {.intr_type = GPIO_INTR_DISABLE,
                             .mode = GPIO_MODE_OUTPUT,
                             .pin_bit_mask = (1ULL << RS485_DE_PIN),
                             .pull_down_en = 0,
                             .pull_up_en = 0};
    gpio_config(&io_conf);
    gpio_set_level(RS485_DE_PIN, 0); /* Default to RX mode */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device files, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable RS485 module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs485Enable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE(&g_rs485Handle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE(&g_rs485Handle);
#elif defined(PLATFORM_ESP32)
    uart_driver_install(g_rs485UartNum, 256, 0, 0, NULL, 0);
#elif defined(PLATFORM_LINUX)
    if (g_rs485Fd < 0) {
        g_rs485Fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY);
        if (g_rs485Fd >= 0) {
            fcntl(g_rs485Fd, F_SETFL, 0);
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Check if RS485 TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs485TxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_rs485Handle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_rs485Handle, UART_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1; /* ESP32 driver handles buffering */
#elif defined(PLATFORM_LINUX)
    return 1; /* Linux driver handles buffering */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write byte to RS485
 * @param[in] : data --Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void Rs485WriteByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    g_rs485Handle.Instance->DR = data;
#elif defined(PLATFORM_STM32F1)
    g_rs485Handle.Instance->DR = data;
#elif defined(PLATFORM_ESP32)
    uart_write_bytes(g_rs485UartNum, (const char*)&data, 1);
#elif defined(PLATFORM_LINUX)
    if (g_rs485Fd >= 0) {
        (void)write(g_rs485Fd, &data, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Check if RS485 RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs485RxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_rs485Handle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_rs485Handle, UART_FLAG_RXNE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    size_t length = 0;
    uart_get_buffered_data_len(g_rs485UartNum, &length);
    return (length > 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_rs485Fd >= 0) {
        int bytes = 0;
        if (ioctl(g_rs485Fd, FIONREAD, &bytes) < 0) {
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
 * @brief     : Read byte from RS485
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from RS485
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t Rs485ReadByte(void)
{
#ifdef PLATFORM_STM32F4
    return (uint8_t)(g_rs485Handle.Instance->DR & 0xFF);
#elif defined(PLATFORM_STM32F1)
    return (uint8_t)(g_rs485Handle.Instance->DR & 0xFF);
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    uart_read_bytes(g_rs485UartNum, &data, 1, 0);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_rs485Fd >= 0) {
        (void)read(g_rs485Fd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Configure RS485 baud rate
 * @param[in] : baudRate --Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t Rs485ConfigureBaudRate(uint32_t baudRate)
{
#ifdef PLATFORM_STM32F4
    g_rs485Handle.Instance = USART3;
    g_rs485Handle.Init.BaudRate = baudRate;
    g_rs485Handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_rs485Handle.Init.StopBits = UART_STOPBITS_1;
    g_rs485Handle.Init.Parity = UART_PARITY_NONE;
    g_rs485Handle.Init.Mode = UART_MODE_TX_RX;
    g_rs485Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_rs485Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    return (HAL_UART_Init(&g_rs485Handle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_rs485Handle.Instance = USART3;
    g_rs485Handle.Init.BaudRate = baudRate;
    g_rs485Handle.Init.WordLength = UART_WORDLENGTH_8B;
    g_rs485Handle.Init.StopBits = UART_STOPBITS_1;
    g_rs485Handle.Init.Parity = UART_PARITY_NONE;
    g_rs485Handle.Init.Mode = UART_MODE_TX_RX;
    g_rs485Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    return (HAL_UART_Init(&g_rs485Handle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    uart_config_t uartConfig = {.baud_rate = (int)baudRate,
                                .data_bits = UART_DATA_8_BITS,
                                .parity = UART_PARITY_DISABLE,
                                .stop_bits = UART_STOP_BITS_1,
                                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(g_rs485UartNum, &uartConfig);
    uart_set_pin(g_rs485UartNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_rs485Fd < 0) {
        return -1;
    }

    struct termios options;
    tcgetattr(g_rs485Fd, &options);

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

    tcsetattr(g_rs485Fd, TCSANOW, &options);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Set RS485 to transmit mode
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation (DE pin HIGH)
 *****************************************************************************/
void Rs485SetTxMode(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(RS485_DE_PIN, 1);
#elif defined(PLATFORM_LINUX)
    if (g_rs485Fd >= 0) {
        int status;
        ioctl(g_rs485Fd, TIOCMGET, &status);
        status |= TIOCM_RTS;
        ioctl(g_rs485Fd, TIOCMSET, &status);
    }
#endif
}

/******************************************************************************
 * @brief     : Set RS485 to receive mode
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation (DE pin LOW)
 *****************************************************************************/
void Rs485SetRxMode(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(RS485_DE_PIN, 0);
#elif defined(PLATFORM_LINUX)
    if (g_rs485Fd >= 0) {
        int status;
        ioctl(g_rs485Fd, TIOCMGET, &status);
        status &= ~TIOCM_RTS;
        ioctl(g_rs485Fd, TIOCMSET, &status);
    }
#endif
}
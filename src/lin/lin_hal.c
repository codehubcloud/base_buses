#include "lin_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static UART_HandleTypeDef g_linUartHandle;
#define LIN_UART_INSTANCE USART2
#define LIN_UART_IRQn USART2_IRQn
#define LIN_GPIO_PORT GPIOA
#define LIN_TX_PIN GPIO_PIN_2
#define LIN_RX_PIN GPIO_PIN_3
#define LIN_GPIO_AF GPIO_AF7_USART2
#endif

#ifdef PLATFORM_STM32F1
static UART_HandleTypeDef g_linUartHandle;
#define LIN_UART_INSTANCE USART2
#define LIN_UART_IRQn USART2_IRQn
#define LIN_GPIO_PORT GPIOA
#define LIN_TX_PIN GPIO_PIN_2
#define LIN_RX_PIN GPIO_PIN_3
#endif

#ifdef PLATFORM_ESP32
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#define LIN_UART_NUM UART_NUM_2
#define LIN_TX_PIN GPIO_NUM_17
#define LIN_RX_PIN GPIO_NUM_16
#define LIN_BUF_SIZE 256
static QueueHandle_t g_linUartQueue;
#endif

#ifdef PLATFORM_LINUX
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static int g_linFd = -1;
static struct termios g_oldTermios;
#endif

/******************************************************************************
 * @brief     : Enable LIN UART clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinEnableClock(void)
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
 * @brief     : Configure LIN GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* LIN UART GPIO Configuration: PA2(TX), PA3(RX) */
    gpioInit.Pin = LIN_TX_PIN | LIN_RX_PIN;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioInit.Alternate = LIN_GPIO_AF;
    HAL_GPIO_Init(LIN_GPIO_PORT, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* LIN UART TX (PA2) */
    gpioInit.Pin = LIN_TX_PIN;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LIN_GPIO_PORT, &gpioInit);

    /* LIN UART RX (PA3) */
    gpioInit.Pin = LIN_RX_PIN;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(LIN_GPIO_PORT, &gpioInit);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by uart_driver_install */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device files, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure LIN UART peripheral
 * @param[in] : baudRate - Desired baud rate value
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinConfigureUart(uint32_t baudRate)
{
#ifdef PLATFORM_STM32F4
    g_linUartHandle.Instance = LIN_UART_INSTANCE;
    g_linUartHandle.Init.BaudRate = baudRate;
    g_linUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    g_linUartHandle.Init.StopBits = UART_STOPBITS_1;
    g_linUartHandle.Init.Parity = UART_PARITY_NONE;
    g_linUartHandle.Init.Mode = UART_MODE_TX_RX;
    g_linUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_linUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    /* Enable LIN mode if supported */
    g_linUartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    return (HAL_UART_Init(&g_linUartHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_linUartHandle.Instance = LIN_UART_INSTANCE;
    g_linUartHandle.Init.BaudRate = baudRate;
    g_linUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    g_linUartHandle.Init.StopBits = UART_STOPBITS_1;
    g_linUartHandle.Init.Parity = UART_PARITY_NONE;
    g_linUartHandle.Init.Mode = UART_MODE_TX_RX;
    g_linUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    return (HAL_UART_Init(&g_linUartHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    uart_config_t uartConfig = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    if (uart_param_config(LIN_UART_NUM, &uartConfig) != ESP_OK) {
        return -1;
    }

    if (uart_set_pin(LIN_UART_NUM, LIN_TX_PIN, LIN_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return -1;
    }

    if (uart_driver_install(LIN_UART_NUM, LIN_BUF_SIZE * 2, LIN_BUF_SIZE * 2, 10, &g_linUartQueue, 0) != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    struct termios tty;

    /* Open serial port */
    g_linFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_linFd < 0) {
        /* Try alternative device */
        g_linFd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (g_linFd < 0) {
            return -1;
        }
    }

    /* Save old settings */
    if (tcgetattr(g_linFd, &g_oldTermios) != 0) {
        close(g_linFd);
        g_linFd = -1;
        return -1;
    }

    /* Configure terminal */
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(g_linFd, &tty) != 0) {
        close(g_linFd);
        g_linFd = -1;
        return -1;
    }

    /* Set baud rate */
    speed_t speed;
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
        default:
            speed = B19200;
            break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* 8N1 mode */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    /* Raw mode */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    /* Read settings */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; /* 0.1 second timeout */

    if (tcsetattr(g_linFd, TCSANOW, &tty) != 0) {
        close(g_linFd);
        g_linFd = -1;
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable LIN UART module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinEnable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE(&g_linUartHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE(&g_linUartHandle);
#elif defined(PLATFORM_ESP32)
    /* ESP32 UART is enabled by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux UART is enabled after configuration */
#endif
}

/******************************************************************************
 * @brief     : Disable LIN UART module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinDisable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE(&g_linUartHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE(&g_linUartHandle);
#elif defined(PLATFORM_ESP32)
    uart_driver_delete(LIN_UART_NUM);
#elif defined(PLATFORM_LINUX)
    if (g_linFd >= 0) {
        tcsetattr(g_linFd, TCSANOW, &g_oldTermios);
        close(g_linFd);
        g_linFd = -1;
    }
#endif
}

/******************************************************************************
 * @brief     : Send LIN break field (13-bit dominant)
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific break generation
 *****************************************************************************/
int32_t LinHalSendBreak(void)
{
#ifdef PLATFORM_STM32F4
    /* STM32F4 UART supports LIN break generation */
    /* Send break using HAL LIN mode or manual method */

    /* Manual method: Send 0x00 at lower baud rate for 13-bit break */
    uint32_t originalBaud = g_linUartHandle.Init.BaudRate;

    /* Temporarily reduce baud rate to create longer low period */
    g_linUartHandle.Init.BaudRate = originalBaud * 8 / 13;
    HAL_UART_Init(&g_linUartHandle);

    uint8_t breakByte = 0x00;
    HAL_UART_Transmit(&g_linUartHandle, &breakByte, 1, 100);

    /* Wait for transmission complete */
    while (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_TC) == RESET) {
    }

    /* Restore original baud rate */
    g_linUartHandle.Init.BaudRate = originalBaud;
    HAL_UART_Init(&g_linUartHandle);

    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STM32F1 manual break generation */
    uint32_t originalBaud = g_linUartHandle.Init.BaudRate;

    /* Temporarily reduce baud rate */
    g_linUartHandle.Init.BaudRate = originalBaud * 8 / 13;
    HAL_UART_Init(&g_linUartHandle);

    uint8_t breakByte = 0x00;
    HAL_UART_Transmit(&g_linUartHandle, &breakByte, 1, 100);

    /* Wait for transmission complete */
    while (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_TC) == RESET) {
    }

    /* Restore original baud rate */
    g_linUartHandle.Init.BaudRate = originalBaud;
    HAL_UART_Init(&g_linUartHandle);

    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 manual break generation using UART break control */
    /* Send break by transmitting 0x00 at reduced baud rate */
    uart_config_t uartConfig;
    uart_get_baudrate(LIN_UART_NUM, &uartConfig.baud_rate);
    uint32_t originalBaud = uartConfig.baud_rate;

    /* Set lower baud rate for break */
    uart_set_baudrate(LIN_UART_NUM, originalBaud * 8 / 13);

    uint8_t breakByte = 0x00;
    uart_write_bytes(LIN_UART_NUM, (const char*)&breakByte, 1);
    uart_wait_tx_done(LIN_UART_NUM, 100 / portTICK_PERIOD_MS);

    /* Restore baud rate */
    uart_set_baudrate(LIN_UART_NUM, originalBaud);

    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_linFd < 0) {
        return -1;
    }

    /* Send BREAK signal using ioctl */
    if (tcsendbreak(g_linFd, 0) != 0) {
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Write byte to LIN UART
 * @param[in] : data - Byte to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinWriteByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    return (HAL_UART_Transmit(&g_linUartHandle, &data, 1, 100) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    return (HAL_UART_Transmit(&g_linUartHandle, &data, 1, 100) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    int written = uart_write_bytes(LIN_UART_NUM, (const char*)&data, 1);
    return (written == 1) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_linFd < 0) {
        return -1;
    }
    ssize_t written = write(g_linFd, &data, 1);
    return (written == 1) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read byte from LIN UART
 * @param[in] : None
 * @param[out]: data - Pointer to store read byte
 * @return    : 0 if success, -1 if error/timeout
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinReadByte(uint8_t* data)
{
    if (data == NULL) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    return (HAL_UART_Receive(&g_linUartHandle, data, 1, 100) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    return (HAL_UART_Receive(&g_linUartHandle, data, 1, 100) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    int len = uart_read_bytes(LIN_UART_NUM, data, 1, 100 / portTICK_PERIOD_MS);
    return (len == 1) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_linFd < 0) {
        return -1;
    }
    ssize_t n = read(g_linFd, data, 1);
    if (n < 0) {
        return -1;
    }
    return (n == 1) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Check if LIN TX buffer is empty
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if empty, 0 if not empty
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinTxBufferEmpty(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_TXE) == SET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_TXE) == SET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    size_t size = 0;
    uart_get_buffered_data_len(LIN_UART_NUM, &size);
    return (size == 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_linFd < 0) {
        return 1;
    }
    int bytes = 0;
    if (ioctl(g_linFd, TIOCOUTQ, &bytes) < 0) {
        return 1;
    }
    return (bytes == 0) ? 1 : 0;
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Check if LIN RX buffer has data
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if has data, 0 if no data
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t LinRxBufferHasData(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_RXNE) == SET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_RXNE) == SET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    size_t size = 0;
    uart_get_buffered_data_len(LIN_UART_NUM, &size);
    return (size > 0) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    if (g_linFd < 0) {
        return 0;
    }
    int bytes = 0;
    if (ioctl(g_linFd, FIONREAD, &bytes) < 0) {
        return 0;
    }
    return (bytes > 0) ? 1 : 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Flush LIN RX buffer
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinFlushRxBuffer(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_FLUSH_DRREGISTER(&g_linUartHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_FLUSH_DRREGISTER(&g_linUartHandle);
#elif defined(PLATFORM_ESP32)
    uart_flush_input(LIN_UART_NUM);
#elif defined(PLATFORM_LINUX)
    if (g_linFd >= 0) {
        tcflush(g_linFd, TCIFLUSH);
    }
#endif
}

/******************************************************************************
 * @brief     : Flush LIN TX buffer
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinFlushTxBuffer(void)
{
#ifdef PLATFORM_STM32F4
    while (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_TC) == RESET) {
    }
#elif defined(PLATFORM_STM32F1)
    while (__HAL_UART_GET_FLAG(&g_linUartHandle, UART_FLAG_TC) == RESET) {
    }
#elif defined(PLATFORM_ESP32)
    uart_wait_tx_done(LIN_UART_NUM, 1000 / portTICK_PERIOD_MS);
#elif defined(PLATFORM_LINUX)
    if (g_linFd >= 0) {
        tcflush(g_linFd, TCOFLUSH);
    }
#endif
}

/******************************************************************************
 * @brief     : Set LIN break length (for platforms supporting variable break)
 * @param[in] : breakLength - Break length in bit times
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation, LIN uses 13-bit break
 *****************************************************************************/
int32_t LinSetBreakLength(uint8_t breakLength)
{
    /* Most platforms use fixed break length, this is for future expansion */
    (void)breakLength; /* Unused parameter */
    return 0;
}

/******************************************************************************
 * @brief     : Enable LIN RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinEnableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_linUartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_linUartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    uart_enable_rx_intr(LIN_UART_NUM);
#elif defined(PLATFORM_LINUX)
    /* Linux uses polling or select/poll mechanisms */
#endif
}

/******************************************************************************
 * @brief     : Enable LIN TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinEnableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_ENABLE_IT(&g_linUartHandle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_ENABLE_IT(&g_linUartHandle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    uart_enable_tx_intr(LIN_UART_NUM, 1, 0);
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't use TX interrupts in userspace */
#endif
}

/******************************************************************************
 * @brief     : Disable LIN RX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinDisableRxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_linUartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_linUartHandle, UART_IT_RXNE);
#elif defined(PLATFORM_ESP32)
    uart_disable_rx_intr(LIN_UART_NUM);
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Disable LIN TX interrupt
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinDisableTxInterrupt(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_UART_DISABLE_IT(&g_linUartHandle, UART_IT_TXE);
#elif defined(PLATFORM_STM32F1)
    __HAL_UART_DISABLE_IT(&g_linUartHandle, UART_IT_TXE);
#elif defined(PLATFORM_ESP32)
    uart_disable_tx_intr(LIN_UART_NUM);
#elif defined(PLATFORM_LINUX)
    /* Not applicable */
#endif
}

/******************************************************************************
 * @brief     : Configure NVIC for LIN UART
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void LinConfigureNvic(void)
{
#ifdef PLATFORM_STM32F4
    HAL_NVIC_SetPriority(LIN_UART_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(LIN_UART_IRQn);
#elif defined(PLATFORM_STM32F1)
    HAL_NVIC_SetPriority(LIN_UART_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(LIN_UART_IRQn);
#elif defined(PLATFORM_ESP32)
    /* ESP32 interrupt handling is done by driver */
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't have NVIC */
#endif
}

/******************************************************************************
 * @brief     : Delay in microseconds
 * @param[in] : us - Microseconds to delay
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation for precise timing
 *****************************************************************************/
void LinDelayUs(uint32_t us)
{
#ifdef PLATFORM_STM32F4
    /* Use DWT cycle counter for precise microsecond delay */
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000) * us;
    while ((DWT->CYCCNT - start) < cycles) {
    }
#elif defined(PLATFORM_STM32F1)
    /* Simple delay loop (not precise) */
    volatile uint32_t count = us * (SystemCoreClock / 1000000) / 5;
    while (count--) {
    }
#elif defined(PLATFORM_ESP32)
    esp_rom_delay_us(us);
#elif defined(PLATFORM_LINUX)
    usleep(us);
#endif
}

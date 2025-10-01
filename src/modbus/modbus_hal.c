#include "securec.h"
#include "modbus_hal.h"
#include "platform_config.h"

/* NOTE: Modbus is implemented on top of UART/RS485
 * Modbus RTU uses CRC16 for error checking
 * All platforms share the same CRC calculation algorithm
 */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static UART_HandleTypeDef g_modbusHandle;
#endif

#ifdef PLATFORM_STM32F1
static UART_HandleTypeDef g_modbusHandle;
#endif

#ifdef PLATFORM_ESP32
static const int g_modbusUartNum = UART_NUM_2;
#endif

#ifdef PLATFORM_LINUX
static int g_modbusFd = -1;
#endif

/******************************************************************************
 * @brief     : Initialize Modbus UART
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ModbusUartInit(void)
{
#ifdef PLATFORM_STM32F4
    /* Enable clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO */
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure UART */
    g_modbusHandle.Instance = USART2;
    g_modbusHandle.Init.BaudRate = 9600;  /* Default Modbus baud rate */
    g_modbusHandle.Init.WordLength = UART_WORDLENGTH_8B;
    g_modbusHandle.Init.StopBits = UART_STOPBITS_1;
    g_modbusHandle.Init.Parity = UART_PARITY_NONE;
    g_modbusHandle.Init.Mode = UART_MODE_TX_RX;
    g_modbusHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    g_modbusHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    return (HAL_UART_Init(&g_modbusHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    /* Enable clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO */
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = GPIO_PIN_2;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    gpioInit.Pin = GPIO_PIN_3;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure UART */
    g_modbusHandle.Instance = USART2;
    g_modbusHandle.Init.BaudRate = 9600;
    g_modbusHandle.Init.WordLength = UART_WORDLENGTH_8B;
    g_modbusHandle.Init.StopBits = UART_STOPBITS_1;
    g_modbusHandle.Init.Parity = UART_PARITY_NONE;
    g_modbusHandle.Init.Mode = UART_MODE_TX_RX;
    g_modbusHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;

    return (HAL_UART_Init(&g_modbusHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    uart_config_t uartConfig = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(g_modbusUartNum, &uartConfig);
    uart_set_pin(g_modbusUartNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(g_modbusUartNum, 256, 0, 0, NULL, 0);
    return 0;
#elif defined(PLATFORM_LINUX)
    g_modbusFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_modbusFd < 0) {
        return -1;
    }

    fcntl(g_modbusFd, F_SETFL, 0);

    struct termios options;
    tcgetattr(g_modbusFd, &options);

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    tcsetattr(g_modbusFd, TCSANOW, &options);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Send data through Modbus UART
 * @param[in] : data - Pointer to data buffer to send
 * @param[in] : length - Number of bytes to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ModbusUartSendData(uint8_t* data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    return (HAL_UART_Transmit(&g_modbusHandle, data, length, 1000) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    return (HAL_UART_Transmit(&g_modbusHandle, data, length, 1000) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    int sent = uart_write_bytes(g_modbusUartNum, (const char*)data, length);
    return (sent == length) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_modbusFd < 0) {
        return -1;
    }
    ssize_t sent = write(g_modbusFd, data, length);
    return (sent == length) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Receive data from Modbus UART
 * @param[in] : buffer - Pointer to buffer to store received data
 * @param[in] : maxLength - Maximum number of bytes to receive
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ModbusUartReceiveData(uint8_t* buffer, uint16_t maxLength)
{
    if (buffer == NULL || maxLength == 0) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    uint16_t received = 0;
    while (received < maxLength && (__HAL_UART_GET_FLAG(&g_modbusHandle, UART_FLAG_RXNE) != RESET)) {
        buffer[received++] = (uint8_t)(g_modbusHandle.Instance->DR & 0xFF);
    }
    return received;
#elif defined(PLATFORM_STM32F1)
    uint16_t received = 0;
    while (received < maxLength && (__HAL_UART_GET_FLAG(&g_modbusHandle, UART_FLAG_RXNE) != RESET)) {
        buffer[received++] = (uint8_t)(g_modbusHandle.Instance->DR & 0xFF);
    }
    return received;
#elif defined(PLATFORM_ESP32)
    size_t available = 0;
    uart_get_buffered_data_len(g_modbusUartNum, &available);
    if (available > maxLength) {
        available = maxLength;
    }
    if (available > 0) {
        int read = uart_read_bytes(g_modbusUartNum, buffer, available, 0);
        return read;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_modbusFd < 0) {
        return -1;
    }

    int bytes = 0;
    ioctl(g_modbusFd, FIONREAD, &bytes);
    if (bytes > maxLength) {
        bytes = maxLength;
    }
    if (bytes > 0) {
        ssize_t read_bytes = read(g_modbusFd, buffer, bytes);
        return (int32_t)read_bytes;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Calculate Modbus CRC16
 * @param[in] : data - Pointer to data buffer
 * @param[in] : length - Number of bytes in data buffer
 * @param[out]: None
 * @return    : Calculated CRC16 value
 * @note      : Platform-independent implementation (Modbus RTU CRC16)
 *****************************************************************************/
int32_t ModbusCalculateCRC(uint8_t* data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return -1;
    }

    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];

        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }

    return (int32_t)crc;
}
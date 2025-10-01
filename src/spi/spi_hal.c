#include "platform_config.h"
#include "securec.h"
#include "spi_hal.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static SPI_HandleTypeDef g_spiHandle;
#endif

#ifdef PLATFORM_STM32F1
static SPI_HandleTypeDef g_spiHandle;
#endif

#ifdef PLATFORM_ESP32
static spi_device_handle_t g_spiDevice;
static spi_bus_config_t g_busConfig = {.mosi_io_num = GPIO_NUM_23,
                                       .miso_io_num = GPIO_NUM_19,
                                       .sclk_io_num = GPIO_NUM_18,
                                       .quadwp_io_num = -1,
                                       .quadhd_io_num = -1,
                                       .max_transfer_sz = 4096};
static spi_device_interface_config_t g_devConfig = {.clock_speed_hz = 1000000, .mode = 0, .spics_io_num = GPIO_NUM_5, .queue_size = 7};
#endif

#ifdef PLATFORM_LINUX
static int g_spiFd = -1;
static uint8_t g_spiMode = 0;
static uint32_t g_spiSpeed = 1000000;
#endif

/******************************************************************************
 * @brief     : Enable SPI clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SpiEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 SPI clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure SPI GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SpiConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* SPI1 GPIO Configuration: PA5(SCK), PA6(MISO), PA7(MOSI) */
    gpioInit.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* SPI1 SCK (PA5) and MOSI (PA7) */
    gpioInit.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* SPI1 MISO (PA6) */
    gpioInit.Pin = GPIO_PIN_6;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by spi_bus_initialize */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device files, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable SPI module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SpiEnable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_SPI_ENABLE(&g_spiHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_SPI_ENABLE(&g_spiHandle);
#elif defined(PLATFORM_ESP32)
    spi_bus_initialize(SPI_HOST_DEFAULT, &g_busConfig, 1);
    spi_bus_add_device(SPI_HOST_DEFAULT, &g_devConfig, &g_spiDevice);
#elif defined(PLATFORM_LINUX)
    if (g_spiFd < 0) {
        g_spiFd = open("/dev/spidev0.0", O_RDWR);
        if (g_spiFd >= 0) {
            if (ioctl(g_spiFd, SPI_IOC_WR_MODE, &g_spiMode) < 0) {
                close(g_spiFd);
                g_spiFd = -1;
                return;
            }
            if (ioctl(g_spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &g_spiSpeed) < 0) {
                close(g_spiFd);
                g_spiFd = -1;
                return;
            }
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Check if SPI is ready
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SpiIsReady(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_TXE) != RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1; /* ESP32 driver handles readiness */
#elif defined(PLATFORM_LINUX)
    return (g_spiFd >= 0) ? 1 : 0;
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Check if SPI transfer is complete
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if complete, 0 if not complete
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SpiTransferComplete(void)
{
#ifdef PLATFORM_STM32F4
    return (__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_BSY) == RESET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_BSY) == RESET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return 1; /* ESP32 driver handles transfer completion */
#elif defined(PLATFORM_LINUX)
    return 1; /* Linux transfers are blocking */
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Write data to SPI
 * @param[in] : data - Byte to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SpiWriteData(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    while (!(__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_TXE)))
        ;
    *(__IO uint8_t*)&g_spiHandle.Instance->DR = data;
#elif defined(PLATFORM_STM32F1)
    while (!(__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_TXE)))
        ;
    *(__IO uint8_t*)&g_spiHandle.Instance->DR = data;
#elif defined(PLATFORM_ESP32)
    spi_transaction_t trans = {.length = 8, .tx_buffer = &data, .rx_buffer = NULL};
    spi_device_transmit(g_spiDevice, &trans);
#elif defined(PLATFORM_LINUX)
    if (g_spiFd >= 0) {
        (void)write(g_spiFd, &data, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Read data from SPI
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from SPI
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t SpiReadData(void)
{
#ifdef PLATFORM_STM32F4
    while (!(__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_RXNE)))
        ;
    return (uint8_t)g_spiHandle.Instance->DR;
#elif defined(PLATFORM_STM32F1)
    while (!(__HAL_SPI_GET_FLAG(&g_spiHandle, SPI_FLAG_RXNE)))
        ;
    return (uint8_t)g_spiHandle.Instance->DR;
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    uint8_t dummy = 0xFF;
    spi_transaction_t trans = {.length = 8, .tx_buffer = &dummy, .rx_buffer = &data};
    spi_device_transmit(g_spiDevice, &trans);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_spiFd >= 0) {
        (void)read(g_spiFd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Configure SPI clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SpiConfigureClockSpeed(uint32_t clockSpeed)
{
#ifdef PLATFORM_STM32F4
    g_spiHandle.Instance = SPI1;
    g_spiHandle.Init.Mode = SPI_MODE_MASTER;
    g_spiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    g_spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    g_spiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_spiHandle.Init.NSS = SPI_NSS_SOFT;
    g_spiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    /* Configure prescaler based on clock speed (APB2 = 84MHz) */
    if (clockSpeed >= 42000000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    } else if (clockSpeed >= 21000000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    } else if (clockSpeed >= 10500000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    } else if (clockSpeed >= 5250000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    } else if (clockSpeed >= 2625000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    } else if (clockSpeed >= 1312500) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    } else {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    }

    return (HAL_SPI_Init(&g_spiHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_spiHandle.Instance = SPI1;
    g_spiHandle.Init.Mode = SPI_MODE_MASTER;
    g_spiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    g_spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    g_spiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_spiHandle.Init.NSS = SPI_NSS_SOFT;
    g_spiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    /* Configure prescaler based on clock speed (APB2 = 72MHz) */
    if (clockSpeed >= 36000000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    } else if (clockSpeed >= 18000000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    } else if (clockSpeed >= 9000000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    } else if (clockSpeed >= 4500000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    } else if (clockSpeed >= 2250000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    } else if (clockSpeed >= 1125000) {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    } else {
        g_spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    }

    return (HAL_SPI_Init(&g_spiHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    g_devConfig.clock_speed_hz = (int)clockSpeed;
    return 0;
#elif defined(PLATFORM_LINUX)
    g_spiSpeed = clockSpeed;
    if (g_spiFd >= 0) {
        ioctl(g_spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &g_spiSpeed);
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure SPI mode
 * @param[in] : mode - SPI mode (MASTER or SLAVE)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SpiConfigureMode(SpiMode_E mode)
{
#ifdef PLATFORM_STM32F4
    if (mode == SPI_MODE_MASTER) {
        g_spiHandle.Init.Mode = SPI_MODE_MASTER;
    } else {
        g_spiHandle.Init.Mode = SPI_MODE_SLAVE;
    }
    return (HAL_SPI_Init(&g_spiHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    if (mode == SPI_MODE_MASTER) {
        g_spiHandle.Init.Mode = SPI_MODE_MASTER;
    } else {
        g_spiHandle.Init.Mode = SPI_MODE_SLAVE;
    }
    return (HAL_SPI_Init(&g_spiHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 mode is configured during initialization */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux SPI is typically master mode */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Set SPI data format
 * @param[in] : format - Data format (8-bit or 16-bit)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SpiSetDataFormat(SpiDataFormat_E format)
{
#ifdef PLATFORM_STM32F4
    if (format == SPI_DATA_FORMAT_8BIT) {
        g_spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    } else {
        g_spiHandle.Init.DataSize = SPI_DATASIZE_16BIT;
    }
    HAL_SPI_Init(&g_spiHandle);
#elif defined(PLATFORM_STM32F1)
    if (format == SPI_DATA_FORMAT_8BIT) {
        g_spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    } else {
        g_spiHandle.Init.DataSize = SPI_DATASIZE_16BIT;
    }
    HAL_SPI_Init(&g_spiHandle);
#elif defined(PLATFORM_ESP32)
    /* ESP32 data format is configured in transaction */
#elif defined(PLATFORM_LINUX)
    uint8_t bits = (format == SPI_DATA_FORMAT_8BIT) ? 8 : 16;
    if (g_spiFd >= 0) {
        ioctl(g_spiFd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    }
#endif
}

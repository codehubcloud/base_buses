#include "securec.h"
#include "i2s_hal.h"
#include "platform_config.h"

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static I2S_HandleTypeDef g_i2sHandle;
#endif

#ifdef PLATFORM_STM32F1
static I2S_HandleTypeDef g_i2sHandle;
#endif

#ifdef PLATFORM_ESP32
#include "driver/i2s.h"
static const i2s_port_t g_i2sNum = I2S_NUM_0;
#endif

#ifdef PLATFORM_LINUX
#include <stdio.h>
static uint32_t g_currentSampleRate = 44100;
static uint8_t g_currentDataFormat = 0;
#endif

/******************************************************************************
 * @brief     : Enable I2S clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 I2S clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] Clock enabled (simulated)\n");
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Disable I2S clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void I2sDisableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_SPI2_CLK_DISABLE();
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_SPI2_CLK_DISABLE();
#elif defined(PLATFORM_ESP32)
    i2s_driver_uninstall(g_i2sNum);
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] Clock disabled (simulated)\n");
#endif
}

/******************************************************************************
 * @brief     : Configure I2S GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* I2S2 GPIO Configuration:
     * PB12: I2S2_WS (Word Select / LRCLK)
     * PB13: I2S2_CK (Bit Clock / BCLK)
     * PB15: I2S2_SD (Serial Data)
     * PC6:  I2S2_MCK (Master Clock - optional)
     */
    gpioInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioInit.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    /* Master clock pin */
    gpioInit.Pin = GPIO_PIN_6;
    gpioInit.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* I2S2 GPIO Configuration:
     * PB12: I2S2_WS
     * PB13: I2S2_CK
     * PB15: I2S2_SD
     */
    gpioInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by i2s_set_pin */
    return 0;
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] GPIO configured (simulated)\n");
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable I2S module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void I2sEnable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_I2S_ENABLE(&g_i2sHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_I2S_ENABLE(&g_i2sHandle);
#elif defined(PLATFORM_ESP32)
    i2s_start(g_i2sNum);
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] I2S enabled (simulated)\n");
#endif
}

/******************************************************************************
 * @brief     : Disable I2S module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void I2sDisable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_I2S_DISABLE(&g_i2sHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_I2S_DISABLE(&g_i2sHandle);
#elif defined(PLATFORM_ESP32)
    i2s_stop(g_i2sNum);
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] I2S disabled (simulated)\n");
#endif
}

/******************************************************************************
 * @brief     : Set I2S data format
 * @param[in] : dataFormat - Audio data format (16/24/32-bit)
 *              channelMode - Channel mode (stereo/mono)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sSetDataFormat(uint8_t dataFormat, uint8_t channelMode)
{
#ifdef PLATFORM_STM32F4
    uint32_t dataLength = I2S_DATAFORMAT_16B;

    if (dataFormat == 1) {
        dataLength = I2S_DATAFORMAT_24B;
    } else if (dataFormat == 2) {
        dataLength = I2S_DATAFORMAT_32B;
    }

    g_i2sHandle.Init.DataFormat = dataLength;
    return (HAL_I2S_Init(&g_i2sHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    uint32_t dataLength = I2S_DATAFORMAT_16B;

    if (dataFormat == 1) {
        dataLength = I2S_DATAFORMAT_24B;
    } else if (dataFormat == 2) {
        dataLength = I2S_DATAFORMAT_32B;
    }

    g_i2sHandle.Init.DataFormat = dataLength;
    return (HAL_I2S_Init(&g_i2sHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    i2s_bits_per_sample_t bits = I2S_BITS_PER_SAMPLE_16BIT;

    if (dataFormat == 1) {
        bits = I2S_BITS_PER_SAMPLE_24BIT;
    } else if (dataFormat == 2) {
        bits = I2S_BITS_PER_SAMPLE_32BIT;
    }

    i2s_channel_fmt_t channel = I2S_CHANNEL_FMT_RIGHT_LEFT;
    if (channelMode == 1) {
        channel = I2S_CHANNEL_FMT_ONLY_LEFT;
    } else if (channelMode == 2) {
        channel = I2S_CHANNEL_FMT_ONLY_RIGHT;
    }

    i2s_set_clk(g_i2sNum, g_currentSampleRate, bits, channel);
    return 0;
#elif defined(PLATFORM_LINUX)
    const char* formatName[] = {"16-bit", "24-bit", "32-bit"};
    const char* channelName[] = {"Stereo", "Mono-L", "Mono-R"};
    printf("[I2S HAL] Format set: %s, %s (simulated)\n",
           formatName[dataFormat], channelName[channelMode]);
    g_currentDataFormat = dataFormat;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure I2S sample rate
 * @param[in] : sampleRate - Desired sample rate in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sConfigureSampleRate(uint32_t sampleRate)
{
#ifdef PLATFORM_STM32F4
    g_i2sHandle.Instance = SPI2;
    g_i2sHandle.Init.Mode = I2S_MODE_MASTER_TX;
    g_i2sHandle.Init.Standard = I2S_STANDARD_PHILIPS;
    g_i2sHandle.Init.DataFormat = I2S_DATAFORMAT_16B;
    g_i2sHandle.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    g_i2sHandle.Init.AudioFreq = sampleRate;
    g_i2sHandle.Init.CPOL = I2S_CPOL_LOW;
    g_i2sHandle.Init.ClockSource = I2S_CLOCK_PLL;
    return (HAL_I2S_Init(&g_i2sHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_i2sHandle.Instance = SPI2;
    g_i2sHandle.Init.Mode = I2S_MODE_MASTER_TX;
    g_i2sHandle.Init.Standard = I2S_STANDARD_PHILIPS;
    g_i2sHandle.Init.DataFormat = I2S_DATAFORMAT_16B;
    g_i2sHandle.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    g_i2sHandle.Init.AudioFreq = sampleRate;
    g_i2sHandle.Init.CPOL = I2S_CPOL_LOW;
    return (HAL_I2S_Init(&g_i2sHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    i2s_config_t i2sConfig = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_driver_install(g_i2sNum, &i2sConfig, 0, NULL);

    i2s_pin_config_t pinConfig = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 22,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_set_pin(g_i2sNum, &pinConfig);

    g_currentSampleRate = sampleRate;
    return 0;
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] Sample rate set to %u Hz (simulated)\n", sampleRate);
    g_currentSampleRate = sampleRate;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Write data to I2S interface
 * @param[in] : data - Pointer to data buffer
 *              length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sWrite(uint8_t* data, uint32_t length)
{
#ifdef PLATFORM_STM32F4
    HAL_StatusTypeDef status;
    status = HAL_I2S_Transmit(&g_i2sHandle, (uint16_t*)data, length / 2, 1000);
    return (status == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    HAL_StatusTypeDef status;
    status = HAL_I2S_Transmit(&g_i2sHandle, (uint16_t*)data, length / 2, 1000);
    return (status == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    size_t bytesWritten = 0;
    esp_err_t err = i2s_write(g_i2sNum, data, length, &bytesWritten, 1000);
    return (err == ESP_OK && bytesWritten == length) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] Transmitted %u bytes of audio data (simulated)\n", length);
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read data from I2S interface
 * @param[in] : maxLength - Maximum number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : Number of bytes actually read, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sRead(uint8_t* buffer, uint32_t maxLength)
{
#ifdef PLATFORM_STM32F4
    HAL_StatusTypeDef status;
    status = HAL_I2S_Receive(&g_i2sHandle, (uint16_t*)buffer, maxLength / 2, 1000);
    return (status == HAL_OK) ? (int32_t)maxLength : -1;
#elif defined(PLATFORM_STM32F1)
    HAL_StatusTypeDef status;
    status = HAL_I2S_Receive(&g_i2sHandle, (uint16_t*)buffer, maxLength / 2, 1000);
    return (status == HAL_OK) ? (int32_t)maxLength : -1;
#elif defined(PLATFORM_ESP32)
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(g_i2sNum, buffer, maxLength, &bytesRead, 1000);
    return (err == ESP_OK) ? (int32_t)bytesRead : -1;
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] Received %u bytes of audio data (simulated)\n", maxLength);
    if (memset_s(buffer, maxLength, 0, maxLength) != EOK) {
        return -1;
    }
    return (int32_t)maxLength;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure DMA for I2S
 * @param[in] : enable - 1 to enable DMA, 0 to disable
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2sConfigureDMA(uint8_t enable)
{
#ifdef PLATFORM_STM32F4
    if (enable) {
        __HAL_RCC_DMA1_CLK_ENABLE();
        /* DMA configuration would go here */
        printf("[I2S HAL] DMA enabled for I2S\n");
    } else {
        __HAL_RCC_DMA1_CLK_DISABLE();
        printf("[I2S HAL] DMA disabled for I2S\n");
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    if (enable) {
        __HAL_RCC_DMA1_CLK_ENABLE();
        printf("[I2S HAL] DMA enabled for I2S\n");
    } else {
        __HAL_RCC_DMA1_CLK_DISABLE();
        printf("[I2S HAL] DMA disabled for I2S\n");
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 I2S driver uses DMA by default */
    printf("[I2S HAL] ESP32 I2S uses built-in DMA\n");
    return 0;
#elif defined(PLATFORM_LINUX)
    printf("[I2S HAL] DMA %s (simulated)\n", enable ? "enabled" : "disabled");
    return 0;
#else
    return -1;
#endif
}

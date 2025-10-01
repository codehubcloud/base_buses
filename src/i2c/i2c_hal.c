#include "i2c_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static I2C_HandleTypeDef g_i2cHandle;
#endif

#ifdef PLATFORM_STM32F1
static I2C_HandleTypeDef g_i2cHandle;
#endif

#ifdef PLATFORM_ESP32
static i2c_config_t g_i2cConfig = {.mode = I2C_MODE_MASTER,
                                   .sda_io_num = GPIO_NUM_21,
                                   .scl_io_num = GPIO_NUM_22,
                                   .sda_pullup_en = GPIO_PULLUP_ENABLE,
                                   .scl_pullup_en = GPIO_PULLUP_ENABLE,
                                   .master.clk_speed = 100000};
#endif

#ifdef PLATFORM_LINUX
static int g_i2cFd = -1;
static uint8_t g_i2cAddr = 0;
#endif

/******************************************************************************
 * @brief     : Enable I2C clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 I2C clock is enabled by driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure I2C GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* I2C1 GPIO Configuration: PB8(SCL), PB9(SDA) */
    gpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpioInit.Mode = GPIO_MODE_AF_OD;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* I2C1 GPIO Configuration: PB6(SCL), PB7(SDA) */
    gpioInit.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpioInit.Mode = GPIO_MODE_AF_OD;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configured by i2c_driver_install */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux uses device files, no GPIO configuration needed */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable I2C module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void I2cEnable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_I2C_ENABLE(&g_i2cHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_I2C_ENABLE(&g_i2cHandle);
#elif defined(PLATFORM_ESP32)
    i2c_param_config(I2C_NUM_DEFAULT, &g_i2cConfig);
    i2c_driver_install(I2C_NUM_DEFAULT, g_i2cConfig.mode, 0, 0, 0);
#elif defined(PLATFORM_LINUX)
    if (g_i2cFd < 0) {
        g_i2cFd = open("/dev/i2c-1", O_RDWR);
        if (g_i2cFd < 0) {
            return;
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Generate I2C START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cGenerateStart(void)
{
#ifdef PLATFORM_STM32F4
    /* For STM32 HAL, START is generated automatically during transfer */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* For STM32 HAL, START is generated automatically during transfer */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 handles START condition in i2c_master_cmd_begin */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux handles START condition automatically */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Generate I2C STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cGenerateStop(void)
{
#ifdef PLATFORM_STM32F4
    /* For STM32 HAL, STOP is generated automatically during transfer */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* For STM32 HAL, STOP is generated automatically during transfer */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 handles STOP condition in i2c_master_cmd_begin */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux handles STOP condition automatically */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Send byte through I2C
 * @param[in] : data - Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cSendByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    return (HAL_I2C_Master_Transmit(&g_i2cHandle, g_i2cAddr, &data, 1, 1000) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    return (HAL_I2C_Master_Transmit(&g_i2cHandle, g_i2cAddr, &data, 1, 1000) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_i2cAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_i2cFd < 0) {
        return -1;
    }
    return (write(g_i2cFd, &data, 1) == 1) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read byte from I2C
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from I2C
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t I2cReadByte(void)
{
#ifdef PLATFORM_STM32F4
    uint8_t data = 0;
    HAL_I2C_Master_Receive(&g_i2cHandle, g_i2cAddr, &data, 1, 1000);
    return data;
#elif defined(PLATFORM_STM32F1)
    uint8_t data = 0;
    HAL_I2C_Master_Receive(&g_i2cHandle, g_i2cAddr, &data, 1, 1000);
    return data;
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_i2cAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_i2cFd >= 0) {
        (void)read(g_i2cFd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Check I2C ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cCheckAck(void)
{
#ifdef PLATFORM_STM32F4
    /* STM32 HAL handles ACK checking internally */
    return 1;
#elif defined(PLATFORM_STM32F1)
    /* STM32 HAL handles ACK checking internally */
    return 1;
#elif defined(PLATFORM_ESP32)
    /* ESP32 driver handles ACK checking */
    return 1;
#elif defined(PLATFORM_LINUX)
    /* Linux driver handles ACK checking */
    return 1;
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Send I2C ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void I2cSendAck(void)
{
#ifdef PLATFORM_STM32F4
    /* STM32 HAL handles ACK generation automatically */
#elif defined(PLATFORM_STM32F1)
    /* STM32 HAL handles ACK generation automatically */
#elif defined(PLATFORM_ESP32)
    /* ESP32 driver handles ACK generation */
#elif defined(PLATFORM_LINUX)
    /* Linux driver handles ACK generation */
#endif
}

/******************************************************************************
 * @brief     : Send I2C NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void I2cSendNack(void)
{
#ifdef PLATFORM_STM32F4
    /* STM32 HAL handles NACK generation automatically */
#elif defined(PLATFORM_STM32F1)
    /* STM32 HAL handles NACK generation automatically */
#elif defined(PLATFORM_ESP32)
    /* ESP32 driver handles NACK generation */
#elif defined(PLATFORM_LINUX)
    /* Linux driver handles NACK generation */
#endif
}

/******************************************************************************
 * @brief     : Configure I2C clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t I2cConfigureClockSpeed(uint32_t clockSpeed)
{
#ifdef PLATFORM_STM32F4
    g_i2cHandle.Instance = I2C1;
    g_i2cHandle.Init.ClockSpeed = clockSpeed;
    g_i2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_i2cHandle.Init.OwnAddress1 = 0;
    g_i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_i2cHandle.Init.OwnAddress2 = 0;
    g_i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_i2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    return (HAL_I2C_Init(&g_i2cHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_i2cHandle.Instance = I2C1;
    g_i2cHandle.Init.ClockSpeed = clockSpeed;
    g_i2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_i2cHandle.Init.OwnAddress1 = 0;
    g_i2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_i2cHandle.Init.OwnAddress2 = 0;
    g_i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_i2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    return (HAL_I2C_Init(&g_i2cHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    g_i2cConfig.master.clk_speed = clockSpeed;
    i2c_param_config(I2C_NUM_DEFAULT, &g_i2cConfig);
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux I2C clock speed is typically configured at kernel level */
    return 0;
#else
    return -1;
#endif
}

#include "platform_config.h"
#include "securec.h"
#include "smbus_hal.h"


/* NOTE: SMBus is a subset of I2C with additional features
 * SMBus uses PEC (Packet Error Code) for error checking
 * This implementation uses I2C driver with SMBus-specific extensions
 * SMBus typically operates at 100kHz (low speed) or 400kHz (high speed)
 */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static I2C_HandleTypeDef g_smbusHandle;
static uint8_t g_smbusDeviceAddr = 0;
#endif

#ifdef PLATFORM_STM32F1
static I2C_HandleTypeDef g_smbusHandle;
static uint8_t g_smbusDeviceAddr = 0;
#endif

#ifdef PLATFORM_ESP32
static i2c_config_t g_smbusConfig = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000 /* SMBus default 100kHz */
};
static uint8_t g_smbusDeviceAddr = 0;
#endif

#ifdef PLATFORM_LINUX
static int g_smbusFd = -1;
static uint8_t g_smbusDeviceAddr = 0;
#endif

/******************************************************************************
 * @brief     : Calculate SMBus PEC (CRC-8)
 * @param[in] : data - Pointer to data buffer
 * @param[in] : length - Number of bytes in data buffer
 * @param[out]: None
 * @return    : Calculated PEC value
 * @note      : Platform-independent implementation (SMBus PEC uses CRC-8)
 *****************************************************************************/
static uint8_t SmBusCalculatePEC(uint8_t* data, uint16_t length)
{
    uint8_t crc = 0;
    uint16_t i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; /* SMBus polynomial */
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/******************************************************************************
 * @brief     : Enable SMBus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_I2C2_CLK_ENABLE();
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
 * @brief     : Configure SMBus GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* I2C2 GPIO Configuration for SMBus: PB10(SCL), PB11(SDA) */
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_AF_OD;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* I2C2 GPIO Configuration for SMBus: PB10(SCL), PB11(SDA) */
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
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
 * @brief     : Enable SMBus module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SmBusEnable(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_I2C_ENABLE(&g_smbusHandle);
#elif defined(PLATFORM_STM32F1)
    __HAL_I2C_ENABLE(&g_smbusHandle);
#elif defined(PLATFORM_ESP32)
    i2c_param_config(I2C_NUM_DEFAULT, &g_smbusConfig);
    i2c_driver_install(I2C_NUM_DEFAULT, g_smbusConfig.mode, 0, 0, 0);
#elif defined(PLATFORM_LINUX)
    if (g_smbusFd < 0) {
        /* Try SMBus device first, fallback to I2C if not available */
        g_smbusFd = open("/dev/i2c-1", O_RDWR);
        if (g_smbusFd < 0) {
            return;
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Generate SMBus START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusGenerateStart(void)
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
 * @brief     : Generate SMBus STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusGenerateStop(void)
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
 * @brief     : Send byte through SMBus
 * @param[in] : data - Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusSendByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    return (HAL_I2C_Master_Transmit(&g_smbusHandle, g_smbusDeviceAddr, &data, 1, 1000) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    return (HAL_I2C_Master_Transmit(&g_smbusHandle, g_smbusDeviceAddr, &data, 1, 1000) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_smbusDeviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
#elif defined(PLATFORM_LINUX)
    if (g_smbusFd < 0) {
        return -1;
    }
    return (write(g_smbusFd, &data, 1) == 1) ? 0 : -1;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read byte from SMBus
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from SMBus
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t SmBusReadByteHal(void)
{
#ifdef PLATFORM_STM32F4
    uint8_t data = 0;
    HAL_I2C_Master_Receive(&g_smbusHandle, g_smbusDeviceAddr, &data, 1, 1000);
    return data;
#elif defined(PLATFORM_STM32F1)
    uint8_t data = 0;
    HAL_I2C_Master_Receive(&g_smbusHandle, g_smbusDeviceAddr, &data, 1, 1000);
    return data;
#elif defined(PLATFORM_ESP32)
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (g_smbusDeviceAddr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return data;
#elif defined(PLATFORM_LINUX)
    uint8_t data = 0;
    if (g_smbusFd >= 0) {
        (void)read(g_smbusFd, &data, 1);
    }
    return data;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Check SMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusCheckAck(void)
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
 * @brief     : Send SMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SmBusSendAck(void)
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
 * @brief     : Send SMBus NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SmBusSendNack(void)
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
 * @brief     : Configure SMBus clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SmBusConfigureClockSpeed(uint32_t clockSpeed)
{
#ifdef PLATFORM_STM32F4
    g_smbusHandle.Instance = I2C2;
    g_smbusHandle.Init.ClockSpeed = clockSpeed;
    g_smbusHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_smbusHandle.Init.OwnAddress1 = 0;
    g_smbusHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_smbusHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_smbusHandle.Init.OwnAddress2 = 0;
    g_smbusHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_smbusHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    return (HAL_I2C_Init(&g_smbusHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_STM32F1)
    g_smbusHandle.Instance = I2C2;
    g_smbusHandle.Init.ClockSpeed = clockSpeed;
    g_smbusHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_smbusHandle.Init.OwnAddress1 = 0;
    g_smbusHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_smbusHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_smbusHandle.Init.OwnAddress2 = 0;
    g_smbusHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_smbusHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    return (HAL_I2C_Init(&g_smbusHandle) == HAL_OK) ? 0 : -1;
#elif defined(PLATFORM_ESP32)
    g_smbusConfig.master.clk_speed = clockSpeed;
    i2c_param_config(I2C_NUM_DEFAULT, &g_smbusConfig);
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux I2C/SMBus clock speed is typically configured at kernel level */
    return 0;
#else
    return -1;
#endif
}
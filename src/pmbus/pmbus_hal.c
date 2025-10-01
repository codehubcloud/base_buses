#include "platform_config.h"
#include "pmbus_hal.h"
#include "securec.h"


/* NOTE: PMBus is built on SMBus/I2C for power management applications
 * PMBus uses LINEAR11 and LINEAR16 data formats
 * PMBus supports page addressing for multi-rail power supplies
 * This HAL provides the low-level I2C/SMBus transport layer
 */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static I2C_HandleTypeDef g_pmbusHandle;
static uint8_t g_pmbusDeviceAddr = 0;
#endif

#ifdef PLATFORM_STM32F1
static I2C_HandleTypeDef g_pmbusHandle;
static uint8_t g_pmbusDeviceAddr = 0;
#endif

#ifdef PLATFORM_ESP32
static i2c_config_t g_pmbusConfig = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000 /* PMBus default 100kHz */
};
static uint8_t g_pmbusDeviceAddr = 0;
#endif

#ifdef PLATFORM_LINUX
static int g_pmbusFd = -1;
static uint8_t g_pmbusDeviceAddr = 0;
#endif

/******************************************************************************
 * @brief     : Enable PMBus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusEnableClock(void)
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
 * @brief     : Configure PMBus GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit;

    /* Configure I2C2 pins: PB10 (SCL), PB11 (SDA) */
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_AF_OD;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit;

    /* Configure I2C2 pins: PB10 (SCL), PB11 (SDA) */
    gpioInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpioInit.Mode = GPIO_MODE_AF_OD;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO configuration handled by i2c_param_config */
    esp_err_t result = i2c_param_config(I2C_NUM_DEFAULT, &g_pmbusConfig);
    if (result != ESP_OK) {
        return -1;
    }

    result = i2c_driver_install(I2C_NUM_DEFAULT, g_pmbusConfig.mode, 0, 0, 0);
    if (result != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux GPIO configuration not required for I2C */
    const char* i2cDevice = "/dev/i2c-1";
    g_pmbusFd = open(i2cDevice, O_RDWR);
    if (g_pmbusFd < 0) {
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable PMBus module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void PmBusEnable(void)
{
#ifdef PLATFORM_STM32F4
    g_pmbusHandle.Instance = I2C2;
    g_pmbusHandle.Init.ClockSpeed = 100000;
    g_pmbusHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_pmbusHandle.Init.OwnAddress1 = 0;
    g_pmbusHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_pmbusHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_pmbusHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_pmbusHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&g_pmbusHandle);
#elif defined(PLATFORM_STM32F1)
    g_pmbusHandle.Instance = I2C2;
    g_pmbusHandle.Init.ClockSpeed = 100000;
    g_pmbusHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_pmbusHandle.Init.OwnAddress1 = 0;
    g_pmbusHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_pmbusHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_pmbusHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_pmbusHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(&g_pmbusHandle);
#elif defined(PLATFORM_ESP32)
    /* ESP32 I2C driver already enabled in ConfigureGpio */
#elif defined(PLATFORM_LINUX)
    /* Linux I2C device already opened in ConfigureGpio */
#endif
}

/******************************************************************************
 * @brief     : Disable PMBus module
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void PmBusDisable(void)
{
#ifdef PLATFORM_STM32F4
    HAL_I2C_DeInit(&g_pmbusHandle);
#elif defined(PLATFORM_STM32F1)
    HAL_I2C_DeInit(&g_pmbusHandle);
#elif defined(PLATFORM_ESP32)
    i2c_driver_delete(I2C_NUM_DEFAULT);
#elif defined(PLATFORM_LINUX)
    if (g_pmbusFd >= 0) {
        close(g_pmbusFd);
        g_pmbusFd = -1;
    }
#endif
}

/******************************************************************************
 * @brief     : Set PMBus clock speed
 * @param[in] : clockSpeed - Desired clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusSetClockSpeed(uint32_t clockSpeed)
{
#ifdef PLATFORM_STM32F4
    g_pmbusHandle.Init.ClockSpeed = clockSpeed;
    if (HAL_I2C_Init(&g_pmbusHandle) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_STM32F1)
    g_pmbusHandle.Init.ClockSpeed = clockSpeed;
    if (HAL_I2C_Init(&g_pmbusHandle) != HAL_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_ESP32)
    g_pmbusConfig.master.clk_speed = clockSpeed;
    if (i2c_param_config(I2C_NUM_DEFAULT, &g_pmbusConfig) != ESP_OK) {
        return -1;
    }
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux I2C speed is typically fixed or configured via device tree */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Generate PMBus START condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusGenerateStart(void)
{
#ifdef PLATFORM_STM32F4
    /* START condition is handled by HAL functions */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* START condition is handled by HAL functions */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* START condition is handled by ESP-IDF driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* START condition is handled by i2c-dev driver */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Generate PMBus STOP condition
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusGenerateStop(void)
{
#ifdef PLATFORM_STM32F4
    /* STOP condition is handled by HAL functions */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* STOP condition is handled by HAL functions */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* STOP condition is handled by ESP-IDF driver */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* STOP condition is handled by i2c-dev driver */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Send byte through PMBus
 * @param[in] : data - Byte to send
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusSendByte(uint8_t data)
{
#ifdef PLATFORM_STM32F4
    /* Byte transmission is handled by higher-level functions */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Byte transmission is handled by higher-level functions */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* Byte transmission is handled by higher-level functions */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Byte transmission is handled by higher-level functions */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read byte from PMBus
 * @param[in] : None
 * @param[out]: None
 * @return    : Byte read from PMBus
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t PmBusReadByteHal(void)
{
#ifdef PLATFORM_STM32F4
    /* Byte reception is handled by higher-level functions */
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Byte reception is handled by higher-level functions */
    return 0;
#elif defined(PLATFORM_ESP32)
    /* Byte reception is handled by higher-level functions */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Byte reception is handled by higher-level functions */
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Check PMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ACK, 0 if NACK
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusCheckAck(void)
{
#ifdef PLATFORM_STM32F4
    /* ACK checking is handled by HAL functions */
    return 1;
#elif defined(PLATFORM_STM32F1)
    /* ACK checking is handled by HAL functions */
    return 1;
#elif defined(PLATFORM_ESP32)
    /* ACK checking is handled by ESP-IDF driver */
    return 1;
#elif defined(PLATFORM_LINUX)
    /* ACK checking is handled by i2c-dev driver */
    return 1;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Send PMBus ACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void PmBusSendAck(void)
{
#ifdef PLATFORM_STM32F4
    /* ACK generation is handled by HAL functions */
#elif defined(PLATFORM_STM32F1)
    /* ACK generation is handled by HAL functions */
#elif defined(PLATFORM_ESP32)
    /* ACK generation is handled by ESP-IDF driver */
#elif defined(PLATFORM_LINUX)
    /* ACK generation is handled by i2c-dev driver */
#endif
}

/******************************************************************************
 * @brief     : Send PMBus NACK
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void PmBusSendNack(void)
{
#ifdef PLATFORM_STM32F4
    /* NACK generation is handled by HAL functions */
#elif defined(PLATFORM_STM32F1)
    /* NACK generation is handled by HAL functions */
#elif defined(PLATFORM_ESP32)
    /* NACK generation is handled by ESP-IDF driver */
#elif defined(PLATFORM_LINUX)
    /* NACK generation is handled by i2c-dev driver */
#endif
}

/******************************************************************************
 * @brief     : Write data to PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte, data - Pointer to data buffer, length - Number of bytes to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusWriteData(uint8_t deviceAddr, uint8_t command, uint8_t* data, uint16_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    uint8_t buffer[256];
    HAL_StatusTypeDef status;

    if (length > 255) {
        return -1;
    }

    /* Prepare buffer: command + data */
    buffer[0] = command;
    if (memcpy_s(&buffer[1], sizeof(buffer) - 1, data, length) != EOK) {
        return -1;
    }

    status = HAL_I2C_Master_Transmit(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), buffer, length + 1, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_STM32F1)
    uint8_t buffer[256];
    HAL_StatusTypeDef status;

    if (length > 255) {
        return -1;
    }

    /* Prepare buffer: command + data */
    buffer[0] = command;
    if (memcpy_s(&buffer[1], sizeof(buffer) - 1, data, length) != EOK) {
        return -1;
    }

    status = HAL_I2C_Master_Transmit(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), buffer, length + 1, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_ESP32)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_write(cmd, data, length, true);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    struct i2c_rdwr_ioctl_data msgset;
    struct i2c_msg msgs[1];
    uint8_t buffer[256];

    if (length > 255) {
        return -1;
    }

    /* Prepare buffer: command + data */
    buffer[0] = command;
    if (memcpy_s(&buffer[1], sizeof(buffer) - 1, data, length) != EOK) {
        return -1;
    }

    msgs[0].addr = deviceAddr;
    msgs[0].flags = 0; /* Write */
    msgs[0].len = length + 1;
    msgs[0].buf = buffer;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(g_pmbusFd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Read data from PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte, length - Number of bytes to read
 * @param[out]: buffer - Received data
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusReadData(uint8_t deviceAddr, uint8_t command, uint8_t* buffer, uint16_t length)
{
    if ((buffer == NULL) || (length == 0)) {
        return -1;
    }

#ifdef PLATFORM_STM32F4
    HAL_StatusTypeDef status;

    /* First write the command */
    status = HAL_I2C_Master_Transmit(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), &command, 1, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    /* Then read the data */
    status = HAL_I2C_Master_Receive(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), buffer, length, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_StatusTypeDef status;

    /* First write the command */
    status = HAL_I2C_Master_Transmit(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), &command, 1, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    /* Then read the data */
    status = HAL_I2C_Master_Receive(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), buffer, length, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_ESP32)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Write command */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);

    /* Read data */
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, buffer, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    struct i2c_rdwr_ioctl_data msgset;
    struct i2c_msg msgs[2];

    /* First message: write command */
    msgs[0].addr = deviceAddr;
    msgs[0].flags = 0; /* Write */
    msgs[0].len = 1;
    msgs[0].buf = &command;

    /* Second message: read data */
    msgs[1].addr = deviceAddr;
    msgs[1].flags = I2C_M_RD; /* Read */
    msgs[1].len = length;
    msgs[1].buf = buffer;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(g_pmbusFd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Write single byte to PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte, data - Byte to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusWriteByte(uint8_t deviceAddr, uint8_t command, uint8_t data)
{
    return PmBusWriteData(deviceAddr, command, &data, 1);
}

/******************************************************************************
 * @brief     : Read single byte from PMBus device
 * @param[in] : deviceAddr - PMBus device address, command - Command byte
 * @param[out]: data - Pointer to store received byte
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusReadByte(uint8_t deviceAddr, uint8_t command, uint8_t* data)
{
    if (data == NULL) {
        return -1;
    }

    return PmBusReadData(deviceAddr, command, data, 1);
}

/******************************************************************************
 * @brief     : Send PMBus command only (no data)
 * @param[in] : deviceAddr - PMBus device address, command - Command byte
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t PmBusSendCommand(uint8_t deviceAddr, uint8_t command)
{
#ifdef PLATFORM_STM32F4
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), &command, 1, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(&g_pmbusHandle, (uint16_t)(deviceAddr << 1), &command, 1, 1000);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_ESP32)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(I2C_NUM_DEFAULT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (result != ESP_OK) {
        return -1;
    }

    return 0;
#elif defined(PLATFORM_LINUX)
    struct i2c_rdwr_ioctl_data msgset;
    struct i2c_msg msgs[1];

    msgs[0].addr = deviceAddr;
    msgs[0].flags = 0; /* Write */
    msgs[0].len = 1;
    msgs[0].buf = &command;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(g_pmbusFd, I2C_RDWR, &msgset) < 0) {
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

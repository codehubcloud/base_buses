/******************************************************************************
 * @file    : sdio_hal.c
 * @brief   : SDIO Hardware Abstraction Layer implementation
 * @author  : Code Generator
 * @date    : 2025-10-01
 * @version : V1.0
 * @note    : Multi-platform SDIO HAL implementation
 ******************************************************************************/

#include <string.h>
#include "platform_config.h"
#include "sdio_hal.h"


/******************************************************************************
 * Platform: STM32F4
 ******************************************************************************/
#ifdef PLATFORM_STM32F4

/******************************************************************************
 * @brief     : Enable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F4 platform - Enables SDIO, GPIOC and GPIOD clocks
 *****************************************************************************/
int32_t SdioEnableClock(void)
{
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    return 0;
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : STM32F4 platform - Disables SDIO clock
 *****************************************************************************/
void SdioDisableClock(void)
{
    __HAL_RCC_SDIO_CLK_DISABLE();
}

/******************************************************************************
 * @brief     : Configure SDIO GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F4 platform - Configures PC8-PC12 (D0-D4, CLK), PD2 (CMD)
 *****************************************************************************/
int32_t SdioConfigureGpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SDIO GPIO Configuration: PC8-PC12 (D0-D4, CLK), PD2 (CMD) */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    return 0;
}

/******************************************************************************
 * @brief     : Enable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : STM32F4 platform - Activates SDIO clock output
 *****************************************************************************/
void SdioEnable(void)
{
    SDIO->CLKCR |= SDIO_CLKCR_CLKEN;
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : STM32F4 platform - Deactivates SDIO clock output
 *****************************************************************************/
void SdioDisable(void)
{
    SDIO->CLKCR &= ~SDIO_CLKCR_CLKEN;
}

/******************************************************************************
 * @brief     : Send command via HAL layer
 * @param[in] : cmd --Command index, arg --Command argument, crc --CRC7 checksum
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout or error
 * @note      : STM32F4 platform - Hardware command transmission with timeout
 *****************************************************************************/
int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    SDIO->ARG = arg;
    SDIO->CMD = (uint32_t)(SDIO_CMD_CPSMEN | SDIO_CMD_WAITRESP_0 | cmd);

    uint32_t timeout = 0x00010000;
    while ((SDIO->STA & (SDIO_STA_CMDSENT | SDIO_STA_CTIMEOUT)) == 0 && timeout > 0) {
        timeout--;
    }

    if (timeout == 0 || (SDIO->STA & SDIO_STA_CTIMEOUT)) {
        return -1;
    }

    SDIO->ICR = SDIO_ICR_CMDSENTC;
    return 0;
}

/******************************************************************************
 * @brief     : Read response via HAL layer
 * @param[in] : responseType --Type of response expected (short or long)
 * @param[out]: response --Buffer to store response data
 * @return    : 0 if success
 * @note      : STM32F4 platform - Reads response from RESP1-RESP4 registers
 *****************************************************************************/
int32_t SdioReadResponseHal(uint32_t* response, SdioResponseType_E responseType)
{
    if (responseType == SDIO_RESPONSE_SHORT) {
        response[0] = SDIO->RESP1;
    } else if (responseType == SDIO_RESPONSE_LONG) {
        response[0] = SDIO->RESP1;
        response[1] = SDIO->RESP2;
        response[2] = SDIO->RESP3;
        response[3] = SDIO->RESP4;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Read data via HAL layer
 * @param[in] : length --Number of bytes to read
 * @param[out]: data --Buffer to store received data
 * @return    : 0 if success
 * @note      : STM32F4 platform - Reads data from FIFO using polling
 *****************************************************************************/
int32_t SdioReadDataHal(uint8_t* data, uint32_t length)
{
    SDIO->DLEN = length;
    SDIO->DCTRL = SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR;

    uint32_t* data32 = (uint32_t*)data;
    uint32_t count = (length + 3) / 4;

    for (uint32_t i = 0; i < count; i++) {
        while ((SDIO->STA & SDIO_STA_RXDAVL) == 0) {
        }
        data32[i] = SDIO->FIFO;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write data via HAL layer
 * @param[in] : data --Pointer to data buffer, length --Number of bytes, crc16 --CRC16 checksum
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F4 platform - Writes data to FIFO using polling
 *****************************************************************************/
int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16)
{
    SDIO->DLEN = length;
    SDIO->DCTRL = SDIO_DCTRL_DTEN;

    uint32_t* data32 = (uint32_t*)data;
    uint32_t count = (length + 3) / 4;

    for (uint32_t i = 0; i < count; i++) {
        while ((SDIO->STA & SDIO_STA_TXFIFOHE) == 0) {
        }
        SDIO->FIFO = data32[i];
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set bus width via HAL layer
 * @param[in] : width --Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F4 platform - Configures CLKCR WIDBUS field
 *****************************************************************************/
int32_t SdioSetBusWidthHal(uint8_t width)
{
    uint32_t clkcr = SDIO->CLKCR & ~SDIO_CLKCR_WIDBUS;
    if (width == 4) {
        clkcr |= SDIO_CLKCR_WIDBUS_0;
    } else if (width == 8) {
        clkcr |= SDIO_CLKCR_WIDBUS_1;
    }
    SDIO->CLKCR = clkcr;
    return 0;
}

/******************************************************************************
 * @brief     : Set clock speed via HAL layer
 * @param[in] : speed --Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F4 platform - Calculates divider from 48MHz base clock
 *****************************************************************************/
int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    uint32_t div = (48000000 / speed) - 2;
    if (div > 0xFF) {
        div = 0xFF;
    }
    SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV) | div;
    return 0;
}

/******************************************************************************
 * @brief     : Check if SDIO is busy
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if busy, 0 if idle
 * @note      : STM32F4 platform - Checks CMDACT flag in status register
 *****************************************************************************/
int32_t SdioIsBusy(void)
{
    return (SDIO->STA & SDIO_STA_CMDACT) ? 1 : 0;
}

/******************************************************************************
 * @brief     : Wait for SDIO command completion
 * @param[in] : timeout --Timeout in loop iterations
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout
 * @note      : STM32F4 platform - Polls CMDSENT flag
 *****************************************************************************/
int32_t SdioWaitCommandComplete(uint32_t timeout)
{
    while (timeout > 0) {
        if ((SDIO->STA & SDIO_STA_CMDSENT) != 0) {
            return 0;
        }
        timeout--;
    }
    return -1;
}

/******************************************************************************
 * @brief     : Wait for SDIO data transfer completion
 * @param[in] : timeout --Timeout in loop iterations
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout
 * @note      : STM32F4 platform - Polls DATAEND flag
 *****************************************************************************/
int32_t SdioWaitDataComplete(uint32_t timeout)
{
    while (timeout > 0) {
        if ((SDIO->STA & SDIO_STA_DATAEND) != 0) {
            return 0;
        }
        timeout--;
    }
    return -1;
}

#endif /* PLATFORM_STM32F4 */

/******************************************************************************
 * Platform: STM32F1
 ******************************************************************************/
#ifdef PLATFORM_STM32F1

/******************************************************************************
 * @brief     : Enable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F1 platform - Enables SDIO, GPIOC and GPIOD clocks
 *****************************************************************************/
int32_t SdioEnableClock(void)
{
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    return 0;
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : STM32F1 platform - Disables SDIO clock
 *****************************************************************************/
void SdioDisableClock(void)
{
    __HAL_RCC_SDIO_CLK_DISABLE();
}

/******************************************************************************
 * @brief     : Configure SDIO GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F1 platform - Configures PC8-PC12 (D0-D4, CLK), PD2 (CMD)
 *****************************************************************************/
int32_t SdioConfigureGpio(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SDIO GPIO Configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    return 0;
}

/******************************************************************************
 * @brief     : Enable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : STM32F1 platform - Activates SDIO clock output
 *****************************************************************************/
void SdioEnable(void)
{
    SDIO->CLKCR |= SDIO_CLKCR_CLKEN;
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : STM32F1 platform - Deactivates SDIO clock output
 *****************************************************************************/
void SdioDisable(void)
{
    SDIO->CLKCR &= ~SDIO_CLKCR_CLKEN;
}

/******************************************************************************
 * @brief     : Send command via HAL layer
 * @param[in] : cmd --Command index, arg --Command argument, crc --CRC7 checksum
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout or error
 * @note      : STM32F1 platform - Hardware command transmission with timeout
 *****************************************************************************/
int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    SDIO->ARG = arg;
    SDIO->CMD = (uint32_t)(SDIO_CMD_CPSMEN | SDIO_CMD_WAITRESP_0 | cmd);

    uint32_t timeout = 0x00010000;
    while ((SDIO->STA & (SDIO_STA_CMDSENT | SDIO_STA_CTIMEOUT)) == 0 && timeout > 0) {
        timeout--;
    }

    if (timeout == 0 || (SDIO->STA & SDIO_STA_CTIMEOUT)) {
        return -1;
    }

    SDIO->ICR = SDIO_ICR_CMDSENTC;
    return 0;
}

/******************************************************************************
 * @brief     : Read response via HAL layer
 * @param[in] : responseType --Type of response expected (short or long)
 * @param[out]: response --Buffer to store response data
 * @return    : 0 if success
 * @note      : STM32F1 platform - Reads response from RESP1-RESP4 registers
 *****************************************************************************/
int32_t SdioReadResponseHal(uint32_t* response, SdioResponseType_E responseType)
{
    if (responseType == SDIO_RESPONSE_SHORT) {
        response[0] = SDIO->RESP1;
    } else if (responseType == SDIO_RESPONSE_LONG) {
        response[0] = SDIO->RESP1;
        response[1] = SDIO->RESP2;
        response[2] = SDIO->RESP3;
        response[3] = SDIO->RESP4;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Read data via HAL layer
 * @param[in] : length --Number of bytes to read
 * @param[out]: data --Buffer to store received data
 * @return    : 0 if success
 * @note      : STM32F1 platform - Reads data from FIFO using polling
 *****************************************************************************/
int32_t SdioReadDataHal(uint8_t* data, uint32_t length)
{
    SDIO->DLEN = length;
    SDIO->DCTRL = SDIO_DCTRL_DTEN | SDIO_DCTRL_DTDIR;

    uint32_t* data32 = (uint32_t*)data;
    uint32_t count = (length + 3) / 4;

    for (uint32_t i = 0; i < count; i++) {
        while ((SDIO->STA & SDIO_STA_RXDAVL) == 0) {
        }
        data32[i] = SDIO->FIFO;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Write data via HAL layer
 * @param[in] : data --Pointer to data buffer, length --Number of bytes, crc16 --CRC16 checksum
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F1 platform - Writes data to FIFO using polling
 *****************************************************************************/
int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16)
{
    SDIO->DLEN = length;
    SDIO->DCTRL = SDIO_DCTRL_DTEN;

    uint32_t* data32 = (uint32_t*)data;
    uint32_t count = (length + 3) / 4;

    for (uint32_t i = 0; i < count; i++) {
        while ((SDIO->STA & SDIO_STA_TXFIFOHE) == 0) {
        }
        SDIO->FIFO = data32[i];
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set bus width via HAL layer
 * @param[in] : width --Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F1 platform - Configures CLKCR WIDBUS field
 *****************************************************************************/
int32_t SdioSetBusWidthHal(uint8_t width)
{
    uint32_t clkcr = SDIO->CLKCR & ~SDIO_CLKCR_WIDBUS;
    if (width == 4) {
        clkcr |= SDIO_CLKCR_WIDBUS_0;
    } else if (width == 8) {
        clkcr |= SDIO_CLKCR_WIDBUS_1;
    }
    SDIO->CLKCR = clkcr;
    return 0;
}

/******************************************************************************
 * @brief     : Set clock speed via HAL layer
 * @param[in] : speed --Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success
 * @note      : STM32F1 platform - Calculates divider from 48MHz base clock
 *****************************************************************************/
int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    uint32_t div = (48000000 / speed) - 2;
    if (div > 0xFF) {
        div = 0xFF;
    }
    SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV) | div;
    return 0;
}

/******************************************************************************
 * @brief     : Check if SDIO is busy
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if busy, 0 if idle
 * @note      : STM32F1 platform - Checks CMDACT flag in status register
 *****************************************************************************/
int32_t SdioIsBusy(void)
{
    return (SDIO->STA & SDIO_STA_CMDACT) ? 1 : 0;
}

/******************************************************************************
 * @brief     : Wait for SDIO command completion
 * @param[in] : timeout --Timeout in loop iterations
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout
 * @note      : STM32F1 platform - Polls CMDSENT flag
 *****************************************************************************/
int32_t SdioWaitCommandComplete(uint32_t timeout)
{
    while (timeout > 0) {
        if ((SDIO->STA & SDIO_STA_CMDSENT) != 0) {
            return 0;
        }
        timeout--;
    }
    return -1;
}

/******************************************************************************
 * @brief     : Wait for SDIO data transfer completion
 * @param[in] : timeout --Timeout in loop iterations
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout
 * @note      : STM32F1 platform - Polls DATAEND flag
 *****************************************************************************/
int32_t SdioWaitDataComplete(uint32_t timeout)
{
    while (timeout > 0) {
        if ((SDIO->STA & SDIO_STA_DATAEND) != 0) {
            return 0;
        }
        timeout--;
    }
    return -1;
}

#endif /* PLATFORM_STM32F1 */

/******************************************************************************
 * Platform: ESP32
 ******************************************************************************/
#ifdef PLATFORM_ESP32

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"

static sdmmc_host_t host = SDMMC_HOST_DEFAULT();

/******************************************************************************
 * @brief     : Enable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : ESP32 platform - No operation required, managed by ESP-IDF
 *****************************************************************************/
int32_t SdioEnableClock(void)
{
    return 0;
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : ESP32 platform - No operation required, managed by ESP-IDF
 *****************************************************************************/
void SdioDisableClock(void) {}

/******************************************************************************
 * @brief     : Configure SDIO GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : ESP32 platform - No operation required, managed by ESP-IDF
 *****************************************************************************/
int32_t SdioConfigureGpio(void)
{
    return 0;
}

/******************************************************************************
 * @brief     : Enable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : ESP32 platform - Initializes SDMMC host controller
 *****************************************************************************/
void SdioEnable(void)
{
    sdmmc_host_init();
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : ESP32 platform - Deinitializes SDMMC host controller
 *****************************************************************************/
void SdioDisable(void)
{
    sdmmc_host_deinit();
}

/******************************************************************************
 * @brief     : Send command via HAL layer
 * @param[in] : cmd --Command index, arg --Command argument, crc --CRC7 checksum
 * @param[out]: None
 * @return    : 0 if success, error code otherwise
 * @note      : ESP32 platform - Uses ESP-IDF SDMMC driver for transaction
 *****************************************************************************/
int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    sdmmc_command_t command = {.opcode = cmd, .arg = arg, .flags = SCF_CMD_AC | SCF_RSP_R1};
    return sdmmc_host_do_transaction(host.slot, &command);
}

/******************************************************************************
 * @brief     : Read response via HAL layer
 * @param[in] : responseType --Type of response expected (short or long)
 * @param[out]: response --Buffer to store response data
 * @return    : 0 if success
 * @note      : ESP32 platform - Response handled internally by ESP-IDF driver
 *****************************************************************************/
int32_t SdioReadResponseHal(uint32_t* response, SdioResponseType_E responseType)
{
    /* ESP32 HAL handles response internally */
    return 0;
}

/******************************************************************************
 * @brief     : Read data via HAL layer
 * @param[in] : length --Number of bytes to read
 * @param[out]: data --Buffer to store received data
 * @return    : 0 if success
 * @note      : ESP32 platform - Uses ESP-IDF SDMMC read functions
 *****************************************************************************/
int32_t SdioReadDataHal(uint8_t* data, uint32_t length)
{
    /* Use ESP32 SDMMC read functions */
    return 0;
}

/******************************************************************************
 * @brief     : Write data via HAL layer
 * @param[in] : data --Pointer to data buffer, length --Number of bytes, crc16 --CRC16 checksum
 * @param[out]: None
 * @return    : 0 if success
 * @note      : ESP32 platform - Uses ESP-IDF SDMMC write functions
 *****************************************************************************/
int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16)
{
    /* Use ESP32 SDMMC write functions */
    return 0;
}

/******************************************************************************
 * @brief     : Set bus width via HAL layer
 * @param[in] : width --Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success, error code otherwise
 * @note      : ESP32 platform - Configures bus width via ESP-IDF driver
 *****************************************************************************/
int32_t SdioSetBusWidthHal(uint8_t width)
{
    return sdmmc_host_set_bus_width(host.slot, width);
}

/******************************************************************************
 * @brief     : Set clock speed via HAL layer
 * @param[in] : speed --Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success, error code otherwise
 * @note      : ESP32 platform - Sets clock via ESP-IDF driver (converts to kHz)
 *****************************************************************************/
int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    return sdmmc_host_set_card_clk(host.slot, speed / 1000);
}

/******************************************************************************
 * @brief     : Check if SDIO is busy
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 (always idle)
 * @note      : ESP32 platform - Busy state managed internally by ESP-IDF
 *****************************************************************************/
int32_t SdioIsBusy(void)
{
    return 0;
}

/******************************************************************************
 * @brief     : Wait for SDIO command completion
 * @param[in] : timeout --Timeout value (unused)
 * @param[out]: None
 * @return    : 0 (always success)
 * @note      : ESP32 platform - Synchronous operation, no wait needed
 *****************************************************************************/
int32_t SdioWaitCommandComplete(uint32_t timeout)
{
    return 0;
}

/******************************************************************************
 * @brief     : Wait for SDIO data transfer completion
 * @param[in] : timeout --Timeout value (unused)
 * @param[out]: None
 * @return    : 0 (always success)
 * @note      : ESP32 platform - Synchronous operation, no wait needed
 *****************************************************************************/
int32_t SdioWaitDataComplete(uint32_t timeout)
{
    return 0;
}

#endif /* PLATFORM_ESP32 */

/******************************************************************************
 * Platform: Linux
 ******************************************************************************/
#ifdef PLATFORM_LINUX

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>


static int sdioFd = -1;

/******************************************************************************
 * @brief     : Enable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : Linux platform - Simulated operation, prints debug message
 *****************************************************************************/
int32_t SdioEnableClock(void)
{
    printf("[SDIO HAL] Clock enabled (simulated)\n");
    return 0;
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral clock
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Linux platform - Simulated operation, prints debug message
 *****************************************************************************/
void SdioDisableClock(void)
{
    printf("[SDIO HAL] Clock disabled (simulated)\n");
}

/******************************************************************************
 * @brief     : Configure SDIO GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success
 * @note      : Linux platform - Simulated operation, prints debug message
 *****************************************************************************/
int32_t SdioConfigureGpio(void)
{
    printf("[SDIO HAL] GPIO configured (simulated)\n");
    return 0;
}

/******************************************************************************
 * @brief     : Enable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Linux platform - Simulated operation, prints debug message
 *****************************************************************************/
void SdioEnable(void)
{
    printf("[SDIO HAL] SDIO enabled (simulated)\n");
}

/******************************************************************************
 * @brief     : Disable SDIO peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Linux platform - Simulated operation, prints debug message
 *****************************************************************************/
void SdioDisable(void)
{
    printf("[SDIO HAL] SDIO disabled (simulated)\n");
}

/******************************************************************************
 * @brief     : Send command via HAL layer
 * @param[in] : cmd --Command index, arg --Command argument, crc --CRC7 checksum
 * @param[out]: None
 * @return    : 0 if success
 * @note      : Linux platform - Simulated operation, prints command details
 *****************************************************************************/
int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    printf("[SDIO HAL] Send CMD%d, ARG=0x%08X (simulated)\n", cmd, arg);
    return 0;
}

/******************************************************************************
 * @brief     : Read response via HAL layer
 * @param[in] : responseType --Type of response expected (short or long)
 * @param[out]: response --Buffer to store response data
 * @return    : 0 if success
 * @note      : Linux platform - Returns simulated response data
 *****************************************************************************/
int32_t SdioReadResponseHal(uint32_t* response, SdioResponseType_E responseType)
{
    if (responseType == SDIO_RESPONSE_SHORT) {
        response[0] = 0x00000900;
        printf("[SDIO HAL] Read short response: 0x%08X (simulated)\n", response[0]);
    } else if (responseType == SDIO_RESPONSE_LONG) {
        response[0] = 0x12345678;
        response[1] = 0x9ABCDEF0;
        response[2] = 0x11223344;
        response[3] = 0x55667788;
        printf("[SDIO HAL] Read long response (simulated)\n");
    }
    return 0;
}

/******************************************************************************
 * @brief     : Read data via HAL layer
 * @param[in] : length --Number of bytes to read
 * @param[out]: data --Buffer to store received data
 * @return    : 0 if success, -1 if error
 * @note      : Linux platform - Fills buffer with pattern 0xAA for simulation
 *****************************************************************************/
int32_t SdioReadDataHal(uint8_t* data, uint32_t length)
{
    printf("[SDIO HAL] Read %u bytes (simulated)\n", length);
    if (memset_s(data, length, 0xAA, length) != EOK) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Write data via HAL layer
 * @param[in] : data --Pointer to data buffer, length --Number of bytes, crc16 --CRC16 checksum
 * @param[out]: None
 * @return    : 0 if success
 * @note      : Linux platform - Simulated operation, prints write details
 *****************************************************************************/
int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16)
{
    printf("[SDIO HAL] Write %u bytes, CRC16=0x%04X (simulated)\n", length, crc16);
    return 0;
}

/******************************************************************************
 * @brief     : Set bus width via HAL layer
 * @param[in] : width --Bus width (1, 4, or 8 bits)
 * @param[out]: None
 * @return    : 0 if success
 * @note      : Linux platform - Simulated operation, prints bus width
 *****************************************************************************/
int32_t SdioSetBusWidthHal(uint8_t width)
{
    printf("[SDIO HAL] Set bus width to %d-bit (simulated)\n", width);
    return 0;
}

/******************************************************************************
 * @brief     : Set clock speed via HAL layer
 * @param[in] : speed --Clock speed in Hz
 * @param[out]: None
 * @return    : 0 if success
 * @note      : Linux platform - Simulated operation, prints clock speed
 *****************************************************************************/
int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    printf("[SDIO HAL] Set clock speed to %u Hz (simulated)\n", speed);
    return 0;
}

/******************************************************************************
 * @brief     : Check if SDIO is busy
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 (always idle)
 * @note      : Linux platform - Always returns idle state
 *****************************************************************************/
int32_t SdioIsBusy(void)
{
    return 0;
}

/******************************************************************************
 * @brief     : Wait for SDIO command completion
 * @param[in] : timeout --Timeout value (unused)
 * @param[out]: None
 * @return    : 0 (always success)
 * @note      : Linux platform - Simulates delay with 100us sleep
 *****************************************************************************/
int32_t SdioWaitCommandComplete(uint32_t timeout)
{
    usleep(100);
    return 0;
}

/******************************************************************************
 * @brief     : Wait for SDIO data transfer completion
 * @param[in] : timeout --Timeout value (unused)
 * @param[out]: None
 * @return    : 0 (always success)
 * @note      : Linux platform - Simulates delay with 100us sleep
 *****************************************************************************/
int32_t SdioWaitDataComplete(uint32_t timeout)
{
    usleep(100);
    return 0;
}

#endif /* PLATFORM_LINUX */

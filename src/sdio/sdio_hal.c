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

int32_t SdioEnableClock(void)
{
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    return 0;
}

void SdioDisableClock(void)
{
    __HAL_RCC_SDIO_CLK_DISABLE();
}

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

void SdioEnable(void)
{
    SDIO->CLKCR |= SDIO_CLKCR_CLKEN;
}

void SdioDisable(void)
{
    SDIO->CLKCR &= ~SDIO_CLKCR_CLKEN;
}

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

int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    uint32_t div = (48000000 / speed) - 2;
    if (div > 0xFF) {
        div = 0xFF;
    }
    SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV) | div;
    return 0;
}

int32_t SdioIsBusy(void)
{
    return (SDIO->STA & SDIO_STA_CMDACT) ? 1 : 0;
}

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

int32_t SdioEnableClock(void)
{
    __HAL_RCC_SDIO_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    return 0;
}

void SdioDisableClock(void)
{
    __HAL_RCC_SDIO_CLK_DISABLE();
}

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

void SdioEnable(void)
{
    SDIO->CLKCR |= SDIO_CLKCR_CLKEN;
}

void SdioDisable(void)
{
    SDIO->CLKCR &= ~SDIO_CLKCR_CLKEN;
}

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

int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    uint32_t div = (48000000 / speed) - 2;
    if (div > 0xFF) {
        div = 0xFF;
    }
    SDIO->CLKCR = (SDIO->CLKCR & ~SDIO_CLKCR_CLKDIV) | div;
    return 0;
}

int32_t SdioIsBusy(void)
{
    return (SDIO->STA & SDIO_STA_CMDACT) ? 1 : 0;
}

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

int32_t SdioEnableClock(void)
{
    return 0;
}

void SdioDisableClock(void) {}

int32_t SdioConfigureGpio(void)
{
    return 0;
}

void SdioEnable(void)
{
    sdmmc_host_init();
}

void SdioDisable(void)
{
    sdmmc_host_deinit();
}

int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    sdmmc_command_t command = {.opcode = cmd, .arg = arg, .flags = SCF_CMD_AC | SCF_RSP_R1};
    return sdmmc_host_do_transaction(host.slot, &command);
}

int32_t SdioReadResponseHal(uint32_t* response, SdioResponseType_E responseType)
{
    /* ESP32 HAL handles response internally */
    return 0;
}

int32_t SdioReadDataHal(uint8_t* data, uint32_t length)
{
    /* Use ESP32 SDMMC read functions */
    return 0;
}

int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16)
{
    /* Use ESP32 SDMMC write functions */
    return 0;
}

int32_t SdioSetBusWidthHal(uint8_t width)
{
    return sdmmc_host_set_bus_width(host.slot, width);
}

int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    return sdmmc_host_set_card_clk(host.slot, speed / 1000);
}

int32_t SdioIsBusy(void)
{
    return 0;
}

int32_t SdioWaitCommandComplete(uint32_t timeout)
{
    return 0;
}

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

int32_t SdioEnableClock(void)
{
    printf("[SDIO HAL] Clock enabled (simulated)\n");
    return 0;
}

void SdioDisableClock(void)
{
    printf("[SDIO HAL] Clock disabled (simulated)\n");
}

int32_t SdioConfigureGpio(void)
{
    printf("[SDIO HAL] GPIO configured (simulated)\n");
    return 0;
}

void SdioEnable(void)
{
    printf("[SDIO HAL] SDIO enabled (simulated)\n");
}

void SdioDisable(void)
{
    printf("[SDIO HAL] SDIO disabled (simulated)\n");
}

int32_t SdioSendCommandHal(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    printf("[SDIO HAL] Send CMD%d, ARG=0x%08X (simulated)\n", cmd, arg);
    return 0;
}

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

int32_t SdioReadDataHal(uint8_t* data, uint32_t length)
{
    printf("[SDIO HAL] Read %u bytes (simulated)\n", length);
    memset(data, 0xAA, length);
    return 0;
}

int32_t SdioWriteDataHal(uint8_t* data, uint32_t length, uint16_t crc16)
{
    printf("[SDIO HAL] Write %u bytes, CRC16=0x%04X (simulated)\n", length, crc16);
    return 0;
}

int32_t SdioSetBusWidthHal(uint8_t width)
{
    printf("[SDIO HAL] Set bus width to %d-bit (simulated)\n", width);
    return 0;
}

int32_t SdioSetClockSpeedHal(uint32_t speed)
{
    printf("[SDIO HAL] Set clock speed to %u Hz (simulated)\n", speed);
    return 0;
}

int32_t SdioIsBusy(void)
{
    return 0;
}

int32_t SdioWaitCommandComplete(uint32_t timeout)
{
    usleep(100);
    return 0;
}

int32_t SdioWaitDataComplete(uint32_t timeout)
{
    usleep(100);
    return 0;
}

#endif /* PLATFORM_LINUX */

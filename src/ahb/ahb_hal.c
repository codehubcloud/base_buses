#include "../platform_config.h"
#include "ahb.h"
#include "ahb_hal.h"


/* Platform-specific includes and implementations */

#ifdef PLATFORM_STM32F4

/* STM32F4 AHB Implementation - Memory-mapped AHB peripherals */
#include "stm32f4xx_hal.h"

static volatile uint32_t ahbAddress = 0;
static volatile uint32_t ahbData = 0;
static volatile uint8_t ahbTransType = AHB_TRANS_IDLE;
static volatile uint8_t ahbBurstType = AHB_BURST_SINGLE;
static volatile uint8_t ahbSize = AHB_SIZE_WORD;
static volatile uint8_t ahbWrite = 0;
static volatile uint8_t ahbProt = 0;
static volatile uint8_t ahbResp = AHB_RESP_OKAY;
static volatile uint8_t ahbReady = 1;

int32_t AhbEnableClock(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    return 0;
}

void AhbDisableClock(void)
{
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_DMA2_CLK_DISABLE();
}

int32_t AhbConfigureBusMatrix(void)
{
    /* Configure AHB bus matrix priority for DMA and peripherals */
    FLASH->ACR |= FLASH_ACR_PRFTEN;
    return 0;
}

void AhbResetController(void)
{
    ahbAddress = 0;
    ahbData = 0;
    ahbTransType = AHB_TRANS_IDLE;
    ahbBurstType = AHB_BURST_SINGLE;
    ahbSize = AHB_SIZE_WORD;
    ahbWrite = 0;
    ahbProt = 0;
    ahbResp = AHB_RESP_OKAY;
    ahbReady = 1;
}

void AhbEnableMaster(void)
{
    /* Enable AHB master interface */
    SCB->CCR |= SCB_CCR_IC_Msk;
}

void AhbDisableMaster(void)
{
    /* Disable AHB master interface */
    SCB->CCR &= ~SCB_CCR_IC_Msk;
}

void AhbSetAddress(uint32_t address)
{
    ahbAddress = address;
}

void AhbSetTransferType(uint8_t transferType)
{
    ahbTransType = transferType;
}

void AhbSetBurstType(uint8_t burstType)
{
    ahbBurstType = burstType;
}

void AhbSetTransferSize(uint8_t transferSize)
{
    ahbSize = transferSize;
}

void AhbSetWrite(uint8_t write)
{
    ahbWrite = write;
}

void AhbSetProtection(uint8_t protection)
{
    ahbProt = protection;
}

void AhbWriteData(uint32_t data)
{
    volatile uint32_t* targetAddress = (volatile uint32_t*)ahbAddress;

    if (ahbSize == AHB_SIZE_BYTE) {
        *(volatile uint8_t*)targetAddress = (uint8_t)data;
    } else if (ahbSize == AHB_SIZE_HALFWORD) {
        *(volatile uint16_t*)targetAddress = (uint16_t)data;
    } else {
        *targetAddress = data;
    }

    ahbData = data;
}

uint32_t AhbReadData(void)
{
    volatile uint32_t* sourceAddress = (volatile uint32_t*)ahbAddress;
    uint32_t data = 0;

    if (ahbSize == AHB_SIZE_BYTE) {
        data = *(volatile uint8_t*)sourceAddress;
    } else if (ahbSize == AHB_SIZE_HALFWORD) {
        data = *(volatile uint16_t*)sourceAddress;
    } else {
        data = *sourceAddress;
    }

    ahbData = data;
    return data;
}

uint8_t AhbGetResponse(void)
{
    return ahbResp;
}

int32_t AhbCheckReady(void)
{
    return ahbReady ? 1 : 0;
}

int32_t AhbConfigurePriority(uint8_t priority)
{
    /* Configure bus matrix priority */
    (void)priority;
    return 0;
}

#elif defined(PLATFORM_STM32F1)

/* STM32F1 AHB Implementation - Memory-mapped AHB peripherals */
#include "stm32f1xx_hal.h"

static volatile uint32_t ahbAddress = 0;
static volatile uint32_t ahbData = 0;
static volatile uint8_t ahbTransType = AHB_TRANS_IDLE;
static volatile uint8_t ahbBurstType = AHB_BURST_SINGLE;
static volatile uint8_t ahbSize = AHB_SIZE_WORD;
static volatile uint8_t ahbWrite = 0;
static volatile uint8_t ahbProt = 0;
static volatile uint8_t ahbResp = AHB_RESP_OKAY;
static volatile uint8_t ahbReady = 1;

int32_t AhbEnableClock(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    return 0;
}

void AhbDisableClock(void)
{
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_DMA2_CLK_DISABLE();
}

int32_t AhbConfigureBusMatrix(void)
{
    /* Configure AHB bus for STM32F1 */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    return 0;
}

void AhbResetController(void)
{
    ahbAddress = 0;
    ahbData = 0;
    ahbTransType = AHB_TRANS_IDLE;
    ahbBurstType = AHB_BURST_SINGLE;
    ahbSize = AHB_SIZE_WORD;
    ahbWrite = 0;
    ahbProt = 0;
    ahbResp = AHB_RESP_OKAY;
    ahbReady = 1;
}

void AhbEnableMaster(void)
{
    /* Enable AHB master interface */
}

void AhbDisableMaster(void)
{
    /* Disable AHB master interface */
}

void AhbSetAddress(uint32_t address)
{
    ahbAddress = address;
}

void AhbSetTransferType(uint8_t transferType)
{
    ahbTransType = transferType;
}

void AhbSetBurstType(uint8_t burstType)
{
    ahbBurstType = burstType;
}

void AhbSetTransferSize(uint8_t transferSize)
{
    ahbSize = transferSize;
}

void AhbSetWrite(uint8_t write)
{
    ahbWrite = write;
}

void AhbSetProtection(uint8_t protection)
{
    ahbProt = protection;
}

void AhbWriteData(uint32_t data)
{
    volatile uint32_t* targetAddress = (volatile uint32_t*)ahbAddress;

    if (ahbSize == AHB_SIZE_BYTE) {
        *(volatile uint8_t*)targetAddress = (uint8_t)data;
    } else if (ahbSize == AHB_SIZE_HALFWORD) {
        *(volatile uint16_t*)targetAddress = (uint16_t)data;
    } else {
        *targetAddress = data;
    }

    ahbData = data;
}

uint32_t AhbReadData(void)
{
    volatile uint32_t* sourceAddress = (volatile uint32_t*)ahbAddress;
    uint32_t data = 0;

    if (ahbSize == AHB_SIZE_BYTE) {
        data = *(volatile uint8_t*)sourceAddress;
    } else if (ahbSize == AHB_SIZE_HALFWORD) {
        data = *(volatile uint16_t*)sourceAddress;
    } else {
        data = *sourceAddress;
    }

    ahbData = data;
    return data;
}

uint8_t AhbGetResponse(void)
{
    return ahbResp;
}

int32_t AhbCheckReady(void)
{
    return ahbReady ? 1 : 0;
}

int32_t AhbConfigurePriority(uint8_t priority)
{
    /* Configure bus priority for STM32F1 */
    (void)priority;
    return 0;
}

#elif defined(PLATFORM_ESP32)

/* ESP32 does not use ARM AHB bus architecture - Not supported */
#include <stdio.h>

int32_t AhbEnableClock(void)
{
    printf("AHB: ESP32 does not support ARM AHB bus architecture\n");
    return -1;
}

void AhbDisableClock(void)
{
    printf("AHB: Not supported on ESP32\n");
}

int32_t AhbConfigureBusMatrix(void)
{
    return -1;
}

void AhbResetController(void) {}

void AhbEnableMaster(void) {}

void AhbDisableMaster(void) {}

void AhbSetAddress(uint32_t address)
{
    (void)address;
}

void AhbSetTransferType(uint8_t transferType)
{
    (void)transferType;
}

void AhbSetBurstType(uint8_t burstType)
{
    (void)burstType;
}

void AhbSetTransferSize(uint8_t transferSize)
{
    (void)transferSize;
}

void AhbSetWrite(uint8_t write)
{
    (void)write;
}

void AhbSetProtection(uint8_t protection)
{
    (void)protection;
}

void AhbWriteData(uint32_t data)
{
    (void)data;
}

uint32_t AhbReadData(void)
{
    return 0;
}

uint8_t AhbGetResponse(void)
{
    return AHB_RESP_ERROR;
}

int32_t AhbCheckReady(void)
{
    return 0;
}

int32_t AhbConfigurePriority(uint8_t priority)
{
    (void)priority;
    return -1;
}

#elif defined(PLATFORM_LINUX)

/* Linux Simulation Implementation */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AHB_MEMORY_SIZE 0x10000

static uint8_t ahbMemory[AHB_MEMORY_SIZE];
static volatile uint32_t ahbAddress = 0;
static volatile uint32_t ahbData = 0;
static volatile uint8_t ahbTransType = AHB_TRANS_IDLE;
static volatile uint8_t ahbBurstType = AHB_BURST_SINGLE;
static volatile uint8_t ahbSize = AHB_SIZE_WORD;
static volatile uint8_t ahbWrite = 0;
static volatile uint8_t ahbProt = 0;
static volatile uint8_t ahbResp = AHB_RESP_OKAY;
static volatile uint8_t ahbReady = 1;
static uint8_t ahbInitialized = 0;

int32_t AhbEnableClock(void)
{
    printf("AHB: Enabling simulated AHB bus clock\n");
    if (!ahbInitialized) {
        memset(ahbMemory, 0, AHB_MEMORY_SIZE);
        ahbInitialized = 1;
    }
    return 0;
}

void AhbDisableClock(void)
{
    printf("AHB: Disabling simulated AHB bus clock\n");
}

int32_t AhbConfigureBusMatrix(void)
{
    printf("AHB: Configuring simulated bus matrix\n");
    return 0;
}

void AhbResetController(void)
{
    printf("AHB: Resetting controller\n");
    ahbAddress = 0;
    ahbData = 0;
    ahbTransType = AHB_TRANS_IDLE;
    ahbBurstType = AHB_BURST_SINGLE;
    ahbSize = AHB_SIZE_WORD;
    ahbWrite = 0;
    ahbProt = 0;
    ahbResp = AHB_RESP_OKAY;
    ahbReady = 1;
}

void AhbEnableMaster(void)
{
    printf("AHB: Enabling master interface\n");
}

void AhbDisableMaster(void)
{
    printf("AHB: Disabling master interface\n");
}

void AhbSetAddress(uint32_t address)
{
    ahbAddress = address % AHB_MEMORY_SIZE;
}

void AhbSetTransferType(uint8_t transferType)
{
    ahbTransType = transferType;
}

void AhbSetBurstType(uint8_t burstType)
{
    ahbBurstType = burstType;
}

void AhbSetTransferSize(uint8_t transferSize)
{
    ahbSize = transferSize;
}

void AhbSetWrite(uint8_t write)
{
    ahbWrite = write;
}

void AhbSetProtection(uint8_t protection)
{
    ahbProt = protection;
}

void AhbWriteData(uint32_t data)
{
    uint32_t offset = ahbAddress % AHB_MEMORY_SIZE;

    if (ahbSize == AHB_SIZE_BYTE) {
        ahbMemory[offset] = (uint8_t)data;
    } else if (ahbSize == AHB_SIZE_HALFWORD) {
        if (offset + 1 < AHB_MEMORY_SIZE) {
            *(uint16_t*)&ahbMemory[offset] = (uint16_t)data;
        }
    } else {
        if (offset + 3 < AHB_MEMORY_SIZE) {
            *(uint32_t*)&ahbMemory[offset] = data;
        }
    }

    ahbData = data;
    printf("AHB: Write 0x%08X to address 0x%08X (size=%d)\n", data, ahbAddress, ahbSize);
}

uint32_t AhbReadData(void)
{
    uint32_t data = 0;
    uint32_t offset = ahbAddress % AHB_MEMORY_SIZE;

    if (ahbSize == AHB_SIZE_BYTE) {
        data = ahbMemory[offset];
    } else if (ahbSize == AHB_SIZE_HALFWORD) {
        if (offset + 1 < AHB_MEMORY_SIZE) {
            data = *(uint16_t*)&ahbMemory[offset];
        }
    } else {
        if (offset + 3 < AHB_MEMORY_SIZE) {
            data = *(uint32_t*)&ahbMemory[offset];
        }
    }

    ahbData = data;
    printf("AHB: Read 0x%08X from address 0x%08X (size=%d)\n", data, ahbAddress, ahbSize);
    return data;
}

uint8_t AhbGetResponse(void)
{
    return ahbResp;
}

int32_t AhbCheckReady(void)
{
    return ahbReady ? 1 : 0;
}

int32_t AhbConfigurePriority(uint8_t priority)
{
    printf("AHB: Setting priority to %d\n", priority);
    return 0;
}

#endif

#include "apb_hal.h"
#include "platform_config.h"
#include "securec.h"


/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
/* STM32F4 uses memory-mapped APB1/APB2 peripherals */
static volatile uint32_t* g_apbAddress = NULL;
static uint32_t g_apbWriteData = 0;
static uint32_t g_apbReadData = 0;
static uint8_t g_apbPenable = 0;
static int32_t g_apbSlaveError = APB_PSLVERR_OK;
#endif

#ifdef PLATFORM_STM32F1
/* STM32F1 uses memory-mapped APB1/APB2 peripherals */
static volatile uint32_t* g_apbAddress = NULL;
static uint32_t g_apbWriteData = 0;
static uint32_t g_apbReadData = 0;
static uint8_t g_apbPenable = 0;
static int32_t g_apbSlaveError = APB_PSLVERR_OK;
#endif

#ifdef PLATFORM_ESP32
/* ESP32 APB bus access */
static uint32_t g_apbAddress = 0;
static uint32_t g_apbWriteData = 0;
static uint32_t g_apbReadData = 0;
static uint8_t g_apbPenable = 0;
static int32_t g_apbSlaveError = APB_PSLVERR_OK;
#endif

#ifdef PLATFORM_LINUX
/* Linux simulation */
#define APB_SIMULATION_SIZE 4096
static uint32_t g_apbMemory[APB_SIMULATION_SIZE];
static uint32_t g_apbAddress = 0;
static uint32_t g_apbWriteData = 0;
static uint32_t g_apbReadData = 0;
static uint8_t g_apbPenable = 0;
static int32_t g_apbSlaveError = APB_PSLVERR_OK;
#endif

/******************************************************************************
 * @brief     : Enable APB bus clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    /* Enable APB1 and APB2 clocks - already enabled by HAL */
    __HAL_RCC_APB1_CLK_ENABLE();
    __HAL_RCC_APB2_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* Enable APB1 and APB2 clocks - already enabled by HAL */
    __HAL_RCC_APB1_CLK_ENABLE();
    __HAL_RCC_APB2_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 APB clock is enabled by default */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    if (memset_s(g_apbMemory, sizeof(g_apbMemory), 0, sizeof(g_apbMemory)) != EOK) {
        return -1;
    }
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure APB bus parameters
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbConfigureBus(void)
{
#ifdef PLATFORM_STM32F4
    /* APB1 and APB2 configuration (prescalers already set) */
    /* APB1: Max 42MHz, APB2: Max 84MHz on STM32F4 */
    g_apbAddress = NULL;
    g_apbPenable = 0;
    g_apbSlaveError = APB_PSLVERR_OK;
    return 0;
#elif defined(PLATFORM_STM32F1)
    /* APB1 and APB2 configuration (prescalers already set) */
    /* APB1: Max 36MHz, APB2: Max 72MHz on STM32F1 */
    g_apbAddress = NULL;
    g_apbPenable = 0;
    g_apbSlaveError = APB_PSLVERR_OK;
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 APB clock is 80MHz by default */
    g_apbAddress = 0;
    g_apbPenable = 0;
    g_apbSlaveError = APB_PSLVERR_OK;
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulation */
    g_apbAddress = 0;
    g_apbPenable = 0;
    g_apbSlaveError = APB_PSLVERR_OK;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbBusEnable(void)
{
#ifdef PLATFORM_STM32F4
    /* APB bus is always enabled on STM32 */
    g_apbPenable = 0;
#elif defined(PLATFORM_STM32F1)
    /* APB bus is always enabled on STM32 */
    g_apbPenable = 0;
#elif defined(PLATFORM_ESP32)
    /* APB bus is always enabled on ESP32 */
    g_apbPenable = 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulation */
    g_apbPenable = 0;
#endif
}

/******************************************************************************
 * @brief     : Disable APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbBusDisable(void)
{
#ifdef PLATFORM_STM32F4
    /* APB bus cannot be disabled on STM32 */
    g_apbPenable = 0;
#elif defined(PLATFORM_STM32F1)
    /* APB bus cannot be disabled on STM32 */
    g_apbPenable = 0;
#elif defined(PLATFORM_ESP32)
    /* APB bus cannot be disabled on ESP32 */
    g_apbPenable = 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulation */
    g_apbPenable = 0;
#endif
}

/******************************************************************************
 * @brief     : Check if APB bus is ready
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if ready, 0 if not ready
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbIsBusReady(void)
{
#ifdef PLATFORM_STM32F4
    /* APB bus is always ready for memory-mapped access */
    return 1;
#elif defined(PLATFORM_STM32F1)
    /* APB bus is always ready for memory-mapped access */
    return 1;
#elif defined(PLATFORM_ESP32)
    /* APB bus is always ready */
    return 1;
#elif defined(PLATFORM_LINUX)
    /* Linux simulation always ready */
    return 1;
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Check if APB transfer is complete
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if complete, 0 if not complete
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbIsTransferComplete(void)
{
#ifdef PLATFORM_STM32F4
    /* Memory-mapped access is synchronous */
    return (g_apbPenable == 1) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    /* Memory-mapped access is synchronous */
    return (g_apbPenable == 1) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    /* APB access is synchronous */
    return (g_apbPenable == 1) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    /* Linux simulation is synchronous */
    return (g_apbPenable == 1) ? 1 : 0;
#else
    return 1;
#endif
}

/******************************************************************************
 * @brief     : Set APB peripheral address (PADDR)
 * @param[in] : address --Peripheral address
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetAddress(uint32_t address)
{
#ifdef PLATFORM_STM32F4
    g_apbAddress = (volatile uint32_t*)address;
#elif defined(PLATFORM_STM32F1)
    g_apbAddress = (volatile uint32_t*)address;
#elif defined(PLATFORM_ESP32)
    g_apbAddress = address;
#elif defined(PLATFORM_LINUX)
    g_apbAddress = address;
#endif
}

/******************************************************************************
 * @brief     : Set APB protection signals (PPROT)
 * @param[in] : prot --Protection signals
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetProtection(uint8_t prot)
{
#ifdef PLATFORM_STM32F4
    /* Protection signals not used in memory-mapped access */
    (void)prot;
#elif defined(PLATFORM_STM32F1)
    /* Protection signals not used in memory-mapped access */
    (void)prot;
#elif defined(PLATFORM_ESP32)
    /* Protection signals not used */
    (void)prot;
#elif defined(PLATFORM_LINUX)
    /* Protection signals simulated but not enforced */
    (void)prot;
#endif
}

/******************************************************************************
 * @brief     : Set APB transfer type (read/write)
 * @param[in] : type --Transfer type
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetTransferType(ApbTransferType_E type)
{
#ifdef PLATFORM_STM32F4
    /* Transfer type handled by memory access operations */
    (void)type;
#elif defined(PLATFORM_STM32F1)
    /* Transfer type handled by memory access operations */
    (void)type;
#elif defined(PLATFORM_ESP32)
    /* Transfer type handled by register access */
    (void)type;
#elif defined(PLATFORM_LINUX)
    /* Transfer type simulated */
    (void)type;
#endif
}

/******************************************************************************
 * @brief     : Set APB write data (PWDATA)
 * @param[in] : data --Data to write
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbSetWriteData(uint32_t data)
{
#ifdef PLATFORM_STM32F4
    g_apbWriteData = data;
    if (g_apbAddress != NULL) {
        *g_apbAddress = data;
    }
#elif defined(PLATFORM_STM32F1)
    g_apbWriteData = data;
    if (g_apbAddress != NULL) {
        *g_apbAddress = data;
    }
#elif defined(PLATFORM_ESP32)
    g_apbWriteData = data;
    if (g_apbAddress != 0) {
        *((volatile uint32_t*)g_apbAddress) = data;
    }
#elif defined(PLATFORM_LINUX)
    g_apbWriteData = data;
    uint32_t offset = g_apbAddress / sizeof(uint32_t);
    if (offset < APB_SIMULATION_SIZE) {
        g_apbMemory[offset] = data;
    }
#endif
}

/******************************************************************************
 * @brief     : Get APB read data (PRDATA)
 * @param[in] : None
 * @param[out]: None
 * @return    : Read data value
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint32_t ApbGetReadData(void)
{
#ifdef PLATFORM_STM32F4
    if (g_apbAddress != NULL) {
        g_apbReadData = *g_apbAddress;
    }
    return g_apbReadData;
#elif defined(PLATFORM_STM32F1)
    if (g_apbAddress != NULL) {
        g_apbReadData = *g_apbAddress;
    }
    return g_apbReadData;
#elif defined(PLATFORM_ESP32)
    if (g_apbAddress != 0) {
        g_apbReadData = *((volatile uint32_t*)g_apbAddress);
    }
    return g_apbReadData;
#elif defined(PLATFORM_LINUX)
    uint32_t offset = g_apbAddress / sizeof(uint32_t);
    if (offset < APB_SIMULATION_SIZE) {
        g_apbReadData = g_apbMemory[offset];
    }
    return g_apbReadData;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Assert PENABLE signal
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbAssertPenable(void)
{
#ifdef PLATFORM_STM32F4
    g_apbPenable = 1;
#elif defined(PLATFORM_STM32F1)
    g_apbPenable = 1;
#elif defined(PLATFORM_ESP32)
    g_apbPenable = 1;
#elif defined(PLATFORM_LINUX)
    g_apbPenable = 1;
#endif
}

/******************************************************************************
 * @brief     : Deassert PENABLE signal
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbDeassertPenable(void)
{
#ifdef PLATFORM_STM32F4
    g_apbPenable = 0;
#elif defined(PLATFORM_STM32F1)
    g_apbPenable = 0;
#elif defined(PLATFORM_ESP32)
    g_apbPenable = 0;
#elif defined(PLATFORM_LINUX)
    g_apbPenable = 0;
#endif
}

/******************************************************************************
 * @brief     : Set APB byte strobe (PSTRB) for APB4
 * @param[in] : strobe --Byte strobe mask
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation, APB4 only
 *****************************************************************************/
void ApbSetStrobe(uint8_t strobe)
{
#ifdef PLATFORM_STM32F4
    /* Strobe not supported in basic memory-mapped access */
    (void)strobe;
#elif defined(PLATFORM_STM32F1)
    /* Strobe not supported in basic memory-mapped access */
    (void)strobe;
#elif defined(PLATFORM_ESP32)
    /* Strobe not supported */
    (void)strobe;
#elif defined(PLATFORM_LINUX)
    /* Strobe simulated but not enforced */
    (void)strobe;
#endif
}

/******************************************************************************
 * @brief     : Get APB slave error status (PSLVERR)
 * @param[in] : None
 * @param[out]: None
 * @return    : APB_PSLVERR_OK or APB_PSLVERR_ERROR
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t ApbGetSlaveError(void)
{
#ifdef PLATFORM_STM32F4
    /* Check for bus faults or errors */
    /* In real implementation, check fault status registers */
    return g_apbSlaveError;
#elif defined(PLATFORM_STM32F1)
    /* Check for bus faults or errors */
    /* In real implementation, check fault status registers */
    return g_apbSlaveError;
#elif defined(PLATFORM_ESP32)
    /* Check for access errors */
    return g_apbSlaveError;
#elif defined(PLATFORM_LINUX)
    /* Linux simulation returns OK */
    uint32_t offset = g_apbAddress / sizeof(uint32_t);
    if (offset >= APB_SIMULATION_SIZE) {
        return APB_PSLVERR_ERROR;
    }
    return g_apbSlaveError;
#else
    return APB_PSLVERR_OK;
#endif
}

/******************************************************************************
 * @brief     : Delay for specified microseconds
 * @param[in] : us --Microseconds to delay
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void ApbDelayUs(uint32_t us)
{
#ifdef PLATFORM_STM32F4
    /* Use DWT cycle counter for precise delay */
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTick) < delayTicks) {
    }
#elif defined(PLATFORM_STM32F1)
    /* Use DWT cycle counter for precise delay */
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTick) < delayTicks) {
    }
#elif defined(PLATFORM_ESP32)
    /* Use ESP32 microsecond delay */
    ets_delay_us(us);
#elif defined(PLATFORM_LINUX)
    /* Use Linux usleep */
    usleep(us);
#endif
}

#include "securec.h"
#include "swd_hal.h"
#include "platform_config.h"

/* Platform-specific includes and definitions */
#ifdef PLATFORM_STM32F4
/* SWCLK: PA14 (TCK), SWDIO: PA13 (TMS) */
#define SWD_GPIO_PORT      GPIOA
#define SWD_SWCLK_PIN      GPIO_PIN_14
#define SWD_SWDIO_PIN      GPIO_PIN_13
#endif

#ifdef PLATFORM_STM32F1
/* SWCLK: PA14 (TCK), SWDIO: PA13 (TMS) */
#define SWD_GPIO_PORT      GPIOA
#define SWD_SWCLK_PIN      GPIO_PIN_14
#define SWD_SWDIO_PIN      GPIO_PIN_13
#endif

#ifdef PLATFORM_ESP32
/* SWCLK: GPIO18, SWDIO: GPIO19 */
#include "driver/gpio.h"
#define SWD_SWCLK_GPIO     GPIO_NUM_18
#define SWD_SWDIO_GPIO     GPIO_NUM_19
#endif

#ifdef PLATFORM_LINUX
/* Using GPIO sysfs interface */
#include <stdlib.h>
#include <string.h>
static int g_swclkFd = -1;
static int g_swdioFd = -1;
static const char* g_swclkPath = "/sys/class/gpio/gpio17/value";
static const char* g_swdioPath = "/sys/class/gpio/gpio27/value";
static const char* g_swclkDirPath = "/sys/class/gpio/gpio17/direction";
static const char* g_swdioDirPath = "/sys/class/gpio/gpio27/direction";
#endif

/* Timing delay for bit-banging (platform-specific) */
static void SwdDelayHalfClock(void)
{
#ifdef PLATFORM_STM32F4
    /* Approx 1 MHz SWD clock (168 MHz CPU) */
    for (volatile int i = 0; i < 20; i++) { }
#elif defined(PLATFORM_STM32F1)
    /* Approx 1 MHz SWD clock (72 MHz CPU) */
    for (volatile int i = 0; i < 10; i++) { }
#elif defined(PLATFORM_ESP32)
    /* Approx 1 MHz SWD clock */
    ets_delay_us(0);
#elif defined(PLATFORM_LINUX)
    /* Use nanosleep for more accurate timing */
    struct timespec ts = {0, 500};  /* 500 ns */
    nanosleep(&ts, NULL);
#endif
}

/* Low-level GPIO operations */
static void SwdSetClockHigh(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWCLK_PIN, GPIO_PIN_SET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWCLK_PIN, GPIO_PIN_SET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(SWD_SWCLK_GPIO, 1);
#elif defined(PLATFORM_LINUX)
    if (g_swclkFd >= 0) {
        (void)write(g_swclkFd, "1", 1);
    }
#endif
}

static void SwdSetClockLow(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWCLK_PIN, GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWCLK_PIN, GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(SWD_SWCLK_GPIO, 0);
#elif defined(PLATFORM_LINUX)
    if (g_swclkFd >= 0) {
        (void)write(g_swclkFd, "0", 1);
    }
#endif
}

static void SwdSetDataHigh(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWDIO_PIN, GPIO_PIN_SET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWDIO_PIN, GPIO_PIN_SET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(SWD_SWDIO_GPIO, 1);
#elif defined(PLATFORM_LINUX)
    if (g_swdioFd >= 0) {
        (void)write(g_swdioFd, "1", 1);
    }
#endif
}

static void SwdSetDataLow(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWDIO_PIN, GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(SWD_GPIO_PORT, SWD_SWDIO_PIN, GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(SWD_SWDIO_GPIO, 0);
#elif defined(PLATFORM_LINUX)
    if (g_swdioFd >= 0) {
        (void)write(g_swdioFd, "0", 1);
    }
#endif
}

static uint8_t SwdReadData(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_GPIO_ReadPin(SWD_GPIO_PORT, SWD_SWDIO_PIN) == GPIO_PIN_SET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (HAL_GPIO_ReadPin(SWD_GPIO_PORT, SWD_SWDIO_PIN) == GPIO_PIN_SET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return gpio_get_level(SWD_SWDIO_GPIO);
#elif defined(PLATFORM_LINUX)
    if (g_swdioFd >= 0) {
        char value = '0';
        lseek(g_swdioFd, 0, SEEK_SET);
        if (read(g_swdioFd, &value, 1) < 0) {
            return 0;
        }
        return (value == '1') ? 1 : 0;
    }
    return 0;
#else
    return 0;
#endif
}

static void SwdSetDataOutput(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = SWD_SWDIO_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = SWD_SWDIO_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);
#elif defined(PLATFORM_ESP32)
    gpio_set_direction(SWD_SWDIO_GPIO, GPIO_MODE_OUTPUT);
#elif defined(PLATFORM_LINUX)
    if (g_swdioFd >= 0) {
        int dirFd = open(g_swdioDirPath, O_WRONLY);
        if (dirFd >= 0) {
            (void)write(dirFd, "out", 3);
            close(dirFd);
        }
    }
#endif
}

static void SwdSetDataInput(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = SWD_SWDIO_PIN;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};
    gpioInit.Pin = SWD_SWDIO_PIN;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);
#elif defined(PLATFORM_ESP32)
    gpio_set_direction(SWD_SWDIO_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWD_SWDIO_GPIO, GPIO_PULLUP_ONLY);
#elif defined(PLATFORM_LINUX)
    if (g_swdioFd >= 0) {
        int dirFd = open(g_swdioDirPath, O_WRONLY);
        if (dirFd >= 0) {
            (void)write(dirFd, "in", 2);
            close(dirFd);
        }
    }
#endif
}

/******************************************************************************
 * @brief     : Enable SWD clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SwdEnableClock(void)
{
#ifdef PLATFORM_STM32F4
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_STM32F1)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* ESP32 GPIO clock is always enabled */
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Linux doesn't require explicit clock enabling */
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Configure SWD GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t SwdConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* Configure SWCLK as output */
    gpioInit.Pin = SWD_SWCLK_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);

    /* Configure SWDIO as output initially */
    gpioInit.Pin = SWD_SWDIO_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);

    /* Set initial state */
    SwdSetClockLow();
    SwdSetDataLow();
    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* Configure SWCLK as output */
    gpioInit.Pin = SWD_SWCLK_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);

    /* Configure SWDIO as output initially */
    gpioInit.Pin = SWD_SWDIO_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_GPIO_PORT, &gpioInit);

    /* Set initial state */
    SwdSetClockLow();
    SwdSetDataLow();
    return 0;
#elif defined(PLATFORM_ESP32)
    /* Configure SWCLK */
    gpio_config_t ioConf = {0};
    ioConf.pin_bit_mask = (1ULL << SWD_SWCLK_GPIO);
    ioConf.mode = GPIO_MODE_OUTPUT;
    ioConf.pull_up_en = GPIO_PULLUP_DISABLE;
    ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&ioConf);

    /* Configure SWDIO */
    ioConf.pin_bit_mask = (1ULL << SWD_SWDIO_GPIO);
    ioConf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&ioConf);

    /* Set initial state */
    SwdSetClockLow();
    SwdSetDataLow();
    return 0;
#elif defined(PLATFORM_LINUX)
    /* Export GPIO pins */
    int exportFd = open("/sys/class/gpio/export", O_WRONLY);
    if (exportFd >= 0) {
        (void)write(exportFd, "17", 2);  /* SWCLK */
        (void)write(exportFd, "27", 2);  /* SWDIO */
        close(exportFd);
    }

    /* Wait for sysfs files to be created */
    usleep(100000);

    /* Open GPIO value files */
    g_swclkFd = open(g_swclkPath, O_RDWR);
    if (g_swclkFd < 0) {
        return -1;
    }
    g_swdioFd = open(g_swdioPath, O_RDWR);
    if (g_swdioFd < 0) {
        close(g_swclkFd);
        g_swclkFd = -1;
        return -1;
    }

    /* Set directions to output */
    int dirFd = open(g_swclkDirPath, O_WRONLY);
    if (dirFd >= 0) {
        (void)write(dirFd, "out", 3);
        close(dirFd);
    }

    dirFd = open(g_swdioDirPath, O_WRONLY);
    if (dirFd >= 0) {
        (void)write(dirFd, "out", 3);
        close(dirFd);
    }

    /* Set initial state */
    SwdSetClockLow();
    SwdSetDataLow();
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Release SWD GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void SwdReleaseGpio(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_DeInit(SWD_GPIO_PORT, SWD_SWCLK_PIN);
    HAL_GPIO_DeInit(SWD_GPIO_PORT, SWD_SWDIO_PIN);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_DeInit(SWD_GPIO_PORT, SWD_SWCLK_PIN);
    HAL_GPIO_DeInit(SWD_GPIO_PORT, SWD_SWDIO_PIN);
#elif defined(PLATFORM_ESP32)
    gpio_reset_pin(SWD_SWCLK_GPIO);
    gpio_reset_pin(SWD_SWDIO_GPIO);
#elif defined(PLATFORM_LINUX)
    if (g_swclkFd >= 0) {
        close(g_swclkFd);
        g_swclkFd = -1;
    }
    if (g_swdioFd >= 0) {
        close(g_swdioFd);
        g_swdioFd = -1;
    }

    /* Unexport GPIO pins */
    int unexportFd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (unexportFd >= 0) {
        (void)write(unexportFd, "17", 2);
        (void)write(unexportFd, "27", 2);
        close(unexportFd);
    }
#endif
}

/******************************************************************************
 * @brief     : Send JTAG-to-SWD switching sequence
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends 16-bit sequence 0xE79E to switch from JTAG to SWD
 *****************************************************************************/
int32_t SwdJtagToSwdSequence(void)
{
    const uint16_t sequence = 0xE79E;

    SwdSetDataOutput();

    /* Send 16-bit switching sequence LSB first */
    for (int32_t i = 0; i < 16; i++) {
        if (sequence & (1 << i)) {
            SwdSetDataHigh();
        } else {
            SwdSetDataLow();
        }
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        SwdSetClockLow();
    }

    return 0;
}

/******************************************************************************
 * @brief     : Perform SWD line reset
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Sends 50+ clock cycles with SWDIO high
 *****************************************************************************/
void SwdLineReset(void)
{
    SwdSetDataOutput();
    SwdSetDataHigh();

    /* Send at least 50 clock cycles with SWDIO high */
    for (int32_t i = 0; i < 60; i++) {
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        SwdSetClockLow();
    }

    SwdSetDataLow();
}

/******************************************************************************
 * @brief     : Send idle cycles
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Sends at least 8 clock cycles with SWDIO low
 *****************************************************************************/
void SwdIdle(void)
{
    SwdSetDataOutput();
    SwdSetDataLow();

    for (int32_t i = 0; i < 8; i++) {
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        SwdSetClockLow();
    }
}

/******************************************************************************
 * @brief     : Perform turnaround (change SWDIO direction)
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : One clock cycle with SWDIO as input
 *****************************************************************************/
void SwdTurnaround(void)
{
    static int32_t isOutput = 1;

    if (isOutput) {
        SwdSetDataInput();
        isOutput = 0;
    } else {
        SwdSetDataOutput();
        isOutput = 1;
    }

    /* One clock cycle for turnaround */
    SwdDelayHalfClock();
    SwdSetClockHigh();
    SwdDelayHalfClock();
    SwdSetClockLow();
}

/******************************************************************************
 * @brief     : Write SWD request header
 * @param[in] : request - 8-bit request value
 * @param[out]: None
 * @return    : None
 * @note      : Sends request LSB first
 *****************************************************************************/
void SwdWriteRequest(uint8_t request)
{
    SwdSetDataOutput();

    /* Send 8 bits LSB first */
    for (int32_t i = 0; i < 8; i++) {
        if (request & (1 << i)) {
            SwdSetDataHigh();
        } else {
            SwdSetDataLow();
        }
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        SwdSetClockLow();
    }
}

/******************************************************************************
 * @brief     : Read SWD ACK response
 * @param[in] : None
 * @param[out]: None
 * @return    : 3-bit ACK value
 * @note      : Reads ACK LSB first
 *****************************************************************************/
uint8_t SwdReadAck(void)
{
    uint8_t ack = 0;

    SwdSetDataInput();

    /* Read 3 bits LSB first */
    for (int32_t i = 0; i < 3; i++) {
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        if (SwdReadData()) {
            ack |= (1 << i);
        }
        SwdSetClockLow();
    }

    return ack;
}

/******************************************************************************
 * @brief     : Write 32-bit data
 * @param[in] : data - Data to write
 * @param[out]: None
 * @return    : None
 * @note      : Sends data LSB first
 *****************************************************************************/
void SwdWriteData(uint32_t data)
{
    SwdSetDataOutput();

    /* Send 32 bits LSB first */
    for (int32_t i = 0; i < 32; i++) {
        if (data & (1U << i)) {
            SwdSetDataHigh();
        } else {
            SwdSetDataLow();
        }
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        SwdSetClockLow();
    }
}

/******************************************************************************
 * @brief     : Read 32-bit data
 * @param[in] : None
 * @param[out]: None
 * @return    : Data read
 * @note      : Reads data LSB first
 *****************************************************************************/
uint32_t SwdReadData(void)
{
    uint32_t data = 0;

    SwdSetDataInput();

    /* Read 32 bits LSB first */
    for (int32_t i = 0; i < 32; i++) {
        SwdDelayHalfClock();
        SwdSetClockHigh();
        SwdDelayHalfClock();
        if (SwdReadData()) {
            data |= (1U << i);
        }
        SwdSetClockLow();
    }

    return data;
}

/******************************************************************************
 * @brief     : Write parity bit
 * @param[in] : parity - Parity bit value (0 or 1)
 * @param[out]: None
 * @return    : None
 * @note      : Sends single parity bit
 *****************************************************************************/
void SwdWriteParity(uint8_t parity)
{
    SwdSetDataOutput();

    if (parity) {
        SwdSetDataHigh();
    } else {
        SwdSetDataLow();
    }
    SwdDelayHalfClock();
    SwdSetClockHigh();
    SwdDelayHalfClock();
    SwdSetClockLow();
}

/******************************************************************************
 * @brief     : Read parity bit
 * @param[in] : None
 * @param[out]: None
 * @return    : Parity bit value (0 or 1)
 * @note      : Reads single parity bit
 *****************************************************************************/
uint8_t SwdReadParity(void)
{
    uint8_t parity = 0;

    SwdSetDataInput();

    SwdDelayHalfClock();
    SwdSetClockHigh();
    SwdDelayHalfClock();
    parity = SwdReadData();
    SwdSetClockLow();

    return parity;
}

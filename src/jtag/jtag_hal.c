#include "jtag_hal.h"
#include "platform_config.h"
#include "securec.h"

/* Platform-specific GPIO pin definitions */
#ifdef PLATFORM_STM32F4
/* STM32F4: Using GPIO pins for bit-banging */
#define JTAG_TCK_PIN GPIO_PIN_13
#define JTAG_TCK_PORT GPIOA
#define JTAG_TMS_PIN GPIO_PIN_14
#define JTAG_TMS_PORT GPIOA
#define JTAG_TDI_PIN GPIO_PIN_15
#define JTAG_TDI_PORT GPIOA
#define JTAG_TDO_PIN GPIO_PIN_12
#define JTAG_TDO_PORT GPIOA
#define JTAG_TRST_PIN GPIO_PIN_11
#define JTAG_TRST_PORT GPIOA

static uint32_t g_clockDelayUs = 1;
#endif

#ifdef PLATFORM_STM32F1
/* STM32F1: Using GPIO pins for bit-banging */
#define JTAG_TCK_PIN GPIO_PIN_13
#define JTAG_TCK_PORT GPIOA
#define JTAG_TMS_PIN GPIO_PIN_14
#define JTAG_TMS_PORT GPIOA
#define JTAG_TDI_PIN GPIO_PIN_15
#define JTAG_TDI_PORT GPIOA
#define JTAG_TDO_PIN GPIO_PIN_12
#define JTAG_TDO_PORT GPIOA
#define JTAG_TRST_PIN GPIO_PIN_11
#define JTAG_TRST_PORT GPIOA

static uint32_t g_clockDelayUs = 1;
#endif

#ifdef PLATFORM_ESP32
/* ESP32: Using GPIO pins for bit-banging */
#define JTAG_TCK_GPIO GPIO_NUM_25
#define JTAG_TMS_GPIO GPIO_NUM_26
#define JTAG_TDI_GPIO GPIO_NUM_27
#define JTAG_TDO_GPIO GPIO_NUM_14
#define JTAG_TRST_GPIO GPIO_NUM_13

#include "driver/gpio.h"
#include "esp_timer.h"

static uint32_t g_clockDelayUs = 1;
#endif

#ifdef PLATFORM_LINUX
/* Linux: Using sysfs GPIO or libftdi */
#include <sys/time.h>
#include <time.h>

#define JTAG_TCK_GPIO 23
#define JTAG_TMS_GPIO 24
#define JTAG_TDI_GPIO 25
#define JTAG_TDO_GPIO 8
#define JTAG_TRST_GPIO 7

static int g_tckFd = -1;
static int g_tmsFd = -1;
static int g_tdiFd = -1;
static int g_tdoFd = -1;
static int g_trstFd = -1;
static uint32_t g_clockDelayUs = 1;

/* Helper function to export GPIO */
static int32_t LinuxExportGpio(uint32_t gpio)
{
    int fd;
    char buf[64];

    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        return -1;
    }

    snprintf(buf, sizeof(buf), "%d", gpio);
    if (write(fd, buf, strlen(buf)) < 0) {
        close(fd);
        return -1;
    }
    close(fd);

    return 0;
}

/* Helper function to set GPIO direction */
static int32_t LinuxSetGpioDirection(uint32_t gpio, const char* direction)
{
    int fd;
    char path[64];

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        return -1;
    }

    if (write(fd, direction, strlen(direction)) < 0) {
        close(fd);
        return -1;
    }
    close(fd);

    return 0;
}

/* Helper function to open GPIO value file */
static int32_t LinuxOpenGpioValue(uint32_t gpio)
{
    char path[64];

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    return open(path, O_RDWR);
}
#endif

/******************************************************************************
 * @brief     : Configure JTAG GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t JtagConfigureGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure TCK, TMS, TDI as output */
    gpioInit.Pin = JTAG_TCK_PIN | JTAG_TMS_PIN | JTAG_TDI_PIN | JTAG_TRST_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure TDO as input */
    gpioInit.Pin = JTAG_TDO_PIN;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure TCK, TMS, TDI as output */
    gpioInit.Pin = JTAG_TCK_PIN | JTAG_TMS_PIN | JTAG_TDI_PIN | JTAG_TRST_PIN;
    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    /* Configure TDO as input */
    gpioInit.Pin = JTAG_TDO_PIN;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    return 0;
#elif defined(PLATFORM_ESP32)
    gpio_config_t ioConfig = {0};

    /* Configure TCK, TMS, TDI as output */
    ioConfig.pin_bit_mask = (1ULL << JTAG_TCK_GPIO) | (1ULL << JTAG_TMS_GPIO) | (1ULL << JTAG_TDI_GPIO) | (1ULL << JTAG_TRST_GPIO);
    ioConfig.mode = GPIO_MODE_OUTPUT;
    ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioConfig.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&ioConfig);

    /* Configure TDO as input */
    ioConfig.pin_bit_mask = (1ULL << JTAG_TDO_GPIO);
    ioConfig.mode = GPIO_MODE_INPUT;
    ioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&ioConfig);

    return 0;
#elif defined(PLATFORM_LINUX)
    /* Export GPIOs */
    LinuxExportGpio(JTAG_TCK_GPIO);
    LinuxExportGpio(JTAG_TMS_GPIO);
    LinuxExportGpio(JTAG_TDI_GPIO);
    LinuxExportGpio(JTAG_TDO_GPIO);
    LinuxExportGpio(JTAG_TRST_GPIO);

    usleep(100000); /* Wait for sysfs to be ready */

    /* Set directions */
    LinuxSetGpioDirection(JTAG_TCK_GPIO, "out");
    LinuxSetGpioDirection(JTAG_TMS_GPIO, "out");
    LinuxSetGpioDirection(JTAG_TDI_GPIO, "out");
    LinuxSetGpioDirection(JTAG_TDO_GPIO, "in");
    LinuxSetGpioDirection(JTAG_TRST_GPIO, "out");

    /* Open value files */
    g_tckFd = LinuxOpenGpioValue(JTAG_TCK_GPIO);
    if (g_tckFd < 0) {
        return -1;
    }
    g_tmsFd = LinuxOpenGpioValue(JTAG_TMS_GPIO);
    if (g_tmsFd < 0) {
        close(g_tckFd);
        g_tckFd = -1;
        return -1;
    }
    g_tdiFd = LinuxOpenGpioValue(JTAG_TDI_GPIO);
    if (g_tdiFd < 0) {
        close(g_tckFd);
        close(g_tmsFd);
        g_tckFd = g_tmsFd = -1;
        return -1;
    }
    g_tdoFd = LinuxOpenGpioValue(JTAG_TDO_GPIO);
    if (g_tdoFd < 0) {
        close(g_tckFd);
        close(g_tmsFd);
        close(g_tdiFd);
        g_tckFd = g_tmsFd = g_tdiFd = -1;
        return -1;
    }
    g_trstFd = LinuxOpenGpioValue(JTAG_TRST_GPIO);
    if (g_trstFd < 0) {
        close(g_tckFd);
        close(g_tmsFd);
        close(g_tdiFd);
        close(g_tdoFd);
        g_tckFd = g_tmsFd = g_tdiFd = g_tdoFd = -1;
        return -1;
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Release JTAG GPIO pins
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t JtagReleaseGpio(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_DeInit(GPIOA, JTAG_TCK_PIN | JTAG_TMS_PIN | JTAG_TDI_PIN | JTAG_TDO_PIN | JTAG_TRST_PIN);
    return 0;
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_DeInit(GPIOA, JTAG_TCK_PIN | JTAG_TMS_PIN | JTAG_TDI_PIN | JTAG_TDO_PIN | JTAG_TRST_PIN);
    return 0;
#elif defined(PLATFORM_ESP32)
    gpio_reset_pin(JTAG_TCK_GPIO);
    gpio_reset_pin(JTAG_TMS_GPIO);
    gpio_reset_pin(JTAG_TDI_GPIO);
    gpio_reset_pin(JTAG_TDO_GPIO);
    gpio_reset_pin(JTAG_TRST_GPIO);
    return 0;
#elif defined(PLATFORM_LINUX)
    if (g_tckFd >= 0) {
        close(g_tckFd);
    }
    if (g_tmsFd >= 0) {
        close(g_tmsFd);
    }
    if (g_tdiFd >= 0) {
        close(g_tdiFd);
    }
    if (g_tdoFd >= 0) {
        close(g_tdoFd);
    }
    if (g_trstFd >= 0) {
        close(g_trstFd);
    }

    g_tckFd = g_tmsFd = g_tdiFd = g_tdoFd = g_trstFd = -1;
    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : Enable JTAG clock
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t JtagEnableClock(void)
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
 * @brief     : Set TCK (Test Clock) pin state
 * @param[in] : state --Pin state (0=low, 1=high)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void JtagSetTck(uint8_t state)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(JTAG_TCK_PORT, JTAG_TCK_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(JTAG_TCK_PORT, JTAG_TCK_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(JTAG_TCK_GPIO, state);
#elif defined(PLATFORM_LINUX)
    if (g_tckFd >= 0) {
        char val = state ? '1' : '0';
        lseek(g_tckFd, 0, SEEK_SET);
        (void)write(g_tckFd, &val, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Set TMS (Test Mode Select) pin state
 * @param[in] : state --Pin state (0=low, 1=high)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void JtagSetTms(uint8_t state)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(JTAG_TMS_PORT, JTAG_TMS_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(JTAG_TMS_PORT, JTAG_TMS_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(JTAG_TMS_GPIO, state);
#elif defined(PLATFORM_LINUX)
    if (g_tmsFd >= 0) {
        char val = state ? '1' : '0';
        lseek(g_tmsFd, 0, SEEK_SET);
        (void)write(g_tmsFd, &val, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Set TDI (Test Data In) pin state
 * @param[in] : state --Pin state (0=low, 1=high)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void JtagSetTdi(uint8_t state)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(JTAG_TDI_PORT, JTAG_TDI_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(JTAG_TDI_PORT, JTAG_TDI_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(JTAG_TDI_GPIO, state);
#elif defined(PLATFORM_LINUX)
    if (g_tdiFd >= 0) {
        char val = state ? '1' : '0';
        lseek(g_tdiFd, 0, SEEK_SET);
        (void)write(g_tdiFd, &val, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Get TDO (Test Data Out) pin state
 * @param[in] : None
 * @param[out]: None
 * @return    : Pin state (0=low, 1=high)
 * @note      : Platform-specific implementation
 *****************************************************************************/
uint8_t JtagGetTdo(void)
{
#ifdef PLATFORM_STM32F4
    return (HAL_GPIO_ReadPin(JTAG_TDO_PORT, JTAG_TDO_PIN) == GPIO_PIN_SET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    return (HAL_GPIO_ReadPin(JTAG_TDO_PORT, JTAG_TDO_PIN) == GPIO_PIN_SET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    return gpio_get_level(JTAG_TDO_GPIO);
#elif defined(PLATFORM_LINUX)
    if (g_tdoFd >= 0) {
        char val = '0';
        lseek(g_tdoFd, 0, SEEK_SET);
        if (read(g_tdoFd, &val, 1) < 0) {
            return 0;
        }
        return (val == '1') ? 1 : 0;
    }
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : Set TRST (Test Reset) pin state
 * @param[in] : state --Pin state (0=low/reset, 1=high/inactive)
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation, optional pin
 *****************************************************************************/
void JtagSetTrst(uint8_t state)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(JTAG_TRST_PORT, JTAG_TRST_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(JTAG_TRST_PORT, JTAG_TRST_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(JTAG_TRST_GPIO, state);
#elif defined(PLATFORM_LINUX)
    if (g_trstFd >= 0) {
        char val = state ? '1' : '0';
        lseek(g_trstFd, 0, SEEK_SET);
        (void)write(g_trstFd, &val, 1);
    }
#endif
}

/******************************************************************************
 * @brief     : Microsecond delay for JTAG timing
 * @param[in] : us --Delay in microseconds
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *****************************************************************************/
void JtagDelayUs(uint32_t us)
{
#ifdef PLATFORM_STM32F4
    /* Simple delay loop for STM32F4 at 168MHz */
    volatile uint32_t count = us * 42; /* Approximate cycles */
    while (count--) {
        __NOP();
    }
#elif defined(PLATFORM_STM32F1)
    /* Simple delay loop for STM32F1 at 72MHz */
    volatile uint32_t count = us * 18; /* Approximate cycles */
    while (count--) {
        __NOP();
    }
#elif defined(PLATFORM_ESP32)
    esp_rom_delay_us(us);
#elif defined(PLATFORM_LINUX)
    usleep(us);
#endif
}

/******************************************************************************
 * @brief     : Configure JTAG clock speed
 * @param[in] : frequency --Clock frequency in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *****************************************************************************/
int32_t JtagConfigureClockSpeed(uint32_t frequency)
{
    if (frequency == 0) {
        return -1;
    }

    /* Calculate delay based on frequency */
    /* delay = 1 / (2 * frequency) converted to microseconds */
    if (frequency >= 1000000) {
        g_clockDelayUs = 1; /* Minimum 1us delay */
    } else {
        g_clockDelayUs = 500000 / frequency;
    }

    return 0;
}

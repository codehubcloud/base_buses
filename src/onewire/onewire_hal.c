#include "onewire_hal.h"
#include "platform_config.h"
#include "securec.h"


/* 1-Wire GPIO引脚定义 / 1-Wire GPIO pin definitions */
/* STM32: PA0, ESP32: GPIO15, Linux: GPIO4 (可配置) */
/* STM32: PA0, ESP32: GPIO15, Linux: GPIO4 (configurable) */

/* Platform-specific global variables */
#ifdef PLATFORM_STM32F4
static GPIO_TypeDef* g_onewirePort = GPIOA;
static uint16_t g_onewirePin = GPIO_PIN_0;
#endif

#ifdef PLATFORM_STM32F1
static GPIO_TypeDef* g_onewirePort = GPIOA;
static uint16_t g_onewirePin = GPIO_PIN_0;
#endif

#ifdef PLATFORM_ESP32
static const int g_onewireGpio = 15;
#endif

#ifdef PLATFORM_LINUX
static int g_onewireGpioNum = 4; /* 默认使用GPIO4 / Default use GPIO4 */
static int g_onewireExported = 0;
static char g_gpioPath[64];
static char g_gpioValuePath[64];
static char g_gpioDirectionPath[64];
#endif

/******************************************************************************
 * @brief     : 初始化1-Wire GPIO引脚 / Initialize 1-Wire GPIO pin
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Platform-specific implementation
 *              STM32: 配置PA0为开漏输出模式，带上拉
 *              STM32: Configure PA0 as open-drain output with pull-up
 *              ESP32: 配置GPIO15为输入输出模式
 *              ESP32: Configure GPIO15 as input/output mode
 *              Linux: 导出GPIO并配置为输出模式
 *              Linux: Export GPIO and configure as output mode
 *****************************************************************************/
int32_t OnewireInitGpio(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    /* 使能GPIOA时钟 / Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置PA0为开漏输出，带上拉，高速 */
    /* Configure PA0 as open-drain output with pull-up, high speed */
    gpioInit.Pin = g_onewirePin;
    gpioInit.Mode = GPIO_MODE_OUTPUT_OD; /* 开漏输出 / Open-drain output */
    gpioInit.Pull = GPIO_PULLUP;         /* 内部上拉 / Internal pull-up */
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_onewirePort, &gpioInit);

    /* 初始化为高电平（释放总线）/ Initialize to high level (release bus) */
    HAL_GPIO_WritePin(g_onewirePort, g_onewirePin, GPIO_PIN_SET);

    return 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    /* 使能GPIOA时钟 / Enable GPIOA clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置PA0为开漏输出，50MHz */
    /* Configure PA0 as open-drain output, 50MHz */
    gpioInit.Pin = g_onewirePin;
    gpioInit.Mode = GPIO_MODE_OUTPUT_OD; /* 开漏输出 / Open-drain output */
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_onewirePort, &gpioInit);

    /* 初始化为高电平（释放总线）/ Initialize to high level (release bus) */
    HAL_GPIO_WritePin(g_onewirePort, g_onewirePin, GPIO_PIN_SET);

    return 0;
#elif defined(PLATFORM_ESP32)
    gpio_config_t ioConfig = {0};

    /* 配置GPIO15为输入输出模式，带上拉 */
    /* Configure GPIO15 as input/output mode with pull-up */
    ioConfig.pin_bit_mask = (1ULL << g_onewireGpio);
    ioConfig.mode = GPIO_MODE_INPUT_OUTPUT_OD; /* 开漏模式 / Open-drain mode */
    ioConfig.pull_up_en = GPIO_PULLUP_ENABLE;  /* 使能上拉 / Enable pull-up */
    ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioConfig.intr_type = GPIO_INTR_DISABLE;

    if (gpio_config(&ioConfig) != ESP_OK) {
        return -1;
    }

    /* 初始化为高电平（释放总线）/ Initialize to high level (release bus) */
    gpio_set_level(g_onewireGpio, 1);

    return 0;
#elif defined(PLATFORM_LINUX)
    FILE* fp = NULL;
    char cmd[128];

    /* 导出GPIO / Export GPIO */
    fp = fopen("/sys/class/gpio/export", "w");
    if (fp == NULL) {
        return -1;
    }

    fprintf(fp, "%d", g_onewireGpioNum);
    fclose(fp);

    g_onewireExported = 1;

    /* 等待系统创建GPIO文件 / Wait for system to create GPIO files */
    usleep(100000); /* 100ms延时 / 100ms delay */

    /* 设置GPIO路径 / Set GPIO paths */
    snprintf_s(g_gpioPath, sizeof(g_gpioPath), sizeof(g_gpioPath) - 1, "/sys/class/gpio/gpio%d", g_onewireGpioNum);
    snprintf_s(g_gpioValuePath, sizeof(g_gpioValuePath), sizeof(g_gpioValuePath) - 1, "/sys/class/gpio/gpio%d/value", g_onewireGpioNum);
    snprintf_s(g_gpioDirectionPath, sizeof(g_gpioDirectionPath), sizeof(g_gpioDirectionPath) - 1, "/sys/class/gpio/gpio%d/direction",
               g_onewireGpioNum);

    /* 配置为输出模式 / Configure as output mode */
    fp = fopen(g_gpioDirectionPath, "w");
    if (fp == NULL) {
        return -1;
    }

    fprintf(fp, "out");
    fclose(fp);

    /* 初始化为高电平（释放总线）/ Initialize to high level (release bus) */
    fp = fopen(g_gpioValuePath, "w");
    if (fp != NULL) {
        fprintf(fp, "1");
        fclose(fp);
    }

    return 0;
#else
    return -1;
#endif
}

/******************************************************************************
 * @brief     : 设置1-Wire引脚为输出模式 / Set 1-Wire pin to output mode
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *              某些平台需要动态切换GPIO方向
 *              Some platforms need to dynamically switch GPIO direction
 *****************************************************************************/
void OnewireSetPinOutput(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    gpioInit.Pin = g_onewirePin;
    gpioInit.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_onewirePort, &gpioInit);
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    gpioInit.Pin = g_onewirePin;
    gpioInit.Mode = GPIO_MODE_OUTPUT_OD;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_onewirePort, &gpioInit);
#elif defined(PLATFORM_ESP32)
    gpio_set_direction(g_onewireGpio, GPIO_MODE_OUTPUT_OD);
#elif defined(PLATFORM_LINUX)
    FILE* fp = fopen(g_gpioDirectionPath, "w");
    if (fp != NULL) {
        fprintf(fp, "out");
        fclose(fp);
    }
#endif
}

/******************************************************************************
 * @brief     : 设置1-Wire引脚为输入模式 / Set 1-Wire pin to input mode
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *              读取总线状态时需要切换为输入模式
 *              Need to switch to input mode when reading bus state
 *****************************************************************************/
void OnewireSetPinInput(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_InitTypeDef gpioInit = {0};

    gpioInit.Pin = g_onewirePin;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_onewirePort, &gpioInit);
#elif defined(PLATFORM_STM32F1)
    GPIO_InitTypeDef gpioInit = {0};

    gpioInit.Pin = g_onewirePin;
    gpioInit.Mode = GPIO_MODE_INPUT;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(g_onewirePort, &gpioInit);
#elif defined(PLATFORM_ESP32)
    gpio_set_direction(g_onewireGpio, GPIO_MODE_INPUT);
#elif defined(PLATFORM_LINUX)
    FILE* fp = fopen(g_gpioDirectionPath, "w");
    if (fp != NULL) {
        fprintf(fp, "in");
        fclose(fp);
    }
#endif
}

/******************************************************************************
 * @brief     : 设置1-Wire引脚为高电平 / Set 1-Wire pin to high level
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *              在开漏模式下，写1实际上是释放总线
 *              In open-drain mode, writing 1 actually releases the bus
 *****************************************************************************/
void OnewireSetPinHigh(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(g_onewirePort, g_onewirePin, GPIO_PIN_SET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(g_onewirePort, g_onewirePin, GPIO_PIN_SET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(g_onewireGpio, 1);
#elif defined(PLATFORM_LINUX)
    FILE* fp = fopen(g_gpioValuePath, "w");
    if (fp != NULL) {
        fprintf(fp, "1");
        fclose(fp);
    }
#endif
}

/******************************************************************************
 * @brief     : 设置1-Wire引脚为低电平 / Set 1-Wire pin to low level
 * @param[in] : None
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *              拉低总线以发送复位信号或逻辑0
 *              Pull bus low to send reset signal or logic 0
 *****************************************************************************/
void OnewireSetPinLow(void)
{
#ifdef PLATFORM_STM32F4
    HAL_GPIO_WritePin(g_onewirePort, g_onewirePin, GPIO_PIN_RESET);
#elif defined(PLATFORM_STM32F1)
    HAL_GPIO_WritePin(g_onewirePort, g_onewirePin, GPIO_PIN_RESET);
#elif defined(PLATFORM_ESP32)
    gpio_set_level(g_onewireGpio, 0);
#elif defined(PLATFORM_LINUX)
    FILE* fp = fopen(g_gpioValuePath, "w");
    if (fp != NULL) {
        fprintf(fp, "0");
        fclose(fp);
    }
#endif
}

/******************************************************************************
 * @brief     : 读取1-Wire引脚电平 / Read 1-Wire pin level
 * @param[in] : None
 * @param[out]: None
 * @return    : 1 if high, 0 if low
 * @note      : Platform-specific implementation
 *              读取总线当前状态，用于检测存在脉冲和读取数据位
 *              Read current bus state for detecting presence pulse and reading data bits
 *****************************************************************************/
int32_t OnewireReadPin(void)
{
#ifdef PLATFORM_STM32F4
    GPIO_PinState pinState = HAL_GPIO_ReadPin(g_onewirePort, g_onewirePin);
    return (pinState == GPIO_PIN_SET) ? 1 : 0;
#elif defined(PLATFORM_STM32F1)
    GPIO_PinState pinState = HAL_GPIO_ReadPin(g_onewirePort, g_onewirePin);
    return (pinState == GPIO_PIN_SET) ? 1 : 0;
#elif defined(PLATFORM_ESP32)
    int level = gpio_get_level(g_onewireGpio);
    return (level == 1) ? 1 : 0;
#elif defined(PLATFORM_LINUX)
    FILE* fp = fopen(g_gpioValuePath, "r");
    if (fp == NULL) {
        return 0;
    }

    char value[4];
    if (fgets(value, sizeof(value), fp) != NULL) {
        fclose(fp);
        return (value[0] == '1') ? 1 : 0;
    }

    fclose(fp);
    return 0;
#else
    return 0;
#endif
}

/******************************************************************************
 * @brief     : 微秒级延时 / Microsecond delay
 * @param[in] : us - 延时时间（微秒）/ Delay time in microseconds
 * @param[out]: None
 * @return    : None
 * @note      : Platform-specific implementation
 *              1-Wire时序要求精确的微秒级延时
 *              1-Wire timing requires precise microsecond delays
 *              不同平台实现方式：
 *              Different platform implementations:
 *              - STM32: 使用DWT计数器或SysTick / Use DWT counter or SysTick
 *              - ESP32: 使用ets_delay_us() / Use ets_delay_us()
 *              - Linux: 使用usleep() / Use usleep()
 *****************************************************************************/
void OnewireDelayUs(uint32_t us)
{
#ifdef PLATFORM_STM32F4
    /* 使用DWT（Data Watchpoint and Trace）计数器实现微秒延时 */
    /* Use DWT (Data Watchpoint and Trace) counter for microsecond delay */
    uint32_t startTick = 0;
    uint32_t delayTicks = 0;

    /* 确保DWT已使能 / Ensure DWT is enabled */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    /* 计算延时所需的时钟周期数 / Calculate clock cycles for delay */
    /* 假设系统时钟为168MHz / Assume system clock is 168MHz */
    delayTicks = us * (SystemCoreClock / 1000000);

    startTick = DWT->CYCCNT;

    while ((DWT->CYCCNT - startTick) < delayTicks) {
        /* 等待延时完成 / Wait for delay to complete */
    }
#elif defined(PLATFORM_STM32F1)
    /* STM32F1使用简单的循环延时（需要根据实际时钟调整）*/
    /* STM32F1 uses simple loop delay (needs adjustment based on actual clock) */
    uint32_t startTick = 0;
    uint32_t delayTicks = 0;

    /* 使用SysTick实现微秒延时 / Use SysTick for microsecond delay */
    /* 假设系统时钟为72MHz / Assume system clock is 72MHz */
    delayTicks = us * (SystemCoreClock / 1000000);

    /* 如果DWT可用，使用DWT / Use DWT if available */
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    startTick = DWT->CYCCNT;

    while ((DWT->CYCCNT - startTick) < delayTicks) {
        /* 等待延时完成 / Wait for delay to complete */
    }
#elif defined(PLATFORM_ESP32)
    /* ESP32使用ROM函数ets_delay_us实现微秒延时 */
    /* ESP32 uses ROM function ets_delay_us for microsecond delay */
    ets_delay_us(us);
#elif defined(PLATFORM_LINUX)
    /* Linux使用usleep实现微秒延时 / Linux uses usleep for microsecond delay */
    usleep(us);
#else
    /* 默认的简单延时实现（不精确）/ Default simple delay (not accurate) */
    /* Default simple delay implementation (not accurate) */
    volatile uint32_t count = us * 10;
    while (count > 0) {
        count--;
    }
#endif
}

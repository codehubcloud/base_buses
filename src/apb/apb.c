#include <string.h>
#include "apb.h"
#include "apb_hal.h"
#include "securec.h"

/* Global APB state management */
static ApbState_E g_apbState = APB_STATE_IDLE;
static uint32_t g_apbTimeoutUs = APB_DEFAULT_TIMEOUT_US;
static int32_t g_lastError = APB_PSLVERR_OK;

/******************************************************************************
 * @brief     : Initialize APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Initializes APB bus state machine and timeout
 *****************************************************************************/
int32_t ApbInit(void)
{
    int32_t result = 0;

    result = ApbEnableClock();
    if (result != 0) {
        return -1;
    }

    result = ApbConfigureBus();
    if (result != 0) {
        return -1;
    }

    g_apbState = APB_STATE_IDLE;
    g_apbTimeoutUs = APB_DEFAULT_TIMEOUT_US;
    g_lastError = APB_PSLVERR_OK;

    ApbBusEnable();

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Resets APB bus to idle state
 *****************************************************************************/
int32_t ApbDeinit(void)
{
    ApbBusDisable();
    g_apbState = APB_STATE_IDLE;
    g_lastError = APB_PSLVERR_OK;

    return 0;
}

/******************************************************************************
 * @brief     : Perform single write transaction on APB bus
 * @param[in] : address --Peripheral address (PADDR)
 * @param[in] : data --Data to write (PWDATA)
 * @param[in] : prot --Protection signals (PPROT)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Executes SETUP and ACCESS phases for write
 *****************************************************************************/
int32_t ApbWrite(uint32_t address, uint32_t data, uint8_t prot)
{
    uint32_t timeout = g_apbTimeoutUs;

    if (g_apbState != APB_STATE_IDLE) {
        return -1;
    }

    /* SETUP phase */
    g_apbState = APB_STATE_SETUP;
    ApbSetAddress(address);
    ApbSetProtection(prot);
    ApbSetTransferType(APB_TRANSFER_WRITE);

    /* Wait for bus ready */
    while (ApbIsBusReady() == 0) {
        if (timeout == 0) {
            g_apbState = APB_STATE_IDLE;
            g_lastError = APB_PSLVERR_ERROR;
            return -1;
        }
        timeout--;
        ApbDelayUs(1);
    }

    /* ACCESS phase */
    g_apbState = APB_STATE_ACCESS;
    ApbSetWriteData(data);
    ApbAssertPenable();

    /* Wait for transfer complete */
    timeout = g_apbTimeoutUs;
    while (ApbIsTransferComplete() == 0) {
        if (timeout == 0) {
            ApbDeassertPenable();
            g_apbState = APB_STATE_IDLE;
            g_lastError = APB_PSLVERR_ERROR;
            return -1;
        }
        timeout--;
        ApbDelayUs(1);
    }

    /* Check for slave error */
    g_lastError = ApbGetSlaveError();
    ApbDeassertPenable();
    g_apbState = APB_STATE_IDLE;

    return (g_lastError == APB_PSLVERR_OK) ? 0 : -1;
}

/******************************************************************************
 * @brief     : Perform single read transaction on APB bus
 * @param[in] : address --Peripheral address (PADDR)
 * @param[in] : prot --Protection signals (PPROT)
 * @param[out]: data --Pointer to receive data (PRDATA)
 * @return    : 0 if success, -1 if error
 * @note      : Executes SETUP and ACCESS phases for read
 *****************************************************************************/
int32_t ApbRead(uint32_t address, uint32_t* data, uint8_t prot)
{
    uint32_t timeout = g_apbTimeoutUs;

    if ((data == NULL) || (g_apbState != APB_STATE_IDLE)) {
        return -1;
    }

    /* SETUP phase */
    g_apbState = APB_STATE_SETUP;
    ApbSetAddress(address);
    ApbSetProtection(prot);
    ApbSetTransferType(APB_TRANSFER_READ);

    /* Wait for bus ready */
    while (ApbIsBusReady() == 0) {
        if (timeout == 0) {
            g_apbState = APB_STATE_IDLE;
            g_lastError = APB_PSLVERR_ERROR;
            return -1;
        }
        timeout--;
        ApbDelayUs(1);
    }

    /* ACCESS phase */
    g_apbState = APB_STATE_ACCESS;
    ApbAssertPenable();

    /* Wait for transfer complete */
    timeout = g_apbTimeoutUs;
    while (ApbIsTransferComplete() == 0) {
        if (timeout == 0) {
            ApbDeassertPenable();
            g_apbState = APB_STATE_IDLE;
            g_lastError = APB_PSLVERR_ERROR;
            return -1;
        }
        timeout--;
        ApbDelayUs(1);
    }

    /* Check for slave error and read data */
    g_lastError = ApbGetSlaveError();
    if (g_lastError == APB_PSLVERR_OK) {
        *data = ApbGetReadData();
    }

    ApbDeassertPenable();
    g_apbState = APB_STATE_IDLE;

    return (g_lastError == APB_PSLVERR_OK) ? 0 : -1;
}

/******************************************************************************
 * @brief     : Perform write transaction with byte strobe (APB4 feature)
 * @param[in] : address --Peripheral address (PADDR)
 * @param[in] : data --Data to write (PWDATA)
 * @param[in] : strobe --Byte strobe mask (PSTRB), 1 bit per byte
 * @param[in] : prot --Protection signals (PPROT)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : APB4 only, allows partial word writes
 *****************************************************************************/
int32_t ApbWriteWithStrobe(uint32_t address, uint32_t data, uint8_t strobe, uint8_t prot)
{
    uint32_t timeout = g_apbTimeoutUs;

    if (g_apbState != APB_STATE_IDLE) {
        return -1;
    }

    /* SETUP phase */
    g_apbState = APB_STATE_SETUP;
    ApbSetAddress(address);
    ApbSetProtection(prot);
    ApbSetTransferType(APB_TRANSFER_WRITE);
    ApbSetStrobe(strobe);

    /* Wait for bus ready */
    while (ApbIsBusReady() == 0) {
        if (timeout == 0) {
            g_apbState = APB_STATE_IDLE;
            g_lastError = APB_PSLVERR_ERROR;
            return -1;
        }
        timeout--;
        ApbDelayUs(1);
    }

    /* ACCESS phase */
    g_apbState = APB_STATE_ACCESS;
    ApbSetWriteData(data);
    ApbAssertPenable();

    /* Wait for transfer complete */
    timeout = g_apbTimeoutUs;
    while (ApbIsTransferComplete() == 0) {
        if (timeout == 0) {
            ApbDeassertPenable();
            g_apbState = APB_STATE_IDLE;
            g_lastError = APB_PSLVERR_ERROR;
            return -1;
        }
        timeout--;
        ApbDelayUs(1);
    }

    /* Check for slave error */
    g_lastError = ApbGetSlaveError();
    ApbDeassertPenable();
    g_apbState = APB_STATE_IDLE;

    return (g_lastError == APB_PSLVERR_OK) ? 0 : -1;
}

/******************************************************************************
 * @brief     : Perform multiple consecutive read transactions
 * @param[in] : startAddress --Starting peripheral address
 * @param[in] : count --Number of words to read
 * @param[in] : prot --Protection signals (PPROT)
 * @param[out]: dataBuffer --Pointer to buffer for received data
 * @return    : Number of words read, -1 if error
 * @note      : Optimized for burst reads from consecutive addresses
 *****************************************************************************/
int32_t ApbReadMultiple(uint32_t startAddress, uint32_t* dataBuffer, uint32_t count, uint8_t prot)
{
    uint32_t i = 0;
    int32_t result = 0;

    if ((dataBuffer == NULL) || (count == 0)) {
        return -1;
    }

    for (i = 0; i < count; i++) {
        result = ApbRead(startAddress + (i * 4), &dataBuffer[i], prot);
        if (result != 0) {
            return (int32_t)i;
        }
    }

    return (int32_t)count;
}

/******************************************************************************
 * @brief     : Perform multiple consecutive write transactions
 * @param[in] : startAddress --Starting peripheral address
 * @param[in] : dataBuffer --Pointer to data buffer to write
 * @param[in] : count --Number of words to write
 * @param[in] : prot --Protection signals (PPROT)
 * @param[out]: None
 * @return    : Number of words written, -1 if error
 * @note      : Optimized for burst writes to consecutive addresses
 *****************************************************************************/
int32_t ApbWriteMultiple(uint32_t startAddress, uint32_t* dataBuffer, uint32_t count, uint8_t prot)
{
    uint32_t i = 0;
    int32_t result = 0;

    if ((dataBuffer == NULL) || (count == 0)) {
        return -1;
    }

    for (i = 0; i < count; i++) {
        result = ApbWrite(startAddress + (i * 4), dataBuffer[i], prot);
        if (result != 0) {
            return (int32_t)i;
        }
    }

    return (int32_t)count;
}

/******************************************************************************
 * @brief     : Configure transaction timeout
 * @param[in] : timeoutUs --Timeout in microseconds
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sets maximum wait time for bus ready state
 *****************************************************************************/
int32_t ApbSetTimeout(uint32_t timeoutUs)
{
    if (timeoutUs == 0) {
        return -1;
    }

    g_apbTimeoutUs = timeoutUs;
    return 0;
}

/******************************************************************************
 * @brief     : Get last error status from APB transaction
 * @param[in] : None
 * @param[out]: None
 * @return    : APB_PSLVERR_OK or APB_PSLVERR_ERROR
 * @note      : Checks PSLVERR signal from last transaction
 *****************************************************************************/
int32_t ApbGetError(void)
{
    return g_lastError;
}

/******************************************************************************
 * @brief     : Get current APB bus state
 * @param[in] : None
 * @param[out]: None
 * @return    : Current APB state (IDLE, SETUP, ACCESS)
 * @note      : Returns current state machine state
 *****************************************************************************/
ApbState_E ApbGetState(void)
{
    return g_apbState;
}

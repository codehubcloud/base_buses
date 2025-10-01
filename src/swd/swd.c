#include <string.h>
#include "securec.h"
#include "swd.h"
#include "swd_hal.h"

/* Internal helper function prototypes */
static uint8_t SwdCalculateParity(uint32_t value);
static int32_t SwdTransaction(uint8_t request, uint32_t* data, int32_t isRead);

/******************************************************************************
 * @brief     : Calculate even parity bit for 32-bit value
 * @param[in] : value - Input value
 * @param[out]: None
 * @return    : Parity bit (0 or 1)
 * @note      : Counts number of set bits and returns LSB of count
 *****************************************************************************/
static uint8_t SwdCalculateParity(uint32_t value)
{
    uint8_t parity = 0;
    for (int32_t i = 0; i < 32; i++) {
        if (value & (1U << i)) {
            parity ^= 1;
        }
    }
    return parity;
}

/******************************************************************************
 * @brief     : Perform low-level SWD transaction
 * @param[in] : request - 8-bit request header, isRead - 1 for read, 0 for write
 * @param[out]: data - Pointer to data (read or write)
 * @return    : ACK value (SWD_ACK_OK/WAIT/FAULT), or -1 on error
 * @note      : Implements SWD protocol: request phase, ACK, data phase
 *****************************************************************************/
static int32_t SwdTransaction(uint8_t request, uint32_t* data, int32_t isRead)
{
    uint8_t ack = 0;
    uint32_t dataValue = 0;
    uint8_t parity = 0;

    if (data == NULL) {
        return -1;
    }

    /* Send request header (8 bits) */
    SwdWriteRequest(request);

    /* Turnaround: 1 clock cycle */
    SwdTurnaround();

    /* Read ACK (3 bits) */
    ack = SwdReadAck();

    if (ack != SWD_ACK_OK) {
        SwdTurnaround();
        return (int32_t)ack;
    }

    if (isRead) {
        /* Read data (32 bits) + parity (1 bit) */
        dataValue = SwdReadData();
        parity = SwdReadParity();

        /* Verify parity */
        if (parity != SwdCalculateParity(dataValue)) {
            SwdTurnaround();
            return -1;
        }

        *data = dataValue;
        SwdTurnaround();
    } else {
        /* Turnaround before write */
        SwdTurnaround();

        /* Write data (32 bits) + parity (1 bit) */
        dataValue = *data;
        parity = SwdCalculateParity(dataValue);
        SwdWriteData(dataValue);
        SwdWriteParity(parity);
    }

    /* Idle cycles */
    SwdIdle();

    return SWD_ACK_OK;
}

/******************************************************************************
 * @brief     : Initialize SWD interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures GPIO pins and sends JTAG-to-SWD sequence
 *****************************************************************************/
int32_t SwdInit(void)
{
    int32_t result = 0;

    result = SwdEnableClock();
    if (result != 0) {
        return -1;
    }

    result = SwdConfigureGpio();
    if (result != 0) {
        return -1;
    }

    /* Send JTAG-to-SWD switching sequence */
    result = SwdJtagToSwdSequence();
    if (result != 0) {
        return -1;
    }

    /* Perform line reset */
    result = SwdReset();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize SWD interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Releases GPIO pins and resets state
 *****************************************************************************/
int32_t SwdDeinit(void)
{
    SwdReleaseGpio();
    return 0;
}

/******************************************************************************
 * @brief     : Perform SWD line reset sequence
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends 50+ clock cycles with SWDIO high followed by idle
 *****************************************************************************/
int32_t SwdReset(void)
{
    SwdLineReset();
    SwdIdle();
    return 0;
}

/******************************************************************************
 * @brief     : Read Debug Port register
 * @param[in] : address - DP register address (aligned to 4 bytes)
 * @param[out]: data - Pointer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Performs SWD read transaction with ACK checking
 *****************************************************************************/
int32_t SwdReadDP(uint8_t address, uint32_t* data)
{
    uint8_t request = 0;
    int32_t ack = 0;
    int32_t retryCount = 0;

    if (data == NULL) {
        return -1;
    }

    /* Build request: Start(1) + AP/DP(0) + R/W(1) + A[2:3] + Parity + Stop(0) + Park(1) */
    request = SWD_REQ_PARK_START;
    request |= SWD_REQ_READ_WRITE;           /* Read operation */
    request |= ((address >> 2) & 0x03) << 3; /* Address bits A[2:3] */

    /* Calculate parity for request */
    uint8_t parityBits = ((request >> 1) & 0x0F); /* AP/DP + R/W + A[2:3] */
    uint8_t reqParity = 0;
    for (int32_t i = 0; i < 4; i++) {
        if (parityBits & (1 << i)) {
            reqParity ^= 1;
        }
    }
    request |= (reqParity << 5);

    /* Perform transaction with retry on WAIT */
    for (retryCount = 0; retryCount < SWD_RETRY_COUNT; retryCount++) {
        ack = SwdTransaction(request, data, 1);
        if (ack == SWD_ACK_OK) {
            return 0;
        }
        if (ack != SWD_ACK_WAIT) {
            return -1;
        }
    }

    return -1;
}

/******************************************************************************
 * @brief     : Write Debug Port register
 * @param[in] : address - DP register address (aligned to 4 bytes), data - Data to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Performs SWD write transaction with ACK checking
 *****************************************************************************/
int32_t SwdWriteDP(uint8_t address, uint32_t data)
{
    uint8_t request = 0;
    int32_t ack = 0;
    int32_t retryCount = 0;
    uint32_t writeData = data;

    /* Build request: Start(1) + AP/DP(0) + R/W(0) + A[2:3] + Parity + Stop(0) + Park(1) */
    request = SWD_REQ_PARK_START;
    /* R/W bit is 0 for write */
    request |= ((address >> 2) & 0x03) << 3; /* Address bits A[2:3] */

    /* Calculate parity for request */
    uint8_t parityBits = ((request >> 1) & 0x0F); /* AP/DP + R/W + A[2:3] */
    uint8_t reqParity = 0;
    for (int32_t i = 0; i < 4; i++) {
        if (parityBits & (1 << i)) {
            reqParity ^= 1;
        }
    }
    request |= (reqParity << 5);

    /* Perform transaction with retry on WAIT */
    for (retryCount = 0; retryCount < SWD_RETRY_COUNT; retryCount++) {
        ack = SwdTransaction(request, &writeData, 0);
        if (ack == SWD_ACK_OK) {
            return 0;
        }
        if (ack != SWD_ACK_WAIT) {
            return -1;
        }
    }

    return -1;
}

/******************************************************************************
 * @brief     : Read Access Port register
 * @param[in] : address - AP register address (aligned to 4 bytes)
 * @param[out]: data - Pointer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Requires prior AP selection via DP SELECT register
 *****************************************************************************/
int32_t SwdReadAP(uint8_t address, uint32_t* data)
{
    uint8_t request = 0;
    int32_t ack = 0;
    int32_t retryCount = 0;
    uint32_t dummyData = 0;

    if (data == NULL) {
        return -1;
    }

    /* Build request: Start(1) + AP/DP(1) + R/W(1) + A[2:3] + Parity + Stop(0) + Park(1) */
    request = SWD_REQ_PARK_START;
    request |= SWD_REQ_AP_DP;                /* AP access */
    request |= SWD_REQ_READ_WRITE;           /* Read operation */
    request |= ((address >> 2) & 0x03) << 3; /* Address bits A[2:3] */

    /* Calculate parity for request */
    uint8_t parityBits = ((request >> 1) & 0x0F); /* AP/DP + R/W + A[2:3] */
    uint8_t reqParity = 0;
    for (int32_t i = 0; i < 4; i++) {
        if (parityBits & (1 << i)) {
            reqParity ^= 1;
        }
    }
    request |= (reqParity << 5);

    /* Perform transaction with retry on WAIT */
    for (retryCount = 0; retryCount < SWD_RETRY_COUNT; retryCount++) {
        ack = SwdTransaction(request, &dummyData, 1);
        if (ack == SWD_ACK_OK) {
            break;
        }
        if (ack != SWD_ACK_WAIT) {
            return -1;
        }
    }

    if (retryCount >= SWD_RETRY_COUNT) {
        return -1;
    }

    /* Read RDBUFF to get actual data */
    return SwdReadDP(SWD_DP_RDBUFF, data);
}

/******************************************************************************
 * @brief     : Write Access Port register
 * @param[in] : address - AP register address (aligned to 4 bytes), data - Data to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Requires prior AP selection via DP SELECT register
 *****************************************************************************/
int32_t SwdWriteAP(uint8_t address, uint32_t data)
{
    uint8_t request = 0;
    int32_t ack = 0;
    int32_t retryCount = 0;
    uint32_t writeData = data;

    /* Build request: Start(1) + AP/DP(1) + R/W(0) + A[2:3] + Parity + Stop(0) + Park(1) */
    request = SWD_REQ_PARK_START;
    request |= SWD_REQ_AP_DP; /* AP access */
    /* R/W bit is 0 for write */
    request |= ((address >> 2) & 0x03) << 3; /* Address bits A[2:3] */

    /* Calculate parity for request */
    uint8_t parityBits = ((request >> 1) & 0x0F); /* AP/DP + R/W + A[2:3] */
    uint8_t reqParity = 0;
    for (int32_t i = 0; i < 4; i++) {
        if (parityBits & (1 << i)) {
            reqParity ^= 1;
        }
    }
    request |= (reqParity << 5);

    /* Perform transaction with retry on WAIT */
    for (retryCount = 0; retryCount < SWD_RETRY_COUNT; retryCount++) {
        ack = SwdTransaction(request, &writeData, 0);
        if (ack == SWD_ACK_OK) {
            return 0;
        }
        if (ack != SWD_ACK_WAIT) {
            return -1;
        }
    }

    return -1;
}

/******************************************************************************
 * @brief     : Read target device IDCODE
 * @param[in] : None
 * @param[out]: idcode - Pointer to store IDCODE value
 * @return    : 0 if success, -1 if error
 * @note      : Reads IDCODE from DP, verifies SWD connection
 *****************************************************************************/
int32_t SwdReadIdcode(uint32_t* idcode)
{
    if (idcode == NULL) {
        return -1;
    }

    /* Read IDCODE from DP register 0x00 */
    return SwdReadDP(SWD_DP_IDCODE, idcode);
}

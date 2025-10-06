#include <string.h>
#include "ahb.h"
#include "ahb_hal.h"
#include "securec.h"


/******************************************************************************
 * @brief     : Initialize AHB bus controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures AHB bus and enables master interface
 *****************************************************************************/
int32_t AhbInit(void)
{
    int32_t result = 0;

    result = AhbEnableClock();
    if (result != 0) {
        return -1;
    }

    result = AhbConfigureBusMatrix();
    if (result != 0) {
        return -1;
    }

    AhbResetController();
    AhbEnableMaster();

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize AHB bus controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables AHB bus and resets configuration
 *****************************************************************************/
int32_t AhbDeinit(void)
{
    AhbDisableMaster();
    AhbResetController();
    AhbDisableClock();

    return 0;
}

/******************************************************************************
 * @brief     : Perform single AHB write transaction
 * @param[in] : address --Target address, data --Data to write, size --Transfer size
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Executes NONSEQ transfer with OKAY response check
 *****************************************************************************/
int32_t AhbWrite(uint32_t address, uint32_t data, uint8_t size)
{
    int32_t result = 0;

    if (size > AHB_SIZE_1024BIT) {
        return -1;
    }

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    AhbSetAddress(address);
    AhbSetTransferType(AHB_TRANS_NONSEQ);
    AhbSetBurstType(AHB_BURST_SINGLE);
    AhbSetTransferSize(size);
    AhbSetWrite(1);

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    AhbWriteData(data);

    if (AhbGetResponse() != AHB_RESP_OKAY) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Perform single AHB read transaction
 * @param[in] : address --Target address, size --Transfer size
 * @param[out]: data --Pointer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Executes NONSEQ transfer with OKAY response check
 *****************************************************************************/
int32_t AhbRead(uint32_t address, uint32_t* data, uint8_t size)
{
    int32_t result = 0;

    if ((data == NULL) || (size > AHB_SIZE_1024BIT)) {
        return -1;
    }

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    AhbSetAddress(address);
    AhbSetTransferType(AHB_TRANS_NONSEQ);
    AhbSetBurstType(AHB_BURST_SINGLE);
    AhbSetTransferSize(size);
    AhbSetWrite(0);

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    *data = AhbReadData();

    if (AhbGetResponse() != AHB_RESP_OKAY) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Perform AHB burst write transaction
 * @param[in] : address --Start address, data --Data buffer, length --Number of transfers, burstType --Burst type
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Supports INCR, INCR4/8/16, WRAP4/8/16 burst types
 *****************************************************************************/
int32_t AhbBurstWrite(uint32_t address, uint32_t* data, uint16_t length, uint8_t burstType)
{
    int32_t result = 0;
    uint32_t currentAddress = address;
    uint16_t addressIncrement = 4;

    if ((data == NULL) || (length == 0) || (length > AHB_MAX_BURST_LENGTH)) {
        return -1;
    }

    if (burstType > AHB_BURST_INCR16) {
        return -1;
    }

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        if (i == 0) {
            AhbSetTransferType(AHB_TRANS_NONSEQ);
        } else {
            AhbSetTransferType(AHB_TRANS_SEQ);
        }

        AhbSetAddress(currentAddress);
        AhbSetBurstType(burstType);
        AhbSetTransferSize(AHB_SIZE_WORD);
        AhbSetWrite(1);

        result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
        if (result != 0) {
            return -1;
        }

        AhbWriteData(data[i]);

        if (AhbGetResponse() != AHB_RESP_OKAY) {
            return -1;
        }

        currentAddress += addressIncrement;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Perform AHB burst read transaction
 * @param[in] : address --Start address, length --Number of transfers, burstType --Burst type
 * @param[out]: data --Buffer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Supports INCR, INCR4/8/16, WRAP4/8/16 burst types
 *****************************************************************************/
int32_t AhbBurstRead(uint32_t address, uint32_t* data, uint16_t length, uint8_t burstType)
{
    int32_t result = 0;
    uint32_t currentAddress = address;
    uint16_t addressIncrement = 4;

    if ((data == NULL) || (length == 0) || (length > AHB_MAX_BURST_LENGTH)) {
        return -1;
    }

    if (burstType > AHB_BURST_INCR16) {
        return -1;
    }

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    for (uint16_t i = 0; i < length; i++) {
        if (i == 0) {
            AhbSetTransferType(AHB_TRANS_NONSEQ);
        } else {
            AhbSetTransferType(AHB_TRANS_SEQ);
        }

        AhbSetAddress(currentAddress);
        AhbSetBurstType(burstType);
        AhbSetTransferSize(AHB_SIZE_WORD);
        AhbSetWrite(0);

        result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
        if (result != 0) {
            return -1;
        }

        data[i] = AhbReadData();

        if (AhbGetResponse() != AHB_RESP_OKAY) {
            return -1;
        }

        currentAddress += addressIncrement;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set AHB master priority level
 * @param[in] : priority --Priority level (0 = highest)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures bus arbitration priority for this master
 *****************************************************************************/
int32_t AhbSetPriority(uint8_t priority)
{
    if (AhbConfigurePriority(priority) != 0) {
        return -1;
    }
    return 0;
}

/******************************************************************************
 * @brief     : Get current AHB bus status
 * @param[in] : None
 * @param[out]: None
 * @return    : AHB_STATUS_IDLE, AHB_STATUS_BUSY, or AHB_STATUS_ERROR
 * @note      : Checks HREADY and HRESP signals
 *****************************************************************************/
int32_t AhbGetBusStatus(void)
{
    if (AhbGetResponse() == AHB_RESP_ERROR) {
        return AHB_STATUS_ERROR;
    }

    if (AhbCheckReady() == 0) {
        return AHB_STATUS_BUSY;
    }

    return AHB_STATUS_IDLE;
}

/******************************************************************************
 * @brief     : Wait for AHB bus ready signal
 * @param[in] : timeout --Maximum cycles to wait (0 = infinite)
 * @param[out]: None
 * @return    : 0 if ready, -1 if timeout or error
 * @note      : Polls HREADY signal until high or timeout
 *****************************************************************************/
int32_t AhbWaitReady(uint32_t timeout)
{
    uint32_t cycles = 0;

    while (AhbCheckReady() == 0) {
        if ((timeout > 0) && (cycles >= timeout)) {
            return -1;
        }
        cycles++;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Execute custom AHB transaction
 * @param[in] : transaction --Pointer to AHB transaction structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Allows full control over all AHB signals
 *****************************************************************************/
int32_t AhbExecuteTransaction(AhbTransaction* transaction)
{
    int32_t result = 0;
    uint32_t currentAddress = 0;

    if ((transaction == NULL) || (transaction->data == NULL)) {
        return -1;
    }

    if (transaction->length == 0) {
        return -1;
    }

    currentAddress = transaction->address;

    result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
    if (result != 0) {
        return -1;
    }

    for (uint16_t i = 0; i < transaction->length; i++) {
        AhbSetAddress(currentAddress);
        AhbSetTransferType(transaction->transferType);
        AhbSetBurstType(transaction->burstType);
        AhbSetTransferSize(transaction->transferSize);
        AhbSetProtection(transaction->protection);
        AhbSetWrite(transaction->write);

        result = AhbWaitReady(AHB_TIMEOUT_CYCLES);
        if (result != 0) {
            return -1;
        }

        if (transaction->write) {
            AhbWriteData(transaction->data[i]);
        } else {
            transaction->data[i] = AhbReadData();
        }

        if (AhbGetResponse() != AHB_RESP_OKAY) {
            return -1;
        }

        currentAddress += (1 << transaction->transferSize);
    }

    return 0;
}

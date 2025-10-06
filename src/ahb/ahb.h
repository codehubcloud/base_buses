#ifndef AHB_H
#define AHB_H

#include <stdint.h>

/* AHB Transfer Types (HTRANS[1:0]) */
#define AHB_TRANS_IDLE 0x00   /* No data transfer required */
#define AHB_TRANS_BUSY 0x01   /* Busy state in burst transfer */
#define AHB_TRANS_NONSEQ 0x02 /* Single transfer or first transfer of burst */
#define AHB_TRANS_SEQ 0x03    /* Remaining transfers in burst */

/* AHB Burst Types (HBURST[2:0]) */
#define AHB_BURST_SINGLE 0x00 /* Single transfer */
#define AHB_BURST_INCR 0x01   /* Incrementing burst of undefined length */
#define AHB_BURST_WRAP4 0x02  /* 4-beat wrapping burst */
#define AHB_BURST_INCR4 0x03  /* 4-beat incrementing burst */
#define AHB_BURST_WRAP8 0x04  /* 8-beat wrapping burst */
#define AHB_BURST_INCR8 0x05  /* 8-beat incrementing burst */
#define AHB_BURST_WRAP16 0x06 /* 16-beat wrapping burst */
#define AHB_BURST_INCR16 0x07 /* 16-beat incrementing burst */

/* AHB Transfer Sizes (HSIZE[2:0]) */
#define AHB_SIZE_BYTE 0x00     /* 8 bits */
#define AHB_SIZE_HALFWORD 0x01 /* 16 bits */
#define AHB_SIZE_WORD 0x02     /* 32 bits */
#define AHB_SIZE_DWORD 0x03    /* 64 bits */
#define AHB_SIZE_128BIT 0x04   /* 128 bits */
#define AHB_SIZE_256BIT 0x05   /* 256 bits */
#define AHB_SIZE_512BIT 0x06   /* 512 bits */
#define AHB_SIZE_1024BIT 0x07  /* 1024 bits */

/* AHB Response Types (HRESP[1:0]) */
#define AHB_RESP_OKAY 0x00  /* Transfer completed successfully */
#define AHB_RESP_ERROR 0x01 /* Transfer failed */
#define AHB_RESP_RETRY 0x02 /* Transfer not ready, retry */
#define AHB_RESP_SPLIT 0x03 /* Transfer split */

/* AHB Protection Control (HPROT[6:0]) */
#define AHB_PROT_DATA 0x01       /* Data access (0 = opcode fetch) */
#define AHB_PROT_PRIVILEGED 0x02 /* Privileged access */
#define AHB_PROT_BUFFERABLE 0x04 /* Bufferable */
#define AHB_PROT_CACHEABLE 0x08  /* Cacheable */
#define AHB_PROT_LOOKUP 0x10     /* Extended memory attribute lookup */
#define AHB_PROT_ALLOCATE 0x20   /* Allocate */
#define AHB_PROT_SHAREABLE 0x40  /* Shareable */

/* AHB Bus Status */
#define AHB_STATUS_IDLE 0x00  /* Bus is idle */
#define AHB_STATUS_BUSY 0x01  /* Bus is busy */
#define AHB_STATUS_ERROR 0x02 /* Bus error occurred */

/* Default Configuration */
#define AHB_DEFAULT_PRIORITY 0x00
#define AHB_MAX_BURST_LENGTH 16
#define AHB_TIMEOUT_CYCLES 1000

/* AHB Transaction Structure */
typedef struct {
    uint32_t address;     /* Transfer address */
    uint32_t* data;       /* Pointer to data buffer */
    uint16_t length;      /* Number of transfers */
    uint8_t transferType; /* Transfer type (HTRANS) */
    uint8_t burstType;    /* Burst type (HBURST) */
    uint8_t transferSize; /* Transfer size (HSIZE) */
    uint8_t protection;   /* Protection control (HPROT) */
    uint8_t write;        /* 1 = write, 0 = read */
} AhbTransaction;

/******************************************************************************
 * @brief     : Initialize AHB bus controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures AHB bus and enables master interface
 *****************************************************************************/
int32_t AhbInit(void);

/******************************************************************************
 * @brief     : Deinitialize AHB bus controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables AHB bus and resets configuration
 *****************************************************************************/
int32_t AhbDeinit(void);

/******************************************************************************
 * @brief     : Perform single AHB write transaction
 * @param[in] : address --Target address, data --Data to write, size --Transfer size
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Executes NONSEQ transfer with OKAY response check
 *****************************************************************************/
int32_t AhbWrite(uint32_t address, uint32_t data, uint8_t size);

/******************************************************************************
 * @brief     : Perform single AHB read transaction
 * @param[in] : address --Target address, size --Transfer size
 * @param[out]: data --Pointer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Executes NONSEQ transfer with OKAY response check
 *****************************************************************************/
int32_t AhbRead(uint32_t address, uint32_t* data, uint8_t size);

/******************************************************************************
 * @brief     : Perform AHB burst write transaction
 * @param[in] : address --Start address, data --Data buffer, length --Number of transfers, burstType --Burst type
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Supports INCR, INCR4/8/16, WRAP4/8/16 burst types
 *****************************************************************************/
int32_t AhbBurstWrite(uint32_t address, uint32_t* data, uint16_t length, uint8_t burstType);

/******************************************************************************
 * @brief     : Perform AHB burst read transaction
 * @param[in] : address --Start address, length --Number of transfers, burstType --Burst type
 * @param[out]: data --Buffer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Supports INCR, INCR4/8/16, WRAP4/8/16 burst types
 *****************************************************************************/
int32_t AhbBurstRead(uint32_t address, uint32_t* data, uint16_t length, uint8_t burstType);

/******************************************************************************
 * @brief     : Set AHB master priority level
 * @param[in] : priority --Priority level (0 = highest)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures bus arbitration priority for this master
 *****************************************************************************/
int32_t AhbSetPriority(uint8_t priority);

/******************************************************************************
 * @brief     : Get current AHB bus status
 * @param[in] : None
 * @param[out]: None
 * @return    : AHB_STATUS_IDLE, AHB_STATUS_BUSY, or AHB_STATUS_ERROR
 * @note      : Checks HREADY and HRESP signals
 *****************************************************************************/
int32_t AhbGetBusStatus(void);

/******************************************************************************
 * @brief     : Wait for AHB bus ready signal
 * @param[in] : timeout --Maximum cycles to wait (0 = infinite)
 * @param[out]: None
 * @return    : 0 if ready, -1 if timeout or error
 * @note      : Polls HREADY signal until high or timeout
 *****************************************************************************/
int32_t AhbWaitReady(uint32_t timeout);

/******************************************************************************
 * @brief     : Execute custom AHB transaction
 * @param[in] : transaction --Pointer to AHB transaction structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Allows full control over all AHB signals
 *****************************************************************************/
int32_t AhbExecuteTransaction(AhbTransaction* transaction);

#endif // AHB_H

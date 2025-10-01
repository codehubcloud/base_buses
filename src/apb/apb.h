#ifndef APB_H
#define APB_H

#include <stdint.h>

/* APB Protocol Version Support */
#define APB_VERSION_APB2 2
#define APB_VERSION_APB3 3
#define APB_VERSION_APB4 4

/* APB Transfer States */
typedef enum {
    APB_STATE_IDLE = 0,
    APB_STATE_SETUP = 1,
    APB_STATE_ACCESS = 2
} ApbState_E;

/* APB Transfer Type */
typedef enum {
    APB_TRANSFER_READ = 0,
    APB_TRANSFER_WRITE = 1
} ApbTransferType_E;

/* APB Protection Signals (PPROT) */
#define APB_PPROT_NORMAL 0x00
#define APB_PPROT_PRIVILEGED 0x01
#define APB_PPROT_NONSECURE 0x00
#define APB_PPROT_SECURE 0x02
#define APB_PPROT_DATA 0x00
#define APB_PPROT_INSTRUCTION 0x04

/* APB Error Response */
#define APB_PSLVERR_OK 0
#define APB_PSLVERR_ERROR 1

/* APB Default Timeout (in microseconds) */
#define APB_DEFAULT_TIMEOUT_US 1000

/******************************************************************************
 * @brief     : Initialize APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Initializes APB bus state machine and timeout
 *****************************************************************************/
int32_t ApbInit(void);

/******************************************************************************
 * @brief     : Deinitialize APB bus interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Resets APB bus to idle state
 *****************************************************************************/
int32_t ApbDeinit(void);

/******************************************************************************
 * @brief     : Perform single write transaction on APB bus
 * @param[in] : address - Peripheral address (PADDR)
 * @param[in] : data - Data to write (PWDATA)
 * @param[in] : prot - Protection signals (PPROT)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Executes SETUP and ACCESS phases for write
 *****************************************************************************/
int32_t ApbWrite(uint32_t address, uint32_t data, uint8_t prot);

/******************************************************************************
 * @brief     : Perform single read transaction on APB bus
 * @param[in] : address - Peripheral address (PADDR)
 * @param[in] : prot - Protection signals (PPROT)
 * @param[out]: data - Pointer to receive data (PRDATA)
 * @return    : 0 if success, -1 if error
 * @note      : Executes SETUP and ACCESS phases for read
 *****************************************************************************/
int32_t ApbRead(uint32_t address, uint32_t* data, uint8_t prot);

/******************************************************************************
 * @brief     : Perform write transaction with byte strobe (APB4 feature)
 * @param[in] : address - Peripheral address (PADDR)
 * @param[in] : data - Data to write (PWDATA)
 * @param[in] : strobe - Byte strobe mask (PSTRB), 1 bit per byte
 * @param[in] : prot - Protection signals (PPROT)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : APB4 only, allows partial word writes
 *****************************************************************************/
int32_t ApbWriteWithStrobe(uint32_t address, uint32_t data, uint8_t strobe, uint8_t prot);

/******************************************************************************
 * @brief     : Perform multiple consecutive read transactions
 * @param[in] : startAddress - Starting peripheral address
 * @param[in] : count - Number of words to read
 * @param[in] : prot - Protection signals (PPROT)
 * @param[out]: dataBuffer - Pointer to buffer for received data
 * @return    : Number of words read, -1 if error
 * @note      : Optimized for burst reads from consecutive addresses
 *****************************************************************************/
int32_t ApbReadMultiple(uint32_t startAddress, uint32_t* dataBuffer, uint32_t count, uint8_t prot);

/******************************************************************************
 * @brief     : Perform multiple consecutive write transactions
 * @param[in] : startAddress - Starting peripheral address
 * @param[in] : dataBuffer - Pointer to data buffer to write
 * @param[in] : count - Number of words to write
 * @param[in] : prot - Protection signals (PPROT)
 * @param[out]: None
 * @return    : Number of words written, -1 if error
 * @note      : Optimized for burst writes to consecutive addresses
 *****************************************************************************/
int32_t ApbWriteMultiple(uint32_t startAddress, uint32_t* dataBuffer, uint32_t count, uint8_t prot);

/******************************************************************************
 * @brief     : Configure transaction timeout
 * @param[in] : timeoutUs - Timeout in microseconds
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sets maximum wait time for bus ready state
 *****************************************************************************/
int32_t ApbSetTimeout(uint32_t timeoutUs);

/******************************************************************************
 * @brief     : Get last error status from APB transaction
 * @param[in] : None
 * @param[out]: None
 * @return    : APB_PSLVERR_OK or APB_PSLVERR_ERROR
 * @note      : Checks PSLVERR signal from last transaction
 *****************************************************************************/
int32_t ApbGetError(void);

/******************************************************************************
 * @brief     : Get current APB bus state
 * @param[in] : None
 * @param[out]: None
 * @return    : Current APB state (IDLE, SETUP, ACCESS)
 * @note      : Returns current state machine state
 *****************************************************************************/
ApbState_E ApbGetState(void);

#endif // APB_H

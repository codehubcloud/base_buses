#ifndef SWD_H
#define SWD_H

#include <stdint.h>

/* SWD ACK Responses */
#define SWD_ACK_OK 0x01    /* Operation successful */
#define SWD_ACK_WAIT 0x02  /* Target not ready, retry */
#define SWD_ACK_FAULT 0x04 /* Transaction fault occurred */

/* SWD DP (Debug Port) Register Addresses */
#define SWD_DP_IDCODE 0x00 /* IDCODE register */
#define SWD_DP_ABORT 0x00  /* Abort register (write) */
#define SWD_DP_CTRL 0x04   /* Control/Status register */
#define SWD_DP_SELECT 0x08 /* AP Select register */
#define SWD_DP_RDBUFF 0x0C /* Read Buffer register */

/* SWD AP (Access Port) Register Addresses */
#define SWD_AP_CSW 0x00 /* Control/Status Word */
#define SWD_AP_TAR 0x04 /* Transfer Address Register */
#define SWD_AP_DRW 0x0C /* Data Read/Write */
#define SWD_AP_IDR 0xFC /* Identification Register */

/* SWD Request Bits */
#define SWD_REQ_PARK_START 0x01
#define SWD_REQ_AP_DP 0x02      /* 0=DP, 1=AP */
#define SWD_REQ_READ_WRITE 0x04 /* 0=Write, 1=Read */

/* SWD Protocol Constants */
#define SWD_RETRY_COUNT 100

/******************************************************************************
 * @brief     : Initialize SWD interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures GPIO pins and sends JTAG-to-SWD sequence
 *****************************************************************************/
int32_t SwdInit(void);

/******************************************************************************
 * @brief     : Deinitialize SWD interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Releases GPIO pins and resets state
 *****************************************************************************/
int32_t SwdDeinit(void);

/******************************************************************************
 * @brief     : Perform SWD line reset sequence
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sends 50+ clock cycles with SWDIO high followed by idle
 *****************************************************************************/
int32_t SwdReset(void);

/******************************************************************************
 * @brief     : Read Debug Port register
 * @param[in] : address - DP register address (aligned to 4 bytes)
 * @param[out]: data - Pointer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Performs SWD read transaction with ACK checking
 *****************************************************************************/
int32_t SwdReadDP(uint8_t address, uint32_t* data);

/******************************************************************************
 * @brief     : Write Debug Port register
 * @param[in] : address - DP register address (aligned to 4 bytes), data - Data to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Performs SWD write transaction with ACK checking
 *****************************************************************************/
int32_t SwdWriteDP(uint8_t address, uint32_t data);

/******************************************************************************
 * @brief     : Read Access Port register
 * @param[in] : address - AP register address (aligned to 4 bytes)
 * @param[out]: data - Pointer to store read data
 * @return    : 0 if success, -1 if error
 * @note      : Requires prior AP selection via DP SELECT register
 *****************************************************************************/
int32_t SwdReadAP(uint8_t address, uint32_t* data);

/******************************************************************************
 * @brief     : Write Access Port register
 * @param[in] : address - AP register address (aligned to 4 bytes), data - Data to write
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Requires prior AP selection via DP SELECT register
 *****************************************************************************/
int32_t SwdWriteAP(uint8_t address, uint32_t data);

/******************************************************************************
 * @brief     : Read target device IDCODE
 * @param[in] : None
 * @param[out]: idcode - Pointer to store IDCODE value
 * @return    : 0 if success, -1 if error
 * @note      : Reads IDCODE from DP, verifies SWD connection
 *****************************************************************************/
int32_t SwdReadIdcode(uint32_t* idcode);

#endif // SWD_H

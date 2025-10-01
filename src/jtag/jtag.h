#ifndef JTAG_H
#define JTAG_H

#include <stdint.h>

/* JTAG TAP (Test Access Port) States */
typedef enum {
    JTAG_STATE_TEST_LOGIC_RESET = 0,
    JTAG_STATE_RUN_TEST_IDLE,
    JTAG_STATE_SELECT_DR_SCAN,
    JTAG_STATE_CAPTURE_DR,
    JTAG_STATE_SHIFT_DR,
    JTAG_STATE_EXIT1_DR,
    JTAG_STATE_PAUSE_DR,
    JTAG_STATE_EXIT2_DR,
    JTAG_STATE_UPDATE_DR,
    JTAG_STATE_SELECT_IR_SCAN,
    JTAG_STATE_CAPTURE_IR,
    JTAG_STATE_SHIFT_IR,
    JTAG_STATE_EXIT1_IR,
    JTAG_STATE_PAUSE_IR,
    JTAG_STATE_EXIT2_IR,
    JTAG_STATE_UPDATE_IR
} JtagState_E;

/* JTAG Instruction Register Commands (Standard IEEE 1149.1) */
typedef enum {
    JTAG_IR_EXTEST = 0x00,      /* Boundary scan test */
    JTAG_IR_SAMPLE = 0x01,      /* Sample boundary scan register */
    JTAG_IR_PRELOAD = 0x01,     /* Preload boundary scan register */
    JTAG_IR_IDCODE = 0x02,      /* Read device identification code */
    JTAG_IR_BYPASS = 0xFF       /* Bypass mode */
} JtagInstruction_E;

/* JTAG Configuration */
typedef struct {
    uint32_t clockFrequency;    /* TCK clock frequency in Hz */
    uint8_t irLength;           /* Instruction register length in bits */
    uint8_t tapCount;           /* Number of TAPs in chain */
    uint8_t enableTrst;         /* Enable TRST pin (1=yes, 0=no) */
} JtagConfig_T;

/* JTAG IDCODE Structure (32-bit IEEE 1149.1) */
typedef struct {
    uint32_t version : 4;       /* Version number */
    uint32_t partNumber : 16;   /* Part number */
    uint32_t manufacturerId : 11; /* Manufacturer ID */
    uint32_t reserved : 1;      /* Always 1 */
} JtagIdcode_T;

#define JTAG_DEFAULT_CLOCK_FREQ 1000000
#define JTAG_DEFAULT_IR_LENGTH 4
#define JTAG_MAX_CHAIN_LENGTH 32
#define JTAG_IDCODE_LENGTH 32

/******************************************************************************
 * @brief     : Initialize JTAG interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures GPIO pins and initializes TAP controller
 *****************************************************************************/
int32_t JtagInit(void);

/******************************************************************************
 * @brief     : Deinitialize JTAG interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Releases GPIO pins and resets TAP controller
 *****************************************************************************/
int32_t JtagDeinit(void);

/******************************************************************************
 * @brief     : Reset TAP controller to Test-Logic-Reset state
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Applies 5 TMS=1 clock cycles
 *****************************************************************************/
int32_t JtagReset(void);

/******************************************************************************
 * @brief     : Set TAP controller to specific state
 * @param[in] : state - Target TAP state
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Transitions through TAP state machine
 *****************************************************************************/
int32_t JtagSetState(JtagState_E state);

/******************************************************************************
 * @brief     : Shift data into Instruction Register
 * @param[in] : instruction - Instruction value to shift, bitLength - Number of bits
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Transitions to Shift-IR state and shifts instruction
 *****************************************************************************/
int32_t JtagShiftIR(uint32_t instruction, uint8_t bitLength);

/******************************************************************************
 * @brief     : Shift data into Data Register
 * @param[in] : txData - Data to transmit, bitLength - Number of bits
 * @param[out]: rxData - Received data from TDO
 * @return    : 0 if success, -1 if error
 * @note      : Transitions to Shift-DR state and shifts data
 *****************************************************************************/
int32_t JtagShiftDR(uint8_t* txData, uint8_t* rxData, uint16_t bitLength);

/******************************************************************************
 * @brief     : Read device IDCODE register
 * @param[in] : None
 * @param[out]: idcode - 32-bit IDCODE value
 * @return    : 0 if success, -1 if error
 * @note      : Loads IDCODE instruction and reads DR
 *****************************************************************************/
int32_t JtagReadIdcode(uint32_t* idcode);

/******************************************************************************
 * @brief     : Write data to JTAG register
 * @param[in] : instruction - IR instruction, data - Data to write, bitLength - Number of bits
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Loads IR then shifts data into DR
 *****************************************************************************/
int32_t JtagWriteRegister(uint32_t instruction, uint8_t* data, uint16_t bitLength);

/******************************************************************************
 * @brief     : Read data from JTAG register
 * @param[in] : instruction - IR instruction, bitLength - Number of bits to read
 * @param[out]: data - Buffer to receive data
 * @return    : 0 if success, -1 if error
 * @note      : Loads IR then shifts data from DR
 *****************************************************************************/
int32_t JtagReadRegister(uint32_t instruction, uint8_t* data, uint16_t bitLength);

/******************************************************************************
 * @brief     : Set JTAG clock frequency
 * @param[in] : frequency - Clock frequency in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures TCK clock speed
 *****************************************************************************/
int32_t JtagSetClockFrequency(uint32_t frequency);

/******************************************************************************
 * @brief     : Configure JTAG parameters
 * @param[in] : config - JTAG configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sets IR length, TAP count, and other parameters
 *****************************************************************************/
int32_t JtagConfigure(JtagConfig_T* config);

/******************************************************************************
 * @brief     : Get current TAP state
 * @param[in] : None
 * @param[out]: None
 * @return    : Current JtagState_E value
 * @note      : Returns internal TAP state tracker
 *****************************************************************************/
JtagState_E JtagGetState(void);

/******************************************************************************
 * @brief     : Run Test/Idle for specified clock cycles
 * @param[in] : cycles - Number of TCK cycles in Run-Test/Idle
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Useful for delay between operations
 *****************************************************************************/
int32_t JtagRunTestIdle(uint32_t cycles);

#endif // JTAG_H

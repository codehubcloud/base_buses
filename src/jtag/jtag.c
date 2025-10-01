#include <string.h>
#include "jtag.h"
#include "jtag_hal.h"
#include "securec.h"


/* Internal state tracking */
static JtagState_E g_currentState = JTAG_STATE_TEST_LOGIC_RESET;
static JtagConfig_T g_jtagConfig = {.clockFrequency = JTAG_DEFAULT_CLOCK_FREQ,
                                    .irLength = JTAG_DEFAULT_IR_LENGTH,
                                    .tapCount = 1,
                                    .enableTrst = 0};

/* TAP State Transition Table (TMS values to reach target state) */
static const uint8_t g_stateTransitions[16][16] = {
    /* From TEST_LOGIC_RESET */ {0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03},
    /* From RUN_TEST_IDLE */ {0x07, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03},
    /* Continue for all 16 states... */
};

/******************************************************************************
 * @brief     : Perform single TCK clock cycle with TMS and TDI values
 * @param[in] : tms - TMS bit value, tdi - TDI bit value
 * @param[out]: None
 * @return    : TDO bit value
 * @note      : Internal helper function
 *****************************************************************************/
static uint8_t JtagClockCycle(uint8_t tms, uint8_t tdi)
{
    uint8_t tdo;

    JtagSetTms(tms);
    JtagSetTdi(tdi);
    JtagSetTck(0);
    JtagDelayUs(1);
    tdo = JtagGetTdo();
    JtagSetTck(1);
    JtagDelayUs(1);

    return tdo;
}

/******************************************************************************
 * @brief     : Shift bits through JTAG interface
 * @param[in] : txData - Data to transmit, bitLength - Number of bits, exitState - 1 to exit on last bit
 * @param[out]: rxData - Received data buffer
 * @return    : 0 if success, -1 if error
 * @note      : Internal helper function for IR/DR shifting
 *****************************************************************************/
static int32_t JtagShiftBits(uint8_t* txData, uint8_t* rxData, uint16_t bitLength, uint8_t exitState)
{
    uint16_t i;
    uint8_t tdi;
    uint8_t tdo;
    uint8_t tms;

    if ((txData == NULL) || (bitLength == 0)) {
        return -1;
    }

    for (i = 0; i < bitLength; i++) {
        tdi = (txData[i / 8] >> (i % 8)) & 0x01;
        tms = (exitState && (i == bitLength - 1)) ? 1 : 0;

        tdo = JtagClockCycle(tms, tdi);

        if (rxData != NULL) {
            if (tdo) {
                rxData[i / 8] |= (1 << (i % 8));
            } else {
                rxData[i / 8] &= ~(1 << (i % 8));
            }
        }
    }

    return 0;
}

/******************************************************************************
 * @brief     : Initialize JTAG interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures GPIO pins and initializes TAP controller
 *****************************************************************************/
int32_t JtagInit(void)
{
    int32_t result;

    result = JtagConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = JtagEnableClock();
    if (result != 0) {
        return -1;
    }

    /* Initialize pin states */
    JtagSetTck(0);
    JtagSetTms(1);
    JtagSetTdi(0);

    if (g_jtagConfig.enableTrst) {
        JtagSetTrst(0);
        JtagDelayUs(100);
        JtagSetTrst(1);
    }

    /* Reset TAP controller */
    result = JtagReset();
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize JTAG interface
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Releases GPIO pins and resets TAP controller
 *****************************************************************************/
int32_t JtagDeinit(void)
{
    JtagReset();
    JtagReleaseGpio();
    g_currentState = JTAG_STATE_TEST_LOGIC_RESET;

    return 0;
}

/******************************************************************************
 * @brief     : Reset TAP controller to Test-Logic-Reset state
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Applies 5 TMS=1 clock cycles
 *****************************************************************************/
int32_t JtagReset(void)
{
    uint8_t i;

    /* Apply 5 TMS=1 clock cycles to reach Test-Logic-Reset */
    for (i = 0; i < 5; i++) {
        JtagClockCycle(1, 0);
    }

    g_currentState = JTAG_STATE_TEST_LOGIC_RESET;

    return 0;
}

/******************************************************************************
 * @brief     : Set TAP controller to specific state
 * @param[in] : state - Target TAP state
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Transitions through TAP state machine
 *****************************************************************************/
int32_t JtagSetState(JtagState_E state)
{
    uint8_t tmsSequence[16];
    uint8_t sequenceLength = 0;
    uint8_t i;
    JtagState_E currentState = g_currentState;

    if (state >= 16) {
        return -1;
    }

    /* Calculate TMS sequence to reach target state */
    while (currentState != state && sequenceLength < 16) {
        /* Simplified state machine navigation */
        if (currentState == JTAG_STATE_TEST_LOGIC_RESET && state == JTAG_STATE_RUN_TEST_IDLE) {
            tmsSequence[sequenceLength++] = 0;
            currentState = JTAG_STATE_RUN_TEST_IDLE;
        } else if (currentState == JTAG_STATE_RUN_TEST_IDLE && state == JTAG_STATE_SHIFT_DR) {
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 0;
            tmsSequence[sequenceLength++] = 0;
            currentState = JTAG_STATE_SHIFT_DR;
        } else if (currentState == JTAG_STATE_RUN_TEST_IDLE && state == JTAG_STATE_SHIFT_IR) {
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 0;
            tmsSequence[sequenceLength++] = 0;
            currentState = JTAG_STATE_SHIFT_IR;
        } else if (currentState == JTAG_STATE_SHIFT_DR && state == JTAG_STATE_RUN_TEST_IDLE) {
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 0;
            currentState = JTAG_STATE_RUN_TEST_IDLE;
        } else if (currentState == JTAG_STATE_SHIFT_IR && state == JTAG_STATE_RUN_TEST_IDLE) {
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 1;
            tmsSequence[sequenceLength++] = 0;
            currentState = JTAG_STATE_RUN_TEST_IDLE;
        } else {
            /* Default: go through reset */
            return JtagReset();
        }
    }

    /* Apply TMS sequence */
    for (i = 0; i < sequenceLength; i++) {
        JtagClockCycle(tmsSequence[i], 0);
    }

    g_currentState = state;

    return 0;
}

/******************************************************************************
 * @brief     : Shift data into Instruction Register
 * @param[in] : instruction - Instruction value to shift, bitLength - Number of bits
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Transitions to Shift-IR state and shifts instruction
 *****************************************************************************/
int32_t JtagShiftIR(uint32_t instruction, uint8_t bitLength)
{
    uint8_t txData[4];
    int32_t result;

    if (bitLength == 0 || bitLength > 32) {
        return -1;
    }

    /* Convert instruction to byte array */
    txData[0] = (uint8_t)(instruction & 0xFF);
    txData[1] = (uint8_t)((instruction >> 8) & 0xFF);
    txData[2] = (uint8_t)((instruction >> 16) & 0xFF);
    txData[3] = (uint8_t)((instruction >> 24) & 0xFF);

    /* Navigate to Shift-IR state */
    result = JtagSetState(JTAG_STATE_SHIFT_IR);
    if (result != 0) {
        return -1;
    }

    /* Shift instruction, exit to Exit1-IR on last bit */
    result = JtagShiftBits(txData, NULL, bitLength, 1);
    if (result != 0) {
        return -1;
    }

    g_currentState = JTAG_STATE_EXIT1_IR;

    /* Return to Run-Test/Idle */
    return JtagSetState(JTAG_STATE_RUN_TEST_IDLE);
}

/******************************************************************************
 * @brief     : Shift data into Data Register
 * @param[in] : txData - Data to transmit, bitLength - Number of bits
 * @param[out]: rxData - Received data from TDO
 * @return    : 0 if success, -1 if error
 * @note      : Transitions to Shift-DR state and shifts data
 *****************************************************************************/
int32_t JtagShiftDR(uint8_t* txData, uint8_t* rxData, uint16_t bitLength)
{
    int32_t result;

    if (txData == NULL || bitLength == 0) {
        return -1;
    }

    /* Clear receive buffer if provided */
    if (rxData != NULL) {
        if (memset_s(rxData, (bitLength + 7) / 8, 0, (bitLength + 7) / 8) != EOK) {
            return -1;
        }
    }

    /* Navigate to Shift-DR state */
    result = JtagSetState(JTAG_STATE_SHIFT_DR);
    if (result != 0) {
        return -1;
    }

    /* Shift data, exit to Exit1-DR on last bit */
    result = JtagShiftBits(txData, rxData, bitLength, 1);
    if (result != 0) {
        return -1;
    }

    g_currentState = JTAG_STATE_EXIT1_DR;

    /* Return to Run-Test/Idle */
    return JtagSetState(JTAG_STATE_RUN_TEST_IDLE);
}

/******************************************************************************
 * @brief     : Read device IDCODE register
 * @param[in] : None
 * @param[out]: idcode - 32-bit IDCODE value
 * @return    : 0 if success, -1 if error
 * @note      : Loads IDCODE instruction and reads DR
 *****************************************************************************/
int32_t JtagReadIdcode(uint32_t* idcode)
{
    uint8_t rxData[4] = {0};
    int32_t result;

    if (idcode == NULL) {
        return -1;
    }

    /* Load IDCODE instruction */
    result = JtagShiftIR(JTAG_IR_IDCODE, g_jtagConfig.irLength);
    if (result != 0) {
        return -1;
    }

    /* Read 32-bit IDCODE from DR */
    uint8_t txData[4] = {0x00, 0x00, 0x00, 0x00};
    result = JtagShiftDR(txData, rxData, JTAG_IDCODE_LENGTH);
    if (result != 0) {
        return -1;
    }

    /* Convert byte array to 32-bit value */
    *idcode = ((uint32_t)rxData[0]) | ((uint32_t)rxData[1] << 8) | ((uint32_t)rxData[2] << 16) | ((uint32_t)rxData[3] << 24);

    return 0;
}

/******************************************************************************
 * @brief     : Write data to JTAG register
 * @param[in] : instruction - IR instruction, data - Data to write, bitLength - Number of bits
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Loads IR then shifts data into DR
 *****************************************************************************/
int32_t JtagWriteRegister(uint32_t instruction, uint8_t* data, uint16_t bitLength)
{
    int32_t result;

    if (data == NULL || bitLength == 0) {
        return -1;
    }

    /* Load instruction */
    result = JtagShiftIR(instruction, g_jtagConfig.irLength);
    if (result != 0) {
        return -1;
    }

    /* Write data to DR */
    result = JtagShiftDR(data, NULL, bitLength);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Read data from JTAG register
 * @param[in] : instruction - IR instruction, bitLength - Number of bits to read
 * @param[out]: data - Buffer to receive data
 * @return    : 0 if success, -1 if error
 * @note      : Loads IR then shifts data from DR
 *****************************************************************************/
int32_t JtagReadRegister(uint32_t instruction, uint8_t* data, uint16_t bitLength)
{
    int32_t result;
    uint8_t txData[JTAG_MAX_CHAIN_LENGTH];

    if (data == NULL || bitLength == 0 || bitLength > JTAG_MAX_CHAIN_LENGTH * 8) {
        return -1;
    }

    /* Load instruction */
    result = JtagShiftIR(instruction, g_jtagConfig.irLength);
    if (result != 0) {
        return -1;
    }

    /* Prepare dummy data for reading */
    if (memset_s(txData, sizeof(txData), 0, (bitLength + 7) / 8) != EOK) {
        return -1;
    }

    /* Read data from DR */
    result = JtagShiftDR(txData, data, bitLength);
    if (result != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * @brief     : Set JTAG clock frequency
 * @param[in] : frequency - Clock frequency in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures TCK clock speed
 *****************************************************************************/
int32_t JtagSetClockFrequency(uint32_t frequency)
{
    if (frequency == 0 || frequency > 10000000) {
        return -1;
    }

    g_jtagConfig.clockFrequency = frequency;

    return JtagConfigureClockSpeed(frequency);
}

/******************************************************************************
 * @brief     : Configure JTAG parameters
 * @param[in] : config - JTAG configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Sets IR length, TAP count, and other parameters
 *****************************************************************************/
int32_t JtagConfigure(JtagConfig_T* config)
{
    if (config == NULL) {
        return -1;
    }

    if (config->irLength == 0 || config->irLength > 32) {
        return -1;
    }

    if (config->tapCount == 0 || config->tapCount > JTAG_MAX_CHAIN_LENGTH) {
        return -1;
    }

    if (memcpy_s(&g_jtagConfig, sizeof(JtagConfig_T), config, sizeof(JtagConfig_T)) != EOK) {
        return -1;
    }

    return JtagSetClockFrequency(config->clockFrequency);
}

/******************************************************************************
 * @brief     : Get current TAP state
 * @param[in] : None
 * @param[out]: None
 * @return    : Current JtagState_E value
 * @note      : Returns internal TAP state tracker
 *****************************************************************************/
JtagState_E JtagGetState(void)
{
    return g_currentState;
}

/******************************************************************************
 * @brief     : Run Test/Idle for specified clock cycles
 * @param[in] : cycles - Number of TCK cycles in Run-Test/Idle
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Useful for delay between operations
 *****************************************************************************/
int32_t JtagRunTestIdle(uint32_t cycles)
{
    uint32_t i;
    int32_t result;

    result = JtagSetState(JTAG_STATE_RUN_TEST_IDLE);
    if (result != 0) {
        return -1;
    }

    /* Clock with TMS=0 to stay in Run-Test/Idle */
    for (i = 0; i < cycles; i++) {
        JtagClockCycle(0, 0);
    }

    return 0;
}

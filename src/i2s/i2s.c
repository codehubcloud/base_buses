#include <string.h>
#include "i2s.h"
#include "i2s_hal.h"
#include "securec.h"


/******************************************************************************
 * @brief     : Initialize I2S peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I2S with 44.1kHz sample rate and 16-bit format
 *****************************************************************************/
int32_t I2sInit(void)
{
    int32_t result = 0;

    result = I2sEnableClock();
    if (result != 0) {
        return -1;
    }

    result = I2sConfigureGpio();
    if (result != 0) {
        return -1;
    }

    result = I2sSetSampleRate(I2S_DEFAULT_SAMPLE_RATE);
    if (result != 0) {
        return -1;
    }

    result = I2sConfigureFormat(I2S_DEFAULT_DATA_FORMAT, I2S_CHANNEL_STEREO);
    if (result != 0) {
        return -1;
    }

    I2sEnable();
    return 0;
}

/******************************************************************************
 * @brief     : Deinitialize I2S peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables I2S and releases resources
 *****************************************************************************/
int32_t I2sDeinit(void)
{
    I2sDisable();
    I2sDisableClock();
    return 0;
}

/******************************************************************************
 * @brief     : Configure I2S audio format and parameters
 * @param[in] : dataFormat --Audio data format (16/24/32-bit)
 *              channelMode - Channel mode (stereo/mono)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before starting audio transmission
 *****************************************************************************/
int32_t I2sConfigureFormat(uint8_t dataFormat, uint8_t channelMode)
{
    if ((dataFormat > I2S_DATA_FORMAT_32BIT) || (channelMode > I2S_CHANNEL_MONO_RIGHT)) {
        return -1;
    }

    return I2sSetDataFormat(dataFormat, channelMode);
}

/******************************************************************************
 * @brief     : Transmit audio data through I2S interface
 * @param[in] : data --Pointer to audio data buffer
 *              length - Number of bytes to transmit
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for transmission to complete
 *****************************************************************************/
int32_t I2sTransmitData(uint8_t* data, uint32_t length)
{
    if ((data == NULL) || (length == 0)) {
        return -1;
    }

    return I2sWrite(data, length);
}

/******************************************************************************
 * @brief     : Receive audio data from I2S interface
 * @param[in] : maxLength --Maximum number of bytes to receive
 * @param[out]: buffer --Received audio data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Blocking function that waits for data reception
 *****************************************************************************/
int32_t I2sReceiveData(uint8_t* buffer, uint32_t maxLength)
{
    if ((buffer == NULL) || (maxLength == 0)) {
        return -1;
    }

    return I2sRead(buffer, maxLength);
}

/******************************************************************************
 * @brief     : Set I2S sample rate
 * @param[in] : sampleRate --Desired sample rate in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Common sample rates: 8kHz to 192kHz
 *****************************************************************************/
int32_t I2sSetSampleRate(uint32_t sampleRate)
{
    if ((sampleRate < I2S_SAMPLE_RATE_8KHZ) || (sampleRate > I2S_SAMPLE_RATE_192KHZ)) {
        return -1;
    }

    return I2sConfigureSampleRate(sampleRate);
}

/******************************************************************************
 * @brief     : Enable DMA for I2S data transfer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables efficient audio streaming without CPU intervention
 *****************************************************************************/
int32_t I2sEnableDMA(void)
{
    return I2sConfigureDMA(1);
}

/******************************************************************************
 * @brief     : Disable DMA for I2S data transfer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables DMA and reverts to CPU-based transfers
 *****************************************************************************/
int32_t I2sDisableDMA(void)
{
    return I2sConfigureDMA(0);
}

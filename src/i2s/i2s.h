#ifndef I2S_H
#define I2S_H

#include <stdint.h>

/* I2S audio format definitions */
#define I2S_DATA_FORMAT_16BIT 0
#define I2S_DATA_FORMAT_24BIT 1
#define I2S_DATA_FORMAT_32BIT 2

/* I2S channel modes */
#define I2S_CHANNEL_STEREO 0
#define I2S_CHANNEL_MONO_LEFT 1
#define I2S_CHANNEL_MONO_RIGHT 2

/* I2S operating modes */
#define I2S_MODE_MASTER_TX 0
#define I2S_MODE_MASTER_RX 1
#define I2S_MODE_SLAVE_TX 2
#define I2S_MODE_SLAVE_RX 3

/* Standard sample rates */
#define I2S_SAMPLE_RATE_8KHZ 8000
#define I2S_SAMPLE_RATE_11KHZ 11025
#define I2S_SAMPLE_RATE_16KHZ 16000
#define I2S_SAMPLE_RATE_22KHZ 22050
#define I2S_SAMPLE_RATE_32KHZ 32000
#define I2S_SAMPLE_RATE_44KHZ 44100
#define I2S_SAMPLE_RATE_48KHZ 48000
#define I2S_SAMPLE_RATE_96KHZ 96000
#define I2S_SAMPLE_RATE_192KHZ 192000

/* Default settings */
#define I2S_DEFAULT_SAMPLE_RATE I2S_SAMPLE_RATE_44KHZ
#define I2S_DEFAULT_DATA_FORMAT I2S_DATA_FORMAT_16BIT

/******************************************************************************
 * @brief     : Initialize I2S peripheral with default settings
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Configures I2S with 44.1kHz sample rate and 16-bit format
 *****************************************************************************/
int32_t I2sInit(void);

/******************************************************************************
 * @brief     : Deinitialize I2S peripheral
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables I2S and releases resources
 *****************************************************************************/
int32_t I2sDeinit(void);

/******************************************************************************
 * @brief     : Configure I2S audio format and parameters
 * @param[in] : dataFormat --Audio data format (16/24/32-bit)
 *              channelMode - Channel mode (stereo/mono)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before starting audio transmission
 *****************************************************************************/
int32_t I2sConfigureFormat(uint8_t dataFormat, uint8_t channelMode);

/******************************************************************************
 * @brief     : Transmit audio data through I2S interface
 * @param[in] : data --Pointer to audio data buffer
 *              length - Number of bytes to transmit
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Blocking function that waits for transmission to complete
 *****************************************************************************/
int32_t I2sTransmitData(uint8_t* data, uint32_t length);

/******************************************************************************
 * @brief     : Receive audio data from I2S interface
 * @param[in] : maxLength --Maximum number of bytes to receive
 * @param[out]: buffer --Received audio data
 * @return    : Number of bytes actually received, -1 if error
 * @note      : Blocking function that waits for data reception
 *****************************************************************************/
int32_t I2sReceiveData(uint8_t* buffer, uint32_t maxLength);

/******************************************************************************
 * @brief     : Set I2S sample rate
 * @param[in] : sampleRate --Desired sample rate in Hz
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Common sample rates: 8kHz to 192kHz
 *****************************************************************************/
int32_t I2sSetSampleRate(uint32_t sampleRate);

/******************************************************************************
 * @brief     : Enable DMA for I2S data transfer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables efficient audio streaming without CPU intervention
 *****************************************************************************/
int32_t I2sEnableDMA(void);

/******************************************************************************
 * @brief     : Disable DMA for I2S data transfer
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables DMA and reverts to CPU-based transfers
 *****************************************************************************/
int32_t I2sDisableDMA(void);

#endif // I2S_H

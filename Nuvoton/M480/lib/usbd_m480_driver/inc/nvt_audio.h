/**************************************************************************//**
 * @file     usbd_audio.h
 * @brief    NuMicro series USB driver header file
 * @version  1.0.0
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __NVT_UAC_H__
#define __NVT_UAC_H__

#define NAU8822     0

#define AUDIO_RATE  AUDIO_RATE_48K

#define AUDIO_RATE_48K   48000       /* The audo play sampling rate. The setting is 48KHz */
#define AUDIO_RATE_96K   96000       /* The audo play sampling rate. The setting is 96KHz */
#define AUDIO_RATE_441K  44100       /* The audo play sampling rate. The setting is 44.1KHz */

#define BUFF_LEN    800


#define PDMA_TXBUFFER_CNT     7
#define PDMA_RXBUFFER_CNT     8

#define PDMA_I2S_TX_CH  1
#define PDMA_I2S_RX_CH  2

/* For I2C transfer */
typedef enum
{
    E_RS_NONE,          // no re-sampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;


/*-------------------------------------------------------------*/

void AudioStartPlay(uint32_t u32SampleRate);
void AudioStartRecord(uint32_t u32SampleRate);
void AdjustCodecPll(RESAMPLE_STATE_T r);

#if NAU8822
void NAU8822_Setup(void);
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate);
#else
void NAU88L25_Reset(void);
void NAU88L25_Setup(void);
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate);
#endif

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;


#endif  /* __NVT_UAC_H_ */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

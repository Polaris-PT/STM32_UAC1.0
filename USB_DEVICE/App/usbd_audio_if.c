/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : usbd_audio_if.c
 * @version        : v1.0_Cube
 * @brief          : Generic media access layer.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include "sai.h"

/*
 * Ping-Pong(半缓冲)播放方案：
 * - USB OUT 每包数据进入本文件维护的 PCM16 立体声环形 FIFO（usb_pcm_ring）。
 * - SAI DMA 以 circular 方式持续播放 sai_tx_buf（32bit slot，左对齐）。
 * - DMA half/full 回调里，只更新“刚刚播放完的那一半缓冲”（安全区），避免覆盖 DMA 正在读的数据。
 * - 简易同步：当 FIFO 水位过低/过高时，每半缓冲执行一次 ±1 stereo frame 的丢/插帧。
 */

extern SAI_HandleTypeDef hsai_BlockA1;

static void AUDIO_StartSaiIfNeeded_FS(void);

/* 半传输完成回调：DMA 刚刚完成半区0的传输，接下来将读取半区1 -> 半区0安全可写 */
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == hsai_BlockA1.Instance)
  {
    HalfTransfer_CallBack_FS();
  }
}

/* 完全传输完成回调：DMA 完成整个缓冲区（半区0+半区1），即将重新从半区0开始 -> 半区1安全可写 */
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == hsai_BlockA1.Instance)
  {
    TransferComplete_CallBack_FS();
  }
}

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* 单一 DMA 播放缓冲：双半区 Ping-Pong（每个 16bit 样本扩展为 32bit 左对齐） */
static int32_t sai_tx_buf[AUDIO_TOTAL_BUF_SIZE / 2] __attribute__((aligned(32)));
static volatile uint8_t sai_started = 0U;
static volatile uint8_t dma_ready_mask = 0U; /* bit0: half0 ready, bit1: half1 ready */
static volatile uint32_t fill_word_pos = 0U; /* [0, SAI_TX_WORDS_TOTAL) */

#define PCM_BYTES_PER_FRAME 4U /* L16 + R16 */
#define SAI_WORDS_PER_FRAME 2U /* L32 + R32 */
#define SAI_TX_WORDS_TOTAL ((uint32_t)(AUDIO_TOTAL_BUF_SIZE / 2U))
#define SAI_TX_WORDS_HALF (SAI_TX_WORDS_TOTAL / 2U)
#define DMA_HALF0_READY_BIT 0x01U
#define DMA_HALF1_READY_BIT 0x02U
#define DMA_BOTH_READY_MASK (DMA_HALF0_READY_BIT | DMA_HALF1_READY_BIT)

static inline void AUDIO_LockIRQ(uint32_t *primask)
{
  *primask = __get_PRIMASK();
  __disable_irq();
}

static inline void AUDIO_UnlockIRQ(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static inline int32_t AUDIO_PCM16_To_SAI32(int16_t sample)
{
  return ((int32_t)sample) << 16;
}

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @brief Usb device library.
 * @{
 */

/** @addtogroup USBD_AUDIO_IF
 * @{
 */

/** @defgroup USBD_AUDIO_IF_Private_TypesDefinitions USBD_AUDIO_IF_Private_TypesDefinitions
 * @brief Private types.
 * @{
 */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_Defines USBD_AUDIO_IF_Private_Defines
 * @brief Private defines.
 * @{
 */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_Macros USBD_AUDIO_IF_Private_Macros
 * @brief Private macros.
 * @{
 */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_Variables USBD_AUDIO_IF_Private_Variables
 * @brief Private variables.
 * @{
 */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Exported_Variables USBD_AUDIO_IF_Exported_Variables
 * @brief Public variables.
 * @{
 */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_AUDIO_IF_Private_FunctionPrototypes USBD_AUDIO_IF_Private_FunctionPrototypes
 * @brief Private functions declaration.
 * @{
 */

static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
    {
        AUDIO_Init_FS,
        AUDIO_DeInit_FS,
        AUDIO_AudioCmd_FS,
        AUDIO_VolumeCtl_FS,
        AUDIO_MuteCtl_FS,
        AUDIO_PeriodicTC_FS,
        AUDIO_GetState_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initializes the AUDIO media low layer over USB FS IP
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
 * @param  options: Reserved for future use
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);

  uint32_t primask;
  AUDIO_LockIRQ(&primask);
  sai_started = 0U;
  dma_ready_mask = 0U;
  fill_word_pos = 0U;
  AUDIO_UnlockIRQ(primask);

  (void)memset((void *)sai_tx_buf, 0, sizeof(sai_tx_buf));
  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
 * @brief  De-Initializes the AUDIO media low layer
 * @param  options: Reserved for future use
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  UNUSED(options);

  uint32_t primask;
  AUDIO_LockIRQ(&primask);
  sai_started = 0U;
  dma_ready_mask = 0U;
  fill_word_pos = 0U;
  AUDIO_UnlockIRQ(primask);

  if (HAL_SAI_GetState(&hsai_BlockA1) != HAL_SAI_STATE_RESET)
  {
    (void)HAL_SAI_DMAStop(&hsai_BlockA1);
  }
  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
 * @brief  Handles AUDIO command.
 * @param  pbuf: Pointer to buffer of data to be sent
 * @param  size: Number of data to be sent (in bytes)
 * @param  cmd: Command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_AudioCmd_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
  UNUSED(pbuf);
  UNUSED(size);

  switch (cmd)
  {
  case AUDIO_CMD_START:
    /* 注意：该回调通常在 USB 中断上下文触发，避免在此做大循环填充 */
    AUDIO_StartSaiIfNeeded_FS();
    break;

  case AUDIO_CMD_PLAY:
    /* Ping-Pong 方案下持续播放由 DMA 回调维持，这里只做“保底启动” */
    AUDIO_StartSaiIfNeeded_FS();
    break;

  case AUDIO_CMD_STOP:
  {
    uint32_t primask;
    AUDIO_LockIRQ(&primask);
    sai_started = 0U;
    dma_ready_mask = 0U;
    fill_word_pos = 0U;
    AUDIO_UnlockIRQ(primask);

    (void)memset((void *)sai_tx_buf, 0, sizeof(sai_tx_buf));
    (void)HAL_SAI_DMAStop(&hsai_BlockA1);
    break;
  }

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
 * @brief  Controls AUDIO Volume.
 * @param  vol: volume level (0..100)
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  UNUSED(vol);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
 * @brief  Controls AUDIO Mute.
 * @param  cmd: command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  UNUSED(cmd);
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
 * @brief  AUDIO_PeriodicT_FS
 * @param  cmd: Command opcode
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  if (cmd != AUDIO_OUT_TC)
    return USBD_OK;

  /* USB 包直接写入 DMA 缓冲（无额外应用层 FIFO） */
  if ((pbuf == NULL) || (size == 0U))
  {
    return USBD_OK;
  }

  /* 防止半个采样帧进入 FIFO，避免左右声道错位 */
  size -= (size % PCM_BYTES_PER_FRAME);
  if (size == 0U)
  {
    return USBD_OK;
  }

  uint32_t frames_left = size / PCM_BYTES_PER_FRAME;
  const uint8_t *src = pbuf;

  while (frames_left > 0U)
  {
    uint32_t pos;
    uint8_t half;
    uint8_t half_busy;
    uint32_t primask;

    AUDIO_LockIRQ(&primask);
    pos = fill_word_pos;
    half = (pos < SAI_TX_WORDS_HALF) ? 0U : 1U;
    half_busy = ((dma_ready_mask & (half == 0U ? DMA_HALF0_READY_BIT : DMA_HALF1_READY_BIT)) != 0U) ? 1U : 0U;
    AUDIO_UnlockIRQ(primask);

    /* 生产者追上消费者：直接丢弃本包剩余数据，保持逻辑简单 */
    if (half_busy != 0U)
    {
      break;
    }

    uint32_t pos_in_half = (half == 0U) ? pos : (pos - SAI_TX_WORDS_HALF);
    uint32_t free_words = SAI_TX_WORDS_HALF - pos_in_half;
    uint32_t free_frames = free_words / SAI_WORDS_PER_FRAME;
    uint32_t n = (frames_left < free_frames) ? frames_left : free_frames;

    int32_t *dst = &sai_tx_buf[pos];
    for (uint32_t i = 0U; i < n; i++)
    {
      uint16_t l16 = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
      uint16_t r16 = (uint16_t)src[2] | ((uint16_t)src[3] << 8);
      *dst++ = AUDIO_PCM16_To_SAI32((int16_t)l16);
      *dst++ = AUDIO_PCM16_To_SAI32((int16_t)r16);
      src += PCM_BYTES_PER_FRAME;
    }

    frames_left -= n;
    pos += n * SAI_WORDS_PER_FRAME;

    AUDIO_LockIRQ(&primask);
    fill_word_pos = pos;
    if ((half == 0U) && (fill_word_pos >= SAI_TX_WORDS_HALF))
    {
      dma_ready_mask |= DMA_HALF0_READY_BIT;
    }
    else if ((half == 1U) && (fill_word_pos >= SAI_TX_WORDS_TOTAL))
    {
      dma_ready_mask |= DMA_HALF1_READY_BIT;
      fill_word_pos = 0U;
    }
    AUDIO_UnlockIRQ(primask);
  }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr((uint32_t *)sai_tx_buf, (int32_t)sizeof(sai_tx_buf));
#endif

  /* 某些主机/栈流程下 CMD_START 可能没有可靠触发，这里做数据到达后的保底启动 */
  AUDIO_StartSaiIfNeeded_FS();

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
 * @brief  Gets AUDIO State.
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
 * @brief  Manages the DMA full transfer complete event.
 * @retval None
 */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  /* DMA 刚播放完后半区，释放后半区给 USB 生产者 */
  if (sai_started != 0U)
  {
    uint32_t primask;
    AUDIO_LockIRQ(&primask);
    dma_ready_mask &= (uint8_t)(~DMA_HALF1_READY_BIT);
    AUDIO_UnlockIRQ(primask);
  }
  /* USER CODE END 7 */
}

/**
 * @brief  Manages the DMA Half transfer complete event.
 * @retval None
 */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  /* DMA 刚播放完前半区，释放前半区给 USB 生产者 */
  if (sai_started != 0U)
  {
    uint32_t primask;
    AUDIO_LockIRQ(&primask);
    dma_ready_mask &= (uint8_t)(~DMA_HALF0_READY_BIT);
    AUDIO_UnlockIRQ(primask);
  }
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

static void AUDIO_StartSaiIfNeeded_FS(void)
{
  uint32_t primask;
  uint8_t ready_mask;

  if (sai_started != 0U)
  {
    return;
  }

  if (HAL_SAI_GetState(&hsai_BlockA1) != HAL_SAI_STATE_READY)
  {
    return;
  }

  AUDIO_LockIRQ(&primask);
  ready_mask = dma_ready_mask;
  AUDIO_UnlockIRQ(primask);

  /* 两个半区都准备好后再启动，避免开播即欠载 */
  if ((ready_mask & DMA_BOTH_READY_MASK) != DMA_BOTH_READY_MASK)
  {
    return;
  }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr((uint32_t *)sai_tx_buf, (int32_t)sizeof(sai_tx_buf));
#endif

  if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)sai_tx_buf, (uint16_t)SAI_TX_WORDS_TOTAL) == HAL_OK)
  {
    sai_started = 1U;
  }
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */

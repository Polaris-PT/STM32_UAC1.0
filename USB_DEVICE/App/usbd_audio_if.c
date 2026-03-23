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
#include <stdint.h>
#include <string.h>
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

static void AUDIO_FillHalfTxBuffer_FS(uint32_t dst_word_offset);
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

/* PCM16 stereo (4 bytes/frame) 环形 FIFO，单位字节 */
static uint8_t usb_pcm_ring[AUDIO_TOTAL_BUF_SIZE] __attribute__((aligned(32)));
static volatile uint32_t usb_pcm_wr = 0U;
static volatile uint32_t usb_pcm_rd = 0U;
static volatile uint32_t usb_pcm_level = 0U; /* 当前 FIFO 内有效字节数 */

/* SAI DMA 播放缓冲：每个 16bit 样本扩展为 32bit（左对齐），故 word 数 = AUDIO_TOTAL_BUF_SIZE/2 */
static int32_t sai_tx_buf[AUDIO_TOTAL_BUF_SIZE / 2] __attribute__((aligned(32)));
static volatile uint8_t sai_started = 0U;

#define PCM_BYTES_PER_FRAME 4U /* L16 + R16 */
#define SAI_WORDS_PER_FRAME 2U /* L32 + R32 */
#define SAI_TX_WORDS_TOTAL ((uint32_t)(AUDIO_TOTAL_BUF_SIZE / 2U))
#define SAI_TX_WORDS_HALF (SAI_TX_WORDS_TOTAL / 2U)
#define SAI_TX_FRAMES_HALF (SAI_TX_WORDS_HALF / SAI_WORDS_PER_FRAME)
#define USB_BYTES_PER_HALF (SAI_TX_FRAMES_HALF * PCM_BYTES_PER_FRAME)
#define AUDIO_START_LEVEL_BYTES (USB_BYTES_PER_HALF)

static inline uint32_t AUDIO_ClampU32(uint32_t value, uint32_t max)
{
  return (value > max) ? max : value;
}

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

static inline uint16_t AUDIO_ReadU16LERing(const uint8_t *ring, uint32_t cap, uint32_t pos)
{
  uint8_t lo = ring[pos];
  uint8_t hi = ring[(pos + 1U < cap) ? (pos + 1U) : 0U];
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static inline void AUDIO_AdvanceRingPos(uint32_t cap, uint32_t *pos, uint32_t inc)
{
  if (cap == 0U)
    return;
  uint32_t next = *pos + (inc % cap);
  if (next >= cap)
  {
    next -= cap;
  }
  *pos = next;
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
  usb_pcm_wr = 0U;
  usb_pcm_rd = 0U;
  usb_pcm_level = 0U;
  sai_started = 0U;
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
  usb_pcm_wr = 0U;
  usb_pcm_rd = 0U;
  usb_pcm_level = 0U;
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
    sai_started = 0U;
    (void)HAL_SAI_DMAStop(&hsai_BlockA1);
    break;

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

  /* 将 USB 收到的 PCM16 数据写入应用层 FIFO（可覆盖最老数据以避免阻塞） */
  if ((pbuf == NULL) || (size == 0U))
  {
    return USBD_OK;
  }

  const uint32_t cap = (uint32_t)sizeof(usb_pcm_ring);

  /* 异常包长保护：仅保留最近 cap 字节 */
  if (size > cap)
  {
    pbuf += (size - cap);
    size = cap;
  }

  /* 防止半个采样帧进入 FIFO，避免左右声道错位 */
  size -= (size % PCM_BYTES_PER_FRAME);
  if (size == 0U)
  {
    return USBD_OK;
  }

  uint32_t wr;
  uint32_t level;
  uint32_t primask;

  AUDIO_LockIRQ(&primask);
  wr = usb_pcm_wr;
  level = usb_pcm_level;
  AUDIO_UnlockIRQ(primask);

  uint32_t copy1 = size;
  if (wr + copy1 > cap)
  {
    copy1 = cap - wr;
  }

  (void)memcpy(&usb_pcm_ring[wr], pbuf, copy1);
  if (size > copy1)
  {
    (void)memcpy(&usb_pcm_ring[0], &pbuf[copy1], size - copy1);
  }

  uint32_t new_wr = wr;
  AUDIO_AdvanceRingPos(cap, &new_wr, size);

  uint32_t new_level = level + size;
  new_level = AUDIO_ClampU32(new_level, cap);

  /* 若溢出，丢弃最老数据（推进 rd） */
  if (level + size > cap)
  {
    uint32_t overflow = (level + size) - cap;
    uint32_t rd;
    AUDIO_LockIRQ(&primask);
    rd = usb_pcm_rd;
    AUDIO_UnlockIRQ(primask);

    AUDIO_AdvanceRingPos(cap, &rd, overflow);

    AUDIO_LockIRQ(&primask);
    usb_pcm_rd = rd;
    AUDIO_UnlockIRQ(primask);
  }

  AUDIO_LockIRQ(&primask);
  usb_pcm_wr = new_wr;
  usb_pcm_level = new_level;
  AUDIO_UnlockIRQ(primask);

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
  /* DMA 刚播放完后半区，后半区安全可写 */
  if (sai_started != 0U)
  {
    AUDIO_FillHalfTxBuffer_FS(SAI_TX_WORDS_HALF);
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
  /* DMA 刚播放完前半区，前半区安全可写 */
  if (sai_started != 0U)
  {
    AUDIO_FillHalfTxBuffer_FS(0U);
  }
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

static void AUDIO_StartSaiIfNeeded_FS(void)
{
  uint32_t primask;
  uint32_t level;

  if (sai_started != 0U)
  {
    return;
  }

  if (HAL_SAI_GetState(&hsai_BlockA1) != HAL_SAI_STATE_READY)
  {
    return;
  }

  AUDIO_LockIRQ(&primask);
  level = usb_pcm_level;
  AUDIO_UnlockIRQ(primask);

  /* 至少积累半缓冲数据后再开播，降低启动即欠载导致长时间静音 */
  if (level < AUDIO_START_LEVEL_BYTES)
  {
    return;
  }

  /* 启动前先预填两半区，避免开播首周期完全静音 */
  (void)memset((void *)sai_tx_buf, 0, sizeof(sai_tx_buf));
  AUDIO_FillHalfTxBuffer_FS(0U);
  AUDIO_FillHalfTxBuffer_FS(SAI_TX_WORDS_HALF);

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_CleanDCache_by_Addr((uint32_t *)sai_tx_buf, (int32_t)sizeof(sai_tx_buf));
#endif

  if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)sai_tx_buf, (uint16_t)SAI_TX_WORDS_TOTAL) == HAL_OK)
  {
    sai_started = 1U;
  }
}

static void AUDIO_FillHalfTxBuffer_FS(uint32_t dst_word_offset)
{
  const uint32_t cap = (uint32_t)sizeof(usb_pcm_ring);
  uint32_t rd;
  uint32_t level;
  uint32_t primask;

  AUDIO_LockIRQ(&primask);
  rd = usb_pcm_rd;
  level = usb_pcm_level;
  AUDIO_UnlockIRQ(primask);

  /* 水位判定：接近空则插1帧(少消耗)，接近满则丢1帧(多消耗) */
  int32_t slip = 0;
  if (level < (uint32_t)AUDIO_OUT_PACKET)
  {
    slip = -1;
  }
  else if (level > (cap - (uint32_t)AUDIO_OUT_PACKET))
  {
    slip = 1;
  }

  const uint32_t slip_index = SAI_TX_FRAMES_HALF / 2U;
  int32_t *dst = &sai_tx_buf[dst_word_offset];
  int32_t last_l = 0;
  int32_t last_r = 0;

  for (uint32_t i = 0; i < SAI_TX_FRAMES_HALF; i++)
  {
    /* 插帧：在中点重复上一帧（不消耗 FIFO） */
    if ((slip < 0) && (i == slip_index) && (i != 0U))
    {
      *dst++ = last_l;
      *dst++ = last_r;
      continue;
    }

    if (level < PCM_BYTES_PER_FRAME)
    {
      /* 欠载：填静音 */
      last_l = 0;
      last_r = 0;
      *dst++ = 0;
      *dst++ = 0;
      continue;
    }

    uint16_t l16 = AUDIO_ReadU16LERing(usb_pcm_ring, cap, rd);
    uint32_t rd_r = rd;
    AUDIO_AdvanceRingPos(cap, &rd_r, 2U);
    uint16_t r16 = AUDIO_ReadU16LERing(usb_pcm_ring, cap, rd_r);

    last_l = AUDIO_PCM16_To_SAI32((int16_t)l16);
    last_r = AUDIO_PCM16_To_SAI32((int16_t)r16);
    *dst++ = last_l;
    *dst++ = last_r;

    AUDIO_AdvanceRingPos(cap, &rd, PCM_BYTES_PER_FRAME);
    level -= PCM_BYTES_PER_FRAME;

    /* 丢帧：在中点额外消耗 1 帧（不输出） */
    if ((slip > 0) && (i == slip_index) && (level >= PCM_BYTES_PER_FRAME))
    {
      AUDIO_AdvanceRingPos(cap, &rd, PCM_BYTES_PER_FRAME);
      level -= PCM_BYTES_PER_FRAME;
    }
  }

  AUDIO_LockIRQ(&primask);
  usb_pcm_rd = rd;
  usb_pcm_level = level;
  AUDIO_UnlockIRQ(primask);

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  /* 若启用DCache，保证 DMA 可见最新数据 */
  SCB_CleanDCache_by_Addr((uint32_t *)&sai_tx_buf[dst_word_offset], (int32_t)(SAI_TX_WORDS_HALF * sizeof(int32_t)));
#endif
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
 * @}
 */

/**
 * @}
 */

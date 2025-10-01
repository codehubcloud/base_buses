#ifndef DMA_H
#define DMA_H

#include <stdint.h>

/* DMA Transfer Direction */
typedef enum {
    DMA_DIR_MEM2MEM = 0,      /* Memory to memory */
    DMA_DIR_PERIPH2MEM = 1,   /* Peripheral to memory */
    DMA_DIR_MEM2PERIPH = 2,   /* Memory to peripheral */
    DMA_DIR_PERIPH2PERIPH = 3 /* Peripheral to peripheral */
} DmaDirection;

/* DMA Data Width */
typedef enum {
    DMA_DATA_WIDTH_BYTE = 0,     /* 8-bit data width */
    DMA_DATA_WIDTH_HALFWORD = 1, /* 16-bit data width */
    DMA_DATA_WIDTH_WORD = 2      /* 32-bit data width */
} DmaDataWidth;

/* DMA Priority Level */
typedef enum {
    DMA_PRIORITY_LOW = 0,       /* Low priority */
    DMA_PRIORITY_MEDIUM = 1,    /* Medium priority */
    DMA_PRIORITY_HIGH = 2,      /* High priority */
    DMA_PRIORITY_VERY_HIGH = 3  /* Very high priority */
} DmaPriority;

/* DMA Transfer Mode */
typedef enum {
    DMA_MODE_NORMAL = 0,   /* Normal mode - transfer once */
    DMA_MODE_CIRCULAR = 1  /* Circular mode - auto-reload */
} DmaMode;

/* DMA Interrupt Flags */
typedef enum {
    DMA_INT_TC = 0x01,  /* Transfer complete interrupt */
    DMA_INT_HT = 0x02,  /* Half transfer interrupt */
    DMA_INT_TE = 0x04,  /* Transfer error interrupt */
    DMA_INT_ALL = 0x07  /* All interrupts */
} DmaInterruptFlag;

/* DMA Transfer Status */
typedef enum {
    DMA_STATUS_IDLE = 0,        /* DMA idle */
    DMA_STATUS_IN_PROGRESS = 1, /* Transfer in progress */
    DMA_STATUS_COMPLETE = 2,    /* Transfer complete */
    DMA_STATUS_ERROR = 3,       /* Transfer error */
    DMA_STATUS_HALF_COMPLETE = 4 /* Half transfer complete */
} DmaStatus;

/* DMA Configuration Structure */
typedef struct {
    uint8_t controller;           /* DMA controller number (1 or 2) */
    uint8_t channel;              /* DMA channel/stream number */
    DmaDirection direction;       /* Transfer direction */
    DmaDataWidth dataWidth;       /* Source/destination data width */
    DmaPriority priority;         /* Channel priority level */
    DmaMode mode;                 /* Normal or circular mode */
    uint8_t memoryIncrement;      /* Enable memory address increment (1=enable, 0=disable) */
    uint8_t peripheralIncrement;  /* Enable peripheral address increment (1=enable, 0=disable) */
    uint8_t enableFifo;           /* Enable FIFO mode (1=enable, 0=disable) */
    uint8_t reserved;             /* Reserved for alignment */
} DmaConfig;

/******************************************************************************
 * @brief     : Initialize DMA controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables DMA clock and initializes hardware resources
 *****************************************************************************/
int32_t DmaInit(void);

/******************************************************************************
 * @brief     : Deinitialize DMA controller
 * @param[in] : None
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables DMA clock and releases hardware resources
 *****************************************************************************/
int32_t DmaDeinit(void);

/******************************************************************************
 * @brief     : Configure DMA channel with specified parameters
 * @param[in] : config - Pointer to DMA configuration structure
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called before starting DMA transfer
 *****************************************************************************/
int32_t DmaConfigureChannel(const DmaConfig* config);

/******************************************************************************
 * @brief     : Start DMA transfer
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number, srcAddr - Source address, destAddr - Destination address, dataLength - Number of data items to transfer
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Transfer parameters must be properly aligned
 *****************************************************************************/
int32_t DmaStartTransfer(uint8_t controller, uint8_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t dataLength);

/******************************************************************************
 * @brief     : Stop DMA transfer
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Stops ongoing transfer immediately
 *****************************************************************************/
int32_t DmaStopTransfer(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Get DMA transfer status
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number
 * @param[out]: None
 * @return    : DmaStatus value, or DMA_STATUS_ERROR if invalid parameters
 * @note      : Returns current transfer state
 *****************************************************************************/
DmaStatus DmaGetStatus(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Wait for DMA transfer completion
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number, timeoutMs - Timeout in milliseconds (0 = wait forever)
 * @param[out]: None
 * @return    : 0 if success, -1 if timeout or error
 * @note      : Blocking function that polls transfer status
 *****************************************************************************/
int32_t DmaWaitComplete(uint8_t controller, uint8_t channel, uint32_t timeoutMs);

/******************************************************************************
 * @brief     : Enable DMA interrupts
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number, interruptFlags - Interrupt flags to enable (bitwise OR of DmaInterruptFlag)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Enables specified interrupt types and configures NVIC
 *****************************************************************************/
int32_t DmaEnableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags);

/******************************************************************************
 * @brief     : Disable DMA interrupts
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number, interruptFlags - Interrupt flags to disable (bitwise OR of DmaInterruptFlag)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Disables specified interrupt types
 *****************************************************************************/
int32_t DmaDisableInterrupt(uint8_t controller, uint8_t channel, uint8_t interruptFlags);

/******************************************************************************
 * @brief     : Perform memory-to-memory copy using DMA
 * @param[in] : srcAddr - Source memory address, destAddr - Destination memory address, dataLength - Number of bytes to copy
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : High-performance alternative to memcpy for large data
 *****************************************************************************/
int32_t DmaMemCopy(uint32_t srcAddr, uint32_t destAddr, uint32_t dataLength);

/******************************************************************************
 * @brief     : Configure circular buffer mode
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number, srcAddr - Source address, destAddr - Destination address, bufferSize - Size of circular buffer
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Automatically restarts transfer after completion
 *****************************************************************************/
int32_t DmaCircularMode(uint8_t controller, uint8_t channel, uint32_t srcAddr, uint32_t destAddr, uint32_t bufferSize);

/******************************************************************************
 * @brief     : Get remaining data count in current transfer
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number
 * @param[out]: None
 * @return    : Remaining data count, or 0 if error
 * @note      : Useful for monitoring transfer progress
 *****************************************************************************/
uint32_t DmaGetRemainingCount(uint8_t controller, uint8_t channel);

/******************************************************************************
 * @brief     : Clear DMA interrupt flags
 * @param[in] : controller - DMA controller number (1 or 2), channel - DMA channel/stream number, interruptFlags - Interrupt flags to clear (bitwise OR of DmaInterruptFlag)
 * @param[out]: None
 * @return    : 0 if success, -1 if error
 * @note      : Must be called in interrupt handler to clear flags
 *****************************************************************************/
int32_t DmaClearInterruptFlags(uint8_t controller, uint8_t channel, uint8_t interruptFlags);

#endif // DMA_H

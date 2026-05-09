/**
 * @file    sts3215_hal.h
 * @brief   STS3215 — STM32 HAL layer (DMA TX/RX + RS485 + IDLE line)
 */

#ifndef STS3215_HAL_H
#define STS3215_HAL_H

#include "stm32g4xx_hal.h"
#include "sts3215_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Compile-time configuration
 * ========================================================================= */

/**
 * Timeout applied while waiting for reply after TX complete.
 * Override in your build flags if needed.
 */
#ifndef STS3215_HAL_REPLY_TIMEOUT_MS
#define STS3215_HAL_REPLY_TIMEOUT_MS   (10U)
#endif

/**
 * RX DMA buffer: large enough for STS3215_SYNC_MAX_SERVOS full reply frames.
 * 16 servos × 16 bytes/frame = 256 bytes.
 */
#define STS3215_HAL_RX_BUF_SIZE (STS3215_SYNC_MAX_SERVOS * STS3215_RX_BUF_SIZE)  

/* =========================================================================
 * Types
 * ========================================================================= */

/**
 * @brief Internal state machine states.
 */
typedef enum {
	STS3215_HAL_STATE_IDLE = 0,
	STS3215_HAL_STATE_TX_BUSY,
	STS3215_HAL_STATE_RX_BUSY,
	STS3215_HAL_STATE_ERROR,
} STS3215_HAL_State_t;

/** Error sources, reported via on_error callback. */
typedef enum {
	STS3215_HAL_ERR_NONE = 0,
	STS3215_HAL_ERR_TIMEOUT,
	STS3215_HAL_ERR_DMA_TX,
	STS3215_HAL_ERR_DMA_RX,
	STS3215_HAL_ERR_PARSE, /* STS3215_ParseReply returned error */
	STS3215_HAL_ERR_BUSY, /* SendFrame called while not IDLE */
} STS3215_HAL_Error_t;

/**
 * @brief Reply callback
 *
 * @param reply      Parsed reply
 * @param reply_idx  0-based index
 * @param status     STS3215_OK or STS3215_ERR_SERVO_FAULT
 * @param user_ctx   Opaque pointer from STS3215_HAL_Init()
 */
typedef void (*STS3215_HAL_ReplyCallback_t)(const STS3215_Reply_t *reply, uint8_t reply_idx, STS3215_Status_t status, void *user_ctx);

/**
 * @brief Error callback
 *
 * @param err       Error source
 * @param user_ctx  Opaque pointer from STS3215_HAL_Init()
 */
typedef void (*STS3215_HAL_ErrorCallback_t)(STS3215_HAL_Error_t err, void *user_ctx);

/* =========================================================================
 * Handle
 * ========================================================================= */

/**
 * @brief STS3215 HAL driver handle.
 *
 * One instance per physical bus. Declare as a global or static variable.
 * Initialise with STS3215_HAL_Init() before any other call.
 */
typedef struct {
	UART_HandleTypeDef *huart;

	/* Buffers — must remain valid for the full DMA transfer lifetime */
	__attribute__((aligned(4))) uint8_t tx_buf[STS3215_TX_BUF_SIZE];
	__attribute__((aligned(4))) uint8_t rx_buf[STS3215_HAL_RX_BUF_SIZE];

	/* State machine */
	volatile STS3215_HAL_State_t state;
	volatile STS3215_HAL_Error_t last_error;
	volatile uint16_t rx_received_len;

	/* Transfer metadata */
	bool is_broadcast;
	uint8_t expected_replies;

	uint32_t tx_timestamp_ms;
	uint32_t reply_timeout_ms;

	STS3215_HAL_ReplyCallback_t on_reply;
	STS3215_HAL_ErrorCallback_t on_error;
	void                       *user_ctx;
} STS3215_HAL_Handle_t;

/* =========================================================================
 * API
 * ========================================================================= */

/**
 * @brief Initialise the HAL handle. Call once before any other function.
 *
 * @param hservo           Handle to initialise
 * @param huart            Pointer to the CubeMX-generated UART handle
 * @param reply_timeout_ms Milliseconds to wait for reply before timeout error
 * @param on_reply         Called for each successfully received reply frame
 * @param on_error         Called on timeout, DMA error, or parse failure
 * @param user_ctx         Forwarded as-is to callbacks (NULL if unused)
 */
void STS3215_HAL_Init(STS3215_HAL_Handle_t       *hservo,
		UART_HandleTypeDef          *huart,
		uint32_t                    reply_timeout_ms,
		STS3215_HAL_ReplyCallback_t  on_reply,
		STS3215_HAL_ErrorCallback_t  on_error,
		void                        *user_ctx);

/**
 * @brief Register the instance that weak HAL callbacks route to.
 *
 * Call once after STS3215_HAL_Init(). Required if STS3215_HAL_NO_WEAK_OVERRIDE is not defined
 *
 * @param hservo  Fully initialised handle
 */
void STS3215_HAL_RegisterInstance(STS3215_HAL_Handle_t *hservo);

/**
 * @brief Send a pre-built packet frame over DMA.
 *
 * Non-blocking. Returns immediately after starting DMA.
 * The frame bytes are copied into the internal tx_buf before DMA starts,
 * so the caller's buffer can be reused or freed immediately after return.
 *
 * @param hservo           Driver handle
 * @param frame            Packet bytes built by protocol layer
 * @param len              Frame length in bytes
 * @param is_broadcast     True -> no reply expected (broadcast or write-only)
 * @param expected_replies Number of reply frames to wait for
 */
STS3215_Status_t STS3215_HAL_SendFrame(STS3215_HAL_Handle_t *hservo,
		const uint8_t        *frame,
		uint16_t              len,
		bool                  is_broadcast,
		uint8_t               expected_replies);

/**
 * @brief Periodic tick — call from main loop or RTOS task (not from IRQ).
 */
void STS3215_HAL_Process(STS3215_HAL_Handle_t *hservo);

/**
 * @brief Abort any in-progress transfer and reset to IDLE.
 */
void STS3215_HAL_Abort(STS3215_HAL_Handle_t *hservo);

/** 
 * @brief Return current state machine state. 
 */
STS3215_HAL_State_t STS3215_HAL_GetState(const STS3215_HAL_Handle_t *hservo);

/** 
 * @brief Return true only when the driver is idle and ready for a new frame.
 *  */
bool STS3215_HAL_IsIdle(const STS3215_HAL_Handle_t *hservo);

/* =========================================================================
 * IRQ-context entry points
 * ========================================================================= */

/** Route from HAL_UART_TxCpltCallback */
void STS3215_HAL_TxCpltCallback(STS3215_HAL_Handle_t *hservo);

/** Route from HAL_UARTEx_RxEventCallback */
void STS3215_HAL_RxEventCallback(STS3215_HAL_Handle_t *hservo, uint16_t size);

/** Route from HAL_UART_ErrorCallback */
void STS3215_HAL_ErrorCallback(STS3215_HAL_Handle_t *hservo);

#ifdef __cplusplus
}
#endif

#endif /* STS3215_HAL_H */

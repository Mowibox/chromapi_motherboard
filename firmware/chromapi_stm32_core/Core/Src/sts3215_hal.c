/**
 * @file    sts3215_hal.c
 * @brief   STS3215 — STM32 HAL layer implementation
 *
 * State machine summary:
 *
 *  [IDLE]
 *    │  SendFrame()
 *    │  ├─ copy frame → tx_buf
 *    │  └─ HAL_UART_Transmit_DMA()
 *    ▼
 *  [TX_BUSY]
 *    │  HAL_UART_TxCpltCallback → STS3215_HAL_TxCpltCallback()
 *    │  ├─ is_broadcast == true  → state = IDLE (done)
 *    │  └─ is_broadcast == false
 *    │       ├─ flush UART DR register (clear RX noise from TX phase)
 *    │       └─ HAL_UARTEx_ReceiveToIdle_DMA(rx_buf, RX_BUF_SIZE)
 *    ▼
 *  [RX_BUSY]
 *    │  tx_timestamp_ms set for timeout guard in Process()
 *    │
 *    ├─ HAL_UARTEx_RxEventCallback(size) → STS3215_HAL_RxEventCallback()
 *    │    ├─ for each expected reply: STS3215_ParseReply() → on_reply()
 *    │    └─ state = IDLE
 *    │
 *    └─ Process(): HAL_GetTick() > tx_timestamp_ms + reply_timeout_ms
 *         ├─ HAL_UART_DMAStop()
 *         ├─ on_error(STS3215_HAL_ERR_TIMEOUT)
 *         └─ state = IDLE
 *
 *  [ERROR]  (DMA error path)
 *    │  STS3215_HAL_ErrorCallback() → on_error() → state = IDLE
 */

#include "sts3215_hal.h"
#include <string.h>

/* =========================================================================
 * Static instance registry (single-bus architecture)
 * ========================================================================= */

/**
 * Single registered instance, used by the overridden weak HAL callbacks.
 * Extend to an array if you need multiple buses.
 */
static STS3215_HAL_Handle_t *s_instance = NULL;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
 * @brief Parse all expected reply frames from the RX buffer.
 * Called from IRQ context (RxEventCallback).
 */
static void prv_parse_rx_buffer(STS3215_HAL_Handle_t *hservo, uint16_t buf_len)
{
	uint16_t offset  = 0U;
	uint8_t  parsed  = 0U;

	while ((parsed < hservo->expected_replies) && (offset < buf_len)) {

		uint16_t remaining = buf_len - offset;
		STS3215_Reply_t reply;

		STS3215_Status_t status = STS3215_ParseReply(
				&hservo->rx_buf[offset],
				remaining,
				&reply
		);

		if (status == STS3215_OK || status == STS3215_ERR_SERVO_FAULT) {
			if (hservo->on_reply != NULL) {
				hservo->on_reply(&reply, parsed, status, hservo->user_ctx);
			}

			offset += (uint16_t)(6U + (uint16_t)reply.data_len);
			parsed++;

		} else {
			if (hservo->on_error != NULL) {
				hservo->on_error(STS3215_HAL_ERR_PARSE, hservo->user_ctx);
			}
			break;
		}
	}
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void STS3215_HAL_Init(STS3215_HAL_Handle_t *hservo,
		UART_HandleTypeDef *huart,
		uint32_t reply_timeout_ms,
		STS3215_HAL_ReplyCallback_t  on_reply,
		STS3215_HAL_ErrorCallback_t  on_error,
		void *user_ctx)
{
	if (hservo == NULL || huart == NULL) { return; }

	memset(hservo, 0, sizeof(STS3215_HAL_Handle_t));

	hservo->huart = huart;
	hservo->reply_timeout_ms = (reply_timeout_ms > 0U)
                                				 ? reply_timeout_ms
                                						 : STS3215_HAL_REPLY_TIMEOUT_MS;
	hservo->on_reply = on_reply;
	hservo->on_error  = on_error;
	hservo->user_ctx = user_ctx;
	hservo->state = STS3215_HAL_STATE_IDLE;
	hservo->last_error  = STS3215_HAL_ERR_NONE;
}

void STS3215_HAL_RegisterInstance(STS3215_HAL_Handle_t *hservo)
{
	s_instance = hservo;
}

STS3215_Status_t STS3215_HAL_SendFrame(STS3215_HAL_Handle_t *hservo,
		const uint8_t *frame,
		uint16_t len,
		bool is_broadcast,
		uint8_t expected_replies)
{
	if (hservo == NULL || frame == NULL) { return STS3215_ERR_NULL_PTR; }

	if (hservo->state != STS3215_HAL_STATE_IDLE) {
		if (hservo->on_error != NULL) {
			hservo->on_error(STS3215_HAL_ERR_BUSY, hservo->user_ctx);
		}
		return STS3215_ERR_INVALID_PARAM;
	}

	if (len > (uint16_t)STS3215_TX_BUF_SIZE) {
		return STS3215_ERR_INVALID_PARAM;
	}

	memcpy(hservo->tx_buf, frame, len);

	hservo->is_broadcast = is_broadcast;
	hservo->expected_replies  = (is_broadcast ? 0U : expected_replies);
	hservo->state = STS3215_HAL_STATE_TX_BUSY;

	// [CORRECTION CRITIQUE : Armement anticipé du DMA RX]
	// Puisque le buffer U8 coupe l'écho et que le Pull-Up assure le silence,
	// on prépare le terrain de réception AVANT de tirer la trame.
	if (!is_broadcast && expected_replies > 0) {
		memset(hservo->rx_buf, 0, STS3215_HAL_RX_BUF_SIZE);
		hservo->rx_received_len = 0U;
		__HAL_UART_FLUSH_DRREGISTER(hservo->huart);
		__HAL_UART_CLEAR_OREFLAG(hservo->huart);

		HAL_UARTEx_ReceiveToIdle_DMA(hservo->huart, hservo->rx_buf, STS3215_HAL_RX_BUF_SIZE);
	}

	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
			hservo->huart,
			hservo->tx_buf,
			len
	);

	if (status != HAL_OK) {
		if (!is_broadcast && expected_replies > 0) {
			HAL_UART_DMAStop(hservo->huart); // Annuler le RX si le TX échoue
		}
		hservo->state = STS3215_HAL_STATE_ERROR;
		hservo->last_error = STS3215_HAL_ERR_DMA_TX;
		if (hservo->on_error != NULL) {
			hservo->on_error(STS3215_HAL_ERR_DMA_TX, hservo->user_ctx);
		}
		hservo->state = STS3215_HAL_STATE_IDLE;
		return STS3215_ERR_INVALID_PARAM;
	}

	return STS3215_OK;
}

void STS3215_HAL_Process(STS3215_HAL_Handle_t *hservo)
{
	if (hservo == NULL) { return; }
	if (hservo->state == STS3215_HAL_STATE_RX_BUSY) {
		uint32_t elapsed = HAL_GetTick() - hservo->tx_timestamp_ms;
		if (elapsed >= hservo->reply_timeout_ms) {
			/* Stop the RX DMA cleanly */
			HAL_UART_DMAStop(hservo->huart);
			__HAL_UART_FLUSH_DRREGISTER(hservo->huart);

			hservo->last_error = STS3215_HAL_ERR_TIMEOUT;
			hservo->state      = STS3215_HAL_STATE_IDLE;

			if (hservo->on_error != NULL) {
				hservo->on_error(STS3215_HAL_ERR_TIMEOUT, hservo->user_ctx);
			}
		}
	}
}

void STS3215_HAL_Abort(STS3215_HAL_Handle_t *hservo)
{
	if (hservo == NULL) { return; }

	HAL_UART_DMAStop(hservo->huart);
	__HAL_UART_FLUSH_DRREGISTER(hservo->huart);

	hservo->state      = STS3215_HAL_STATE_IDLE;
	hservo->last_error = STS3215_HAL_ERR_NONE;
}

STS3215_HAL_State_t STS3215_HAL_GetState(const STS3215_HAL_Handle_t *hservo)
{
	return (hservo != NULL) ? hservo->state : STS3215_HAL_STATE_ERROR;
}

bool STS3215_HAL_IsIdle(const STS3215_HAL_Handle_t *hservo)
{
	return (hservo != NULL) && (hservo->state == STS3215_HAL_STATE_IDLE);
}

/* =========================================================================
 * IRQ-context entry points
 * ========================================================================= */

/**
 * @brief Called from HAL_UART_TxCpltCallback when DMA TX completes.
 * Runs in DMA IRQ context — keep processing minimal.
 */
void STS3215_HAL_TxCpltCallback(STS3215_HAL_Handle_t *hservo)
{
	if (hservo == NULL || hservo->state != STS3215_HAL_STATE_TX_BUSY) {
		return;
	}

	if (hservo->is_broadcast || hservo->expected_replies == 0U) {
		hservo->state = STS3215_HAL_STATE_IDLE;
	} else {
		// [CORRECTION CRITIQUE] Le DMA RX est déjà actif.
		// On bascule simplement la machine à états et on arme le Timeout.
		hservo->state = STS3215_HAL_STATE_RX_BUSY;
		hservo->tx_timestamp_ms = HAL_GetTick();
	}
}

/**
 * @brief Called from HAL_UARTEx_RxEventCallback on IDLE or buffer-full event.
 * Runs in DMA IRQ context.
 *
 * @param size  Actual number of bytes received into rx_buf.
 */
void STS3215_HAL_RxEventCallback(STS3215_HAL_Handle_t *hservo, uint16_t size)
{
	if (hservo == NULL || hservo->state != STS3215_HAL_STATE_RX_BUSY) {
		return;
	}

	hservo->rx_received_len = size;

	prv_parse_rx_buffer(hservo, size);

	hservo->state = STS3215_HAL_STATE_IDLE;
}

/**
 * @brief Called from HAL_UART_ErrorCallback on DMA or UART peripheral error.
 * Runs in DMA/UART IRQ context.
 */
void STS3215_HAL_ErrorCallback(STS3215_HAL_Handle_t *hservo)
{
	if (hservo == NULL) { return; }

	HAL_UART_DMAStop(hservo->huart);
	__HAL_UART_FLUSH_DRREGISTER(hservo->huart);

	STS3215_HAL_Error_t err = (hservo->state == STS3215_HAL_STATE_TX_BUSY)
                            				   ? STS3215_HAL_ERR_DMA_TX
                            						   : STS3215_HAL_ERR_DMA_RX;

	hservo->last_error = err;
	hservo->state      = STS3215_HAL_STATE_IDLE;

	if (hservo->on_error != NULL) {
		hservo->on_error(err, hservo->user_ctx);
	}
}

/* =========================================================================
 * Weak HAL callback overrides
 *
 * These override the __weak symbols defined in stm32xxxx_hal_uart.c.
 * If STS3215_HAL_NO_WEAK_OVERRIDE is defined in build flags, these
 * symbols are omitted and the user must call the entry points manually.
 * ========================================================================= */

#ifndef STS3215_HAL_NO_WEAK_OVERRIDE

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((s_instance != NULL) && (s_instance->huart == huart)) {
		STS3215_HAL_TxCpltCallback(s_instance);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if ((s_instance != NULL) && (s_instance->huart == huart)) {
		STS3215_HAL_RxEventCallback(s_instance, size);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if ((s_instance != NULL) && (s_instance->huart == huart)) {
		STS3215_HAL_ErrorCallback(s_instance);
	}
}

#endif /* STS3215_HAL_NO_WEAK_OVERRIDE */

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
#include <string.h>  /* memcpy */

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
 * @brief Reset RX buffer and start DMA reception with IDLE line detection.
 *
 * HAL_UARTEx_ReceiveToIdle_DMA enables the IDLE interrupt internally.
 * It fires HAL_UARTEx_RxEventCallback when:
 *  a) The RX buffer is full  (HAL_UART_RXEVENT_TC)
 *  b) UART IDLE line detected (HAL_UART_RXEVENT_IDLE) ← our main trigger
 *
 * Called exclusively from STS3215_HAL_TxCpltCallback (IRQ context).
 */
static void prv_start_rx(STS3215_HAL_Handle_t *hservo)
{
    memset(hservo->rx_buf, 0, STS3215_HAL_RX_BUF_SIZE);
    hservo->rx_received_len = 0U;

    /*
     * Flush UART data register before enabling RX.
     * During the TX phase the UART RX pin may have captured bus reflections
     * or noise. Clear the overrun flag and DR to avoid a spurious RxEvent
     * firing immediately.
     */
    __HAL_UART_FLUSH_DRREGISTER(hservo->huart);
    __HAL_UART_CLEAR_OREFLAG(hservo->huart);

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
        hservo->huart,
        hservo->rx_buf,
        STS3215_HAL_RX_BUF_SIZE
    );

    if (status != HAL_OK) {
        /* DMA RX failed to start — report immediately */
        hservo->last_error = STS3215_HAL_ERR_DMA_RX;
        hservo->state      = STS3215_HAL_STATE_ERROR;
        if (hservo->on_error != NULL) {
            hservo->on_error(STS3215_HAL_ERR_DMA_RX, hservo->user_ctx);
        }
        hservo->state = STS3215_HAL_STATE_IDLE;
    } else {
        hservo->state            = STS3215_HAL_STATE_RX_BUSY;
        hservo->tx_timestamp_ms  = HAL_GetTick(); /* Start timeout clock */
    }
}

/**
 * @brief Parse all expected reply frames from the RX buffer.
 *
 * The STS3215 always replies with fixed-format frames. For SYNC READ,
 * multiple frames are concatenated in the buffer (no gaps if return delay = 0).
 * We advance through the buffer frame-by-frame using the LENGTH field.
 *
 * Frame byte count from buffer = 4 (header+id+len) + length_field_value.
 *   length_field = n_data_bytes + 2 (error byte + checksum byte)
 *   total bytes  = 4 + n_data_bytes + 2 = 6 + n_data_bytes
 *
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
            /* Fire user callback with reply and its index in the batch */
            if (hservo->on_reply != NULL) {
                hservo->on_reply(&reply, parsed, status, hservo->user_ctx);
            }

            /*
             * Advance offset by the total frame size consumed.
             * Frame size = 6 (fixed overhead) + n_data_bytes
             * where n_data_bytes = reply.data_len
             */
            offset += (uint16_t)(6U + (uint16_t)reply.data_len);
            parsed++;

        } else {
            /*
             * Parse failure on a frame boundary (bad header, bad checksum,
             * or truncated frame). Fire error and stop — do not attempt to
             * re-sync; a clean retry from the application layer is safer.
             */
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

void STS3215_HAL_Init(STS3215_HAL_Handle_t       *hservo,
                      UART_HandleTypeDef          *huart,
                      uint32_t                    reply_timeout_ms,
                      STS3215_HAL_ReplyCallback_t  on_reply,
                      STS3215_HAL_ErrorCallback_t  on_error,
                      void                        *user_ctx)
{
    if (hservo == NULL || huart == NULL) { return; }

    memset(hservo, 0, sizeof(STS3215_HAL_Handle_t));

    hservo->huart             = huart;
    hservo->reply_timeout_ms  = (reply_timeout_ms > 0U)
                                 ? reply_timeout_ms
                                 : STS3215_HAL_REPLY_TIMEOUT_MS;
    hservo->on_reply          = on_reply;
    hservo->on_error          = on_error;
    hservo->user_ctx          = user_ctx;
    hservo->state             = STS3215_HAL_STATE_IDLE;
    hservo->last_error        = STS3215_HAL_ERR_NONE;
}

void STS3215_HAL_RegisterInstance(STS3215_HAL_Handle_t *hservo)
{
    s_instance = hservo;
}

STS3215_Status_t STS3215_HAL_SendFrame(STS3215_HAL_Handle_t *hservo,
                                        const uint8_t        *frame,
                                        uint16_t              len,
                                        bool                  is_broadcast,
                                        uint8_t               expected_replies)
{
    if (hservo == NULL || frame == NULL) { return STS3215_ERR_NULL_PTR; }

    if (hservo->state != STS3215_HAL_STATE_IDLE) {
        if (hservo->on_error != NULL) {
            hservo->on_error(STS3215_HAL_ERR_BUSY, hservo->user_ctx);
        }
        return STS3215_ERR_INVALID_PARAM; /* Reuse closest error code */
    }

    /* Guard: frame must fit in tx_buf */
    if (len > (uint16_t)STS3215_TX_BUF_SIZE) {
        return STS3215_ERR_INVALID_PARAM;
    }

    /*
     * Copy frame into DMA-backed tx_buf BEFORE starting DMA.
     * This decouples the caller's buffer lifetime from the DMA transfer.
     */
    memcpy(hservo->tx_buf, frame, len);

    hservo->is_broadcast      = is_broadcast;
    hservo->expected_replies  = (is_broadcast ? 0U : expected_replies);
    hservo->state             = STS3215_HAL_STATE_TX_BUSY;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(
        hservo->huart,
        hservo->tx_buf,
        len
    );

    if (status != HAL_OK) {
        hservo->state      = STS3215_HAL_STATE_ERROR;
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

    /*
     * Timeout guard: only relevant while waiting for a reply.
     * HAL_GetTick() returns ms since boot; handles 32-bit rollover correctly
     * via unsigned subtraction.
     */
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
 *
 * At this point the hardware RS485 DE deassertion timer has already
 * been armed (8 sample periods ≈ 8 µs at 1 Mbps). The DE pin will
 * deassert automatically before the next RX byte could arrive from the
 * servo (minimum return delay = 0 µs but the STS3215 needs ~50 µs to
 * prepare its response in practice).
 *
 * Runs in DMA IRQ context — keep processing minimal.
 */
void STS3215_HAL_TxCpltCallback(STS3215_HAL_Handle_t *hservo)
{
    if (hservo == NULL || hservo->state != STS3215_HAL_STATE_TX_BUSY) {
        return;
    }

    if (hservo->is_broadcast || hservo->expected_replies == 0U) {
        /* No reply expected — transfer complete */
        hservo->state = STS3215_HAL_STATE_IDLE;
    } else {
        /* Flip to receive: start DMA RX with IDLE line detection */
        prv_start_rx(hservo);
    }
}

/**
 * @brief Called from HAL_UARTEx_RxEventCallback on IDLE or buffer-full event.
 *
 * @param size  Actual number of bytes received into rx_buf.
 *
 * IDLE fires when the bus has been silent for 1 character time after
 * the last byte. For default return delay = 0, this happens ~10 µs after
 * the last reply stop bit at 1 Mbps.
 *
 * For SYNC READ, all N reply frames should arrive back-to-back before
 * the IDLE fires, and will be concatenated in rx_buf. prv_parse_rx_buffer()
 * walks them frame-by-frame.
 *
 * Runs in DMA IRQ context.
 */
void STS3215_HAL_RxEventCallback(STS3215_HAL_Handle_t *hservo, uint16_t size)
{
    if (hservo == NULL || hservo->state != STS3215_HAL_STATE_RX_BUSY) {
        return;
    }

    hservo->rx_received_len = size;

    /* Parse all received frames and fire on_reply for each */
    prv_parse_rx_buffer(hservo, size);

    /* Return to IDLE — ready for next command */
    hservo->state = STS3215_HAL_STATE_IDLE;
}

/**
 * @brief Called from HAL_UART_ErrorCallback on DMA or UART peripheral error.
 *
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
 * These override the __weak symbols defined in stm32g4xx_hal_uart.c.
 * Routing: check huart == registered instance before dispatching.
 *
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
/* SPDX-License-Identifier: Apache-2.0 */
#include "servo_bus.h"

#include "sts3215_hal.h"
#include "sts3215_protocol.h"

typedef enum {
	TXN_NONE = 0,
	TXN_POLL,     // the cycle's SyncRead
	TXN_COMMAND,  // SyncWrite of the pending targets/torque, or a configuration command
	TXN_RESCAN,   // ping of an absent servo, at the end of a bus cycle
} txn_t;

// One SyncRead covers Present_Position (0x38) .. Present_Current (0x45-0x46): position,
// speed, load, voltage, temperature and current in a single transaction every cycle.
#define POLL_REG  STS3215_REG_CURRENT_POS
#define POLL_LEN  (STS3215_REG_CURRENT_CURRENT + 2U - STS3215_REG_CURRENT_POS)  // 15 bytes
#define POLL_CURRENT_OFS (STS3215_REG_CURRENT_CURRENT - STS3215_REG_CURRENT_POS)
_Static_assert(POLL_LEN <= STS3215_REPLY_DATA_MAX, "servo reply buffer too small for the poll");
_Static_assert(SERVO_BUS_READ_MAX <= STS3215_REPLY_DATA_MAX, "servo reply buffer too small for a register read");

// Servos in the SyncRead (bit i = ID i+1). One that misses SERVO_MISS_LIMIT polls in a row leaves
// it, so a dropped servo costs 3 reply timeouts instead of one per cycle forever. Absent servos
// are pinged again, one per SERVO_RESCAN_MS, at the end of a cycle.
#define SERVO_MISS_LIMIT  3U
#define SERVO_RESCAN_MS   1000U
#define ALL_SERVOS_MASK   0x0FFFU

// A configuration command waits at most this long for the cycle's transaction to end.
#define BUS_CLAIM_TIMEOUT_MS  55U

static STS3215_HAL_Handle_t s_hservo;
static volatile txn_t s_txn = TXN_NONE;
static ServoBusState_t s_state;  // servo[] and fresh_mask written by on_reply() in the USART2 IRQ

static volatile bool s_cycle_due = false;  // start of a bus cycle: write targets, then read
static bool s_writes_due = false;          // pending writes, then the SyncRead, still to go this cycle
static bool s_cycle_done = false;          // a cycle completed, not yet reported by ServoBus_Process()
static uint32_t s_cycle_start_us = 0;

static uint16_t s_present_mask = 0;
static uint16_t s_polled_mask = 0;  // servos asked by the SyncRead in flight
static uint8_t  s_miss[SERVO_BUS_COUNT];
static uint8_t  s_rescan_id = 0;
static uint32_t s_rescan_last_ms = 0;

// Pending writes, dispatched at the start of the next cycle (positions first, then torque)
static bool     s_pending_positions_valid = false;
static uint8_t  s_pending_positions_raw[2U * SERVO_BUS_COUNT];
static uint16_t s_pending_torque_mask = 0;
static uint16_t s_pending_torque_on = 0;

// Reply to the last configuration command (on_reply() outside a poll or rescan)
static volatile bool    s_reply_received = false;
static volatile uint8_t s_reply_len = 0;
static volatile uint8_t s_reply_data[STS3215_REPLY_DATA_MAX];

volatile uint32_t g_servo_err_count[8] = {0};

static void on_reply(const STS3215_Reply_t *reply, uint8_t idx, STS3215_Status_t status, void *ctx) {
	(void)idx;
	(void)ctx;
	if (s_txn == TXN_RESCAN) {
		if (reply->id >= 1 && reply->id <= SERVO_BUS_COUNT) {
			s_present_mask |= (uint16_t)(1U << (reply->id - 1U));
			s_miss[reply->id - 1U] = 0;
		}
		return;
	}
	if (s_txn == TXN_POLL) {
		if (reply->id >= 1 && reply->id <= SERVO_BUS_COUNT &&
				(status == STS3215_OK || status == STS3215_ERR_SERVO_FAULT) &&
				reply->data_len >= POLL_LEN) {
			uint8_t i = reply->id - 1U;
			s_state.servo[i].position    = (uint16_t)STS3215_UnpackS16LE(&reply->data[0]);
			s_state.servo[i].speed       = STS3215_UnpackSignMag16LE(&reply->data[2], 15U);
			s_state.servo[i].load        = STS3215_UnpackSignMag16LE(&reply->data[4], 10U);
			s_state.servo[i].voltage     = reply->data[6];
			s_state.servo[i].temperature = reply->data[7];
			s_state.servo[i].current     = (int16_t)STS3215_UnpackU16LE(&reply->data[POLL_CURRENT_OFS]);
			s_state.fresh_mask |= (uint16_t)(1U << i);
		}
		return;
	}

	uint8_t copy_len = (reply->data_len <= STS3215_REPLY_DATA_MAX) ? reply->data_len : STS3215_REPLY_DATA_MAX;
	for (uint8_t i = 0; i < copy_len; i++) {
		s_reply_data[i] = reply->data[i];
	}
	s_reply_len = reply->data_len;
	s_reply_received = true;
}

static void on_error(STS3215_HAL_Error_t err, void *ctx)
{
	(void)ctx;
	if ((uint32_t)err < 8U) g_servo_err_count[err]++;
}

static bool bus_free(void)
{
	return (s_txn == TXN_NONE) && STS3215_HAL_IsIdle(&s_hservo);
}

/* =========================================================================
 * Bus cycle
 * ========================================================================= */

static void update_presence(void) {
	// Nobody answered: a bus-wide glitch, not 12 servos dropping at once. Keep them all.
	if (s_state.fresh_mask == 0U) return;
	for (uint8_t i = 0; i < SERVO_BUS_COUNT; i++) {
		if (!(s_polled_mask & (1U << i))) continue;
		if (s_state.fresh_mask & (1U << i)) {
			s_miss[i] = 0;
		} else if (++s_miss[i] >= SERVO_MISS_LIMIT) {
			s_present_mask &= (uint16_t)~(1U << i);
		}
	}
}

// Bookkeeping of a transaction the HAL has finished: a completed SyncRead ends the cycle.
static void finish_txn(void) {
	if (s_txn == TXN_NONE || !STS3215_HAL_IsIdle(&s_hservo)) return;
	txn_t done = s_txn;
	s_txn = TXN_NONE;
	if (done == TXN_POLL) {
		update_presence();
		s_cycle_done = true;
	}
}

// False when no SyncWrite was needed: the cycle can move on to the SyncRead.
static bool dispatch_pending_writes(void) {
	STS3215_SyncEntry_t entries[SERVO_BUS_COUNT];
	uint8_t n = 0;
	uint8_t reg, len;

	if (s_pending_positions_valid) {
		s_pending_positions_valid = false;
		for (uint8_t i = 0; i < SERVO_BUS_COUNT; i++, n++) {
			entries[n].id = i + 1;
			entries[n].data[0] = s_pending_positions_raw[i * 2];
			entries[n].data[1] = s_pending_positions_raw[i * 2 + 1];
		}
		reg = STS3215_REG_TARGET_POS;
		len = 2;
	} else if (s_pending_torque_mask != 0U) {
		for (uint8_t i = 0; i < SERVO_BUS_COUNT; i++) {
			if (s_pending_torque_mask & (1U << i)) {
				entries[n].id = i + 1;
				entries[n].data[0] = (uint8_t)((s_pending_torque_on >> i) & 1U);
				n++;
			}
		}
		s_pending_torque_mask = 0;
		reg = STS3215_REG_TORQUE_SWITCH;
		len = 1;
	} else {
		return false;
	}

	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildSyncWrite(buf, sizeof(buf), reg, len, entries, n);
	if (frame_len > 0) {
		s_txn = TXN_COMMAND;
		if (STS3215_HAL_SendFrame(&s_hservo, buf, (uint16_t)frame_len, true, 0) != STS3215_OK) {
			s_txn = TXN_NONE;
		}
	}
	return true;
}

// False when no SyncRead went out (no servo present, or bus refused): the cycle still ends,
// so the Pi keeps getting the IMU and power data.
static bool start_poll(void) {
	s_state.fresh_mask = 0;
	uint8_t ids[SERVO_BUS_COUNT];
	uint8_t n = 0;
	for (uint8_t i = 0; i < SERVO_BUS_COUNT; i++) {
		if (s_present_mask & (1U << i)) ids[n++] = i + 1U;
	}
	if (n == 0) return false;
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildSyncRead(buf, sizeof(buf), POLL_REG, POLL_LEN, ids, n);
	if (frame_len <= 0) return false;
	s_polled_mask = s_present_mask;
	s_txn = TXN_POLL;
	if (STS3215_HAL_SendFrame(&s_hservo, buf, (uint16_t)frame_len, false, n) != STS3215_OK) {
		s_txn = TXN_NONE;
		return false;
	}
	return true;
}

// Ping the next absent servo, at most once per SERVO_RESCAN_MS (non-blocking: on_reply re-adds it).
static void rescan_one_absent(void) {
	if (!bus_free() || s_present_mask == ALL_SERVOS_MASK ||
			(HAL_GetTick() - s_rescan_last_ms) < SERVO_RESCAN_MS) return;
	s_rescan_last_ms = HAL_GetTick();
	do {
		s_rescan_id = (uint8_t)(s_rescan_id % SERVO_BUS_COUNT) + 1U;
	} while (s_present_mask & (1U << (s_rescan_id - 1U)));

	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildPing(buf, sizeof(buf), s_rescan_id);
	if (frame_len <= 0) return;
	s_txn = TXN_RESCAN;
	if (STS3215_HAL_SendFrame(&s_hservo, buf, (uint16_t)frame_len, false, 1) != STS3215_OK) {
		s_txn = TXN_NONE;
	}
}

void ServoBus_TickCycle(void) {
	s_cycle_due = true;
}

bool ServoBus_Process(uint32_t now_us, uint16_t *cycle_us) {
	STS3215_HAL_Process(&s_hservo);
	finish_txn();

	// Fixed-rate bus cycle: SyncWrite the pending targets/torque (if any), then one SyncRead.
	if (s_cycle_due && bus_free()) {
		s_cycle_due = false;
		s_cycle_start_us = now_us;
		s_writes_due = true;
	} else if (s_writes_due && bus_free() && !dispatch_pending_writes()) {
		s_writes_due = false;
		if (!start_poll()) s_cycle_done = true;
	}

	if (!s_cycle_done) return false;
	s_cycle_done = false;
	uint32_t us = now_us - s_cycle_start_us;
	*cycle_us = (uint16_t)(us > 0xFFFFU ? 0xFFFFU : us);
	rescan_one_absent();
	return true;
}

const ServoBusState_t *ServoBus_State(void) {
	return &s_state;
}

void ServoBus_SetTargets(const uint8_t raw_le[2U * SERVO_BUS_COUNT]) {
	for (uint8_t i = 0; i < 2U * SERVO_BUS_COUNT; i++) {
		s_pending_positions_raw[i] = raw_le[i];
	}
	s_pending_positions_valid = true;
}

void ServoBus_SetTorque(uint16_t mask, uint16_t on) {
	mask &= ALL_SERVOS_MASK;
	s_pending_torque_on   = (uint16_t)((s_pending_torque_on & ~mask) | (on & mask));
	s_pending_torque_mask |= mask;
}

void ServoBus_ReleaseAll(void) {
	s_pending_positions_valid = false;
	s_pending_torque_mask = ALL_SERVOS_MASK;
	s_pending_torque_on = 0U;
}

/* =========================================================================
 * Blocking transactions: boot and configuration commands
 * ========================================================================= */

static bool wait_idle(uint32_t timeout_ms)
{
	uint32_t t0 = HAL_GetTick();
	while (!STS3215_HAL_IsIdle(&s_hservo)) {
		STS3215_HAL_Process(&s_hservo);
		if ((HAL_GetTick() - t0) > timeout_ms) {
			STS3215_HAL_Abort(&s_hservo);
			return false;
		}
	}
	return true;
}

static bool claim_bus(void)
{
	if (!wait_idle(BUS_CLAIM_TIMEOUT_MS)) return false;
	finish_txn();  // a SyncRead that just ended still updates presence and gets streamed
	s_txn = TXN_COMMAND;
	return true;
}

static void release_bus(void)
{
	s_txn = TXN_NONE;
}

static bool txn(const uint8_t *buf, int16_t frame_len, bool broadcast, uint8_t replies, uint32_t timeout_ms)
{
	if (frame_len <= 0) return false;
	s_hservo.last_error = STS3215_HAL_ERR_NONE;
	if (STS3215_HAL_SendFrame(&s_hservo, buf, (uint16_t)frame_len, broadcast, replies) != STS3215_OK) return false;
	return wait_idle(timeout_ms) && s_hservo.last_error == STS3215_HAL_ERR_NONE;
}

static bool ping(uint8_t id, uint32_t timeout_ms)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	s_reply_received = false;
	return txn(buf, STS3215_BuildPing(buf, sizeof(buf), id), false, 1, timeout_ms) && s_reply_received;
}

void ServoBus_Init(UART_HandleTypeDef *huart) {
	// 5 ms reply timeout (~4-5 ms with the 1 ms tick): a full 12-servo poll takes ~3 ms,
	// so a servo that dropped out costs one cycle instead of freezing the bus for 50 ms.
	STS3215_HAL_Init(&s_hservo, huart, 5U, on_reply, on_error, NULL);

	// The STS3215 keep their torque and last target while powered, even across an MCU reset
	// (e.g. by the IWDG): release them explicitly instead of assuming anything about their state.
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildDisableTorque(buf, sizeof(buf), STS3215_BROADCAST_ID);
	if (frame_len > 0 && STS3215_HAL_SendFrame(&s_hservo, buf, (uint16_t)frame_len, true, 0) == STS3215_OK) {
		while (!STS3215_HAL_IsIdle(&s_hservo)) {
			STS3215_HAL_Process(&s_hservo);
		}
	}
}

void ServoBus_ScanPresent(void) {
	s_present_mask = 0;
	for (uint8_t id = 1; id <= SERVO_BUS_COUNT; id++) {
		if (ping(id, 50U)) {
			s_present_mask |= (uint16_t)(1U << (id - 1U));
		}
	}
}

bool ServoBus_Ping(uint8_t id) {
	if (!claim_bus()) return false;
	bool ok = ping(id, 20U);
	release_bus();
	return ok;
}

bool ServoBus_SetId(uint8_t old_id, uint8_t new_id) {
	if (!claim_bus()) return false;

	uint8_t buf[STS3215_TX_BUF_SIZE];
	bool bcast = (old_id == STS3215_BROADCAST_ID);
	bool ok = txn(buf, STS3215_BuildUnlockEEPROM(buf, sizeof(buf), old_id), bcast, 0, 50) &&
			txn(buf, STS3215_BuildWrite1B(buf, sizeof(buf), old_id, STS3215_REG_ID, new_id), bcast, 0, 50) &&
			txn(buf, STS3215_BuildLockEEPROM(buf, sizeof(buf), STS3215_BROADCAST_ID), true, 0, 20);
	if (ok) {
		HAL_Delay(20);
		ok = ping(new_id, 30U);
	}
	release_bus();
	return ok;
}

bool ServoBus_WriteRegister(uint8_t id, uint8_t reg, uint8_t size, uint16_t value) {
	if (!claim_bus()) return false;

	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = 0;
	if (size == 1) {
		frame_len = STS3215_BuildWrite1B(buf, sizeof(buf), id, reg, (uint8_t)value);
	} else if (size == 2) {
		frame_len = STS3215_BuildWrite2B(buf, sizeof(buf), id, reg, value);
	}
	bool is_broadcast = (id == STS3215_BROADCAST_ID);
	bool ok = txn(buf, frame_len, is_broadcast, is_broadcast ? 0U : 1U, 50);
	release_bus();
	return ok;
}

bool ServoBus_ReadRegister(uint8_t id, uint8_t reg, uint8_t size, uint8_t *out) {
	if (size == 0 || size > SERVO_BUS_READ_MAX || !claim_bus()) return false;

	uint8_t buf[STS3215_TX_BUF_SIZE];
	s_reply_received = false;
	bool ok = txn(buf, STS3215_BuildRead(buf, sizeof(buf), id, reg, size), false, 1, 10) &&
			s_reply_received && s_reply_len >= size;
	if (ok) {
		for (uint8_t i = 0; i < size; i++) out[i] = s_reply_data[i];
	}
	release_bus();
	return ok;
}

/* =========================================================================
 * HAL UART callbacks for the servo UART (IRQ context)
 * ========================================================================= */

void ServoBus_UartTxCplt(void) {
	STS3215_HAL_TxCpltCallback(&s_hservo);
}

void ServoBus_UartRxEvent(uint16_t size) {
	STS3215_HAL_RxEventCallback(&s_hservo, size);
}

void ServoBus_UartError(void) {
	STS3215_HAL_ErrorCallback(&s_hservo);
}

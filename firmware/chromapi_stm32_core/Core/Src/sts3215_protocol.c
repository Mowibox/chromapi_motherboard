/**
 * @file    sts3215_protocol.c
 * @brief   Pure STS3215 C protocol implementation
 */

#include "sts3215_protocol.h"
#include <string.h>

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/**
 * @brief Compute Feetech checksum (formula in Protocol Manual 1.1).
 */
static uint8_t prv_checksum(uint8_t id, uint8_t length, uint8_t instr,
		const uint8_t *params, uint8_t n_params)
{
	uint32_t sum = (uint32_t)id + (uint32_t)length + (uint32_t)instr;
	for (uint8_t i = 0U; i < n_params; i++) {
		sum += (uint32_t)params[i];
	}
	return (uint8_t)(~sum & 0xFFU);
}

/**
 * @brief Assemble a complete instruction packet into buf.
 *
 * @return Total frame length in bytes.
 */
static uint16_t prv_pack_frame(uint8_t *buf, uint8_t id, uint8_t instr,
		const uint8_t *params, uint8_t n_params)
{
	uint8_t length   = n_params + 2U;          /* instr byte + checksum byte */
	uint8_t checksum = prv_checksum(id, length, instr, params, n_params);

	buf[0] = STS3215_HEADER_BYTE;
	buf[1] = STS3215_HEADER_BYTE;
	buf[2] = id;
	buf[3] = length;
	buf[4] = instr;

	if ((n_params > 0U) && (params != NULL)) {
		memcpy(&buf[5], params, n_params);
	}
	buf[5U + n_params] = checksum;

	return (uint16_t)(6U + n_params);
}

/**
 * @brief Verify caller buffer fits the required frame.
 */
static bool prv_cap_ok(uint16_t cap, uint8_t n_params)
{
	return cap >= (uint16_t)(6U + (uint16_t)n_params);
}

/* =========================================================================
 * Packet Builders — Single-instruction
 * ========================================================================= */

/**
 * PING — Query status (Protocol Manual 1.3.1)
 */
int16_t STS3215_BuildPing(uint8_t *buf, uint16_t cap, uint8_t id)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_PING, NULL, 0U);
}

/**
 * READ DATA (Protocol Manual 1.3.2)
 */
int16_t STS3215_BuildRead(uint8_t *buf, uint16_t cap,
		uint8_t id, uint8_t reg, uint8_t len)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (len == 0U)             { return STS3215_ERR_INVALID_PARAM; }
	if (!prv_cap_ok(cap, 2U)) { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[2] = { reg, len };
	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_READ_DATA, params, 2U);
}

/**
 * WRITE DATA — Write a byte to register (Protocol Manual 1.3.3)
 */
int16_t STS3215_BuildWrite1B(uint8_t *buf, uint16_t cap,
		uint8_t id, uint8_t reg, uint8_t val)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 2U)) { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[2] = { reg, val };
	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 2U);
}

/**
 * WRITE DATA — Write a 16-bit value to register 
 */
int16_t STS3215_BuildWrite2B(uint8_t *buf, uint16_t cap,
		uint8_t id, uint8_t reg, uint16_t val)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 3U)) { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[3] = {
			reg,
			(uint8_t)(val & 0xFFU),         /* low byte before  */
			(uint8_t)((val >> 8U) & 0xFFU)  /* high byte after */
	};
	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 3U);
}

/**
 * WRITE DATA — Write a raw byte array to register
 */
int16_t STS3215_BuildWriteRaw(uint8_t *buf, uint16_t cap,
		uint8_t id, uint8_t reg,
		const uint8_t *data, uint8_t data_len)
{
	if (buf == NULL || data == NULL)  { return STS3215_ERR_NULL_PTR; }
	if (data_len == 0U)               { return STS3215_ERR_INVALID_PARAM; }

	uint8_t n_params = data_len + 1U;

	if (data_len > (uint8_t)STS3215_PARAM_MAX) { return STS3215_ERR_INVALID_PARAM; }
	if (!prv_cap_ok(cap, n_params))             { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[STS3215_PARAM_MAX + 1U];
	params[0] = reg;
	memcpy(&params[1], data, data_len);

	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, n_params);
}

/**
 * WRITE DATA — Atomic motion command builder
 *
 * Packs acceleration + target_pos + running_time + running_speed into a
 * single WRITE DATA frame
 */
int16_t STS3215_BuildMotionCmd(uint8_t *buf, uint16_t cap,
		uint8_t id, const STS3215_MotionCmd_t *cmd)
{
	if (buf == NULL || cmd == NULL)  { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 8U))        { return STS3215_ERR_BUFFER_SMALL; }

	uint16_t pos = (uint16_t)cmd->target_pos;

	uint8_t params[8] = {
			STS3215_REG_ACCELERATION,
			cmd->acceleration,
			(uint8_t)(pos & 0xFFU),
			(uint8_t)((pos >> 8U) & 0xFFU),
			(uint8_t)(cmd->running_time & 0xFFU),
			(uint8_t)((cmd->running_time >> 8U) & 0xFFU),
			(uint8_t)(cmd->running_speed & 0xFFU),
			(uint8_t)((cmd->running_speed >> 8U) & 0xFFU),
	};
	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 8U);
}

/* =========================================================================
 * Packet Builders — Deferred/Syncrhonised Execution
 * ========================================================================= */

/**
 * REG WRITE (Protocol Manual 1.3.4)
 * All buffered writes execute simultaneously on receipt of ACTION
 */
int16_t STS3215_BuildRegWrite(uint8_t *buf, uint16_t cap,
		uint8_t id, uint8_t reg,
		const uint8_t *data, uint8_t data_len)
{
	if (buf == NULL || data == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (data_len == 0U)                        { return STS3215_ERR_INVALID_PARAM; }
	if (data_len > (uint8_t)STS3215_PARAM_MAX) { return STS3215_ERR_INVALID_PARAM; }

	uint8_t n_params = data_len + 1U;
	if (!prv_cap_ok(cap, n_params))            { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[STS3215_PARAM_MAX + 1U];
	params[0] = reg;
	memcpy(&params[1], data, data_len);

	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_REG_WRITE_DATA, params, n_params);
}

/**
 * REG WRITE — Atomic motion command builder.
 */
int16_t STS3215_BuildRegWriteMotionCmd(uint8_t *buf, uint16_t cap,
		uint8_t id, const STS3215_MotionCmd_t *cmd)
{
	if (buf == NULL || cmd == NULL)  { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 8U))        { return STS3215_ERR_BUFFER_SMALL; }

	uint16_t pos = (uint16_t)cmd->target_pos;

	uint8_t params[8] = {
			STS3215_REG_ACCELERATION,
			cmd->acceleration,
			(uint8_t)(pos & 0xFFU),
			(uint8_t)((pos >> 8U) & 0xFFU),
			(uint8_t)(cmd->running_time & 0xFFU),
			(uint8_t)((cmd->running_time >> 8U) & 0xFFU),
			(uint8_t)(cmd->running_speed & 0xFFU),
			(uint8_t)((cmd->running_speed >> 8U) & 0xFFU),
	};
	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_REG_WRITE_DATA, params, 8U);
}

/**
 * ACTION (Protocol Manual 1.3.5)
 */
int16_t STS3215_BuildAction(uint8_t *buf, uint16_t cap, uint8_t id)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_ACTION, NULL, 0U);
}

/**
 * SYNC WRITE (Protocol Manual 1.3.6)
 *
 * @param reg       First register address (same for all servos)
 * @param data_len  Payload bytes per servo
 * @param entries   Array of N {id, data[]} pairs
 * @param n         Number of servos
 */
int16_t STS3215_BuildSyncWrite(uint8_t *buf, uint16_t cap,
		uint8_t reg, uint8_t data_len,
		const STS3215_SyncEntry_t *entries, uint8_t n)
{
	if (buf == NULL || entries == NULL)  { return STS3215_ERR_NULL_PTR; }
	if (n == 0U || n > STS3215_SYNC_MAX_SERVOS) { return STS3215_ERR_INVALID_PARAM; }
	if (data_len == 0U || data_len > STS3215_SYNC_DATA_MAX_LEN) {
		return STS3215_ERR_INVALID_PARAM;
	}

	uint8_t n_params = 2U + (uint8_t)(n * (1U + data_len));
	if (!prv_cap_ok(cap, n_params)) { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[2U + STS3215_SYNC_MAX_SERVOS * (1U + STS3215_SYNC_DATA_MAX_LEN)];

	params[0] = reg;
	params[1] = data_len;

	uint8_t idx = 2U;
	for (uint8_t i = 0U; i < n; i++) {
		params[idx++] = entries[i].id;
		memcpy(&params[idx], entries[i].data, data_len);
		idx += data_len;
	}

	return (int16_t)prv_pack_frame(buf, STS3215_BROADCAST_ID,
			STS3215_INSTR_SYNC_WRITE_DATA, params, n_params);
}

/**
 * SYNC READ (Protocol Manual 1.3.7)
 *
 * @param reg       First register address
 * @param data_len  Bytes to read per servo
 * @param ids       Array of N servo IDs
 * @param n         Number of servos
 */
int16_t STS3215_BuildSyncRead(uint8_t *buf, uint16_t cap,
		uint8_t reg, uint8_t data_len,
		const uint8_t *ids, uint8_t n)
{
	if (buf == NULL || ids == NULL) { return STS3215_ERR_NULL_PTR; }
	if (n == 0U || n > STS3215_SYNC_MAX_SERVOS) { return STS3215_ERR_INVALID_PARAM; }
	if (data_len == 0U) { return STS3215_ERR_INVALID_PARAM; }

	uint8_t n_params = 2U + n;
	if (!prv_cap_ok(cap, n_params)) { return STS3215_ERR_BUFFER_SMALL; }

	uint8_t params[2U + STS3215_SYNC_MAX_SERVOS];
	params[0] = reg;
	params[1] = data_len;
	memcpy(&params[2], ids, n);

	return (int16_t)prv_pack_frame(buf, STS3215_BROADCAST_ID,
			STS3215_INSTR_SYNC_READ_DATA, params, n_params);
}

/* =========================================================================
 * Packet Builders — Control
 * ========================================================================= */

int16_t STS3215_BuildEnableTorque(uint8_t *buf, uint16_t cap, uint8_t id)
{
	return STS3215_BuildWrite1B(buf, cap, id,
			STS3215_REG_TORQUE_SWITCH,
			(uint8_t)STS3215_TORQUE_ON);
}

int16_t STS3215_BuildDisableTorque(uint8_t *buf, uint16_t cap, uint8_t id)
{
	return STS3215_BuildWrite1B(buf, cap, id,
			STS3215_REG_TORQUE_SWITCH,
			(uint8_t)STS3215_TORQUE_OFF);
}

/**
 * @brief Force the servo's internal position correction to center.
 */
int16_t STS3215_BuildCenterServo(uint8_t *buf, uint16_t cap, uint8_t id)
{
	return STS3215_BuildWrite1B(buf, cap, id,
			STS3215_REG_TORQUE_SWITCH,
			(uint8_t)STS3215_TORQUE_CENTER);
}

/**
 * Unlock EEPROM — Must be called before writing any EEPROM-area register if you want
 * the value to persist across power cycles
 */
int16_t STS3215_BuildUnlockEEPROM(uint8_t *buf, uint16_t cap, uint8_t id)
{
	return STS3215_BuildWrite1B(buf, cap, id,
			STS3215_REG_LOCK_MARK,
			(uint8_t)STS3215_LOCK_OPEN);
}

/**
 * Lock EEPROM — Better to call it after any EEPROM configuration to guard against accidental writes
 */
int16_t STS3215_BuildLockEEPROM(uint8_t *buf, uint16_t cap, uint8_t id)
{
	return STS3215_BuildWrite1B(buf, cap, id,
			STS3215_REG_LOCK_MARK,
			(uint8_t)STS3215_LOCK_CLOSED);
}

/**
 * RESET — Restore factory defaults (Protocol Manual 1.3.8)
 */
int16_t STS3215_BuildReset(uint8_t *buf, uint16_t cap, uint8_t id)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_RESET, NULL, 0U);
}

/**
 * RESET TURNS — Clear the multi-turn cycle counter.
 */
int16_t STS3215_BuildResetTurns(uint8_t *buf, uint16_t cap, uint8_t id)
{
	if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
	if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

	return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_RESET_TURNS, NULL, 0U);
}

/* =========================================================================
 * Reply Parser
 * ========================================================================= */

/**
 * @brief Parse a raw UART RX buffer into a STS3215_Reply_t
 */
STS3215_Status_t STS3215_ParseReply(const uint8_t *buf, uint16_t buf_len,
		STS3215_Reply_t *reply)
{
	if (buf == NULL || reply == NULL) { return STS3215_ERR_NULL_PTR; }
	if (buf_len < 6U) { return STS3215_ERR_FRAME_SHORT; }
	if (buf[STS3215_REPLY_OFF_HEADER0] != STS3215_HEADER_BYTE ||
			buf[STS3215_REPLY_OFF_HEADER1] != STS3215_HEADER_BYTE) {
		return STS3215_ERR_BAD_HEADER;
	}

	uint8_t id     = buf[STS3215_REPLY_OFF_ID];
	uint8_t length = buf[STS3215_REPLY_OFF_LENGTH];
	uint8_t error  = buf[STS3215_REPLY_OFF_ERROR];


	if (length < 2U) { return STS3215_ERR_FRAME_SHORT; }
	uint16_t expected_total = (uint16_t)length + 4U;
	if (buf_len < expected_total) { return STS3215_ERR_FRAME_SHORT; }

	uint8_t  n_data = length - 2U;
	uint32_t sum = (uint32_t)id + (uint32_t)length + (uint32_t)error;

	for (uint8_t i = 0U; i < n_data; i++) {
		sum += (uint32_t)buf[STS3215_REPLY_OFF_DATA + i];
	}

	uint8_t expected_chk = (uint8_t)(~sum & 0xFFU);
	uint8_t actual_chk   = buf[STS3215_REPLY_OFF_DATA + n_data];

	if (actual_chk != expected_chk) { return STS3215_ERR_BAD_CHECKSUM; }

	reply->id = id;
	reply->error = error;
	reply->data_len = n_data;

	if (n_data > 0U) {
		uint8_t copy_len = (n_data <= (uint8_t)STS3215_REPLY_DATA_MAX) ? n_data : (uint8_t)STS3215_REPLY_DATA_MAX;
		memcpy(reply->data, &buf[STS3215_REPLY_OFF_DATA], copy_len);
	}

	return (error != 0U) ? STS3215_ERR_SERVO_FAULT : STS3215_OK;
}

/* =========================================================================
 * Utility / Unit Conversion
 * ========================================================================= */

/* Little-endian 2 bytes -> uint16_t */
uint16_t STS3215_UnpackU16LE(const uint8_t *buf)
{
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
}

/* Little-endian 2 bytes -> int16_t (signed positions, speeds) */
int16_t STS3215_UnpackS16LE(const uint8_t *buf)
{
	return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
}

/* Steps -> degrees */
float STS3215_StepsToDeg(int16_t steps)
{
	return (float)steps * STS3215_DEG_PER_STEP_F;
}

/* Steps -> radians */
float STS3215_StepsToRad(int16_t steps)
{
	return (float)steps * STS3215_RAD_PER_STEP_F;
}

/* Degrees -> nearest step */
int16_t STS3215_DegToSteps(float deg)
{
	return (int16_t)(deg / STS3215_DEG_PER_STEP_F);
}

/* Raw current register -> mA */
float STS3215_RawToMilliamps(uint16_t raw)
{
	return (float)raw * STS3215_CURRENT_LSB_MA_F;
}

/* Raw voltage register -> volts */
float STS3215_RawToVolts(uint8_t raw)
{
	return (float)raw * STS3215_VOLTAGE_LSB_V_F;
}

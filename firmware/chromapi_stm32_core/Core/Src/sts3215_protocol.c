/**
 * @file    sts3215_protocol.c
 * @brief   STS3215 — Pure C protocol implementation
 *
 * No HAL dependency. Compile and test on host PC:
 *   gcc -std=c99 -Wall -Wextra sts3215_protocol.c -o test
 */

#include "sts3215_protocol.h"
#include <string.h>   /* memcpy */

/* =========================================================================
 * Internal helpers (static — not exposed)
 * ========================================================================= */

/**
 * @brief Compute Feetech checksum.
 *
 * Formula [Protocol Manual §1.1]:
 *   Checksum = ~( ID + Length + Instruction + Param1 + ... + ParamN ) & 0xFF
 *
 * Uses uint32_t accumulator to safely handle overflow before masking.
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
 * Layout: FF FF ID LENGTH INSTR [PARAMS...] CHECKSUM
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
 * @brief Guard: verify caller buffer fits the required frame.
 *        Required capacity = 6 (overhead) + n_params.
 */
static bool prv_cap_ok(uint16_t cap, uint8_t n_params)
{
    return cap >= (uint16_t)(6U + (uint16_t)n_params);
}

/* =========================================================================
 * Packet Builders — Single-instruction
 * ========================================================================= */

/**
 * PING — Query servo working status.
 *
 * [Protocol Manual §1.3.1]
 * Frame: FF FF <id> 02 01 <checksum>
 * Unique behaviour: broadcast ID (0xFE) also generates a reply.
 */
int16_t STS3215_BuildPing(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_PING, NULL, 0U);
}

/**
 * READ DATA — Read `len` bytes from register `reg`.
 *
 * [Protocol Manual §1.3.2]
 * Frame: FF FF <id> 04 02 <reg> <len> <checksum>
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
 * WRITE DATA — Write one byte to register `reg`.
 *
 * [Protocol Manual §1.3.3]
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
 * WRITE DATA — Write a 16-bit value to register `reg`.
 *
 * Byte order: Little-Endian (low byte first) per Magnetic Encoding series
 * specification. [Protocol Manual §1.0]
 */
int16_t STS3215_BuildWrite2B(uint8_t *buf, uint16_t cap,
                              uint8_t id, uint8_t reg, uint16_t val)
{
    if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 3U)) { return STS3215_ERR_BUFFER_SMALL; }

    uint8_t params[3] = {
        reg,
        (uint8_t)(val & 0xFFU),         /* low byte first  */
        (uint8_t)((val >> 8U) & 0xFFU)  /* high byte second */
    };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 3U);
}

/**
 * WRITE DATA — Write raw byte array starting at register `reg`.
 *
 * General-purpose form. The caller is responsible for correct byte order.
 * Used internally by higher-level builders.
 */
int16_t STS3215_BuildWriteRaw(uint8_t *buf, uint16_t cap,
                               uint8_t id, uint8_t reg,
                               const uint8_t *data, uint8_t data_len)
{
    if (buf == NULL || data == NULL)  { return STS3215_ERR_NULL_PTR; }
    if (data_len == 0U)               { return STS3215_ERR_INVALID_PARAM; }

    /* n_params = reg_addr(1) + data bytes */
    uint8_t n_params = data_len + 1U;

    /* Guard: data_len must fit in local params buffer */
    if (data_len > (uint8_t)STS3215_PARAM_MAX) { return STS3215_ERR_INVALID_PARAM; }
    if (!prv_cap_ok(cap, n_params))             { return STS3215_ERR_BUFFER_SMALL; }

    uint8_t params[STS3215_PARAM_MAX + 1U];
    params[0] = reg;
    memcpy(&params[1], data, data_len);

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, n_params);
}

/**
 * WRITE DATA — Atomic motion command (registers 0x29–0x2F, 7 data bytes).
 *
 * Packs acceleration + target_pos + running_time + running_speed into a
 * single WRITE DATA frame starting at STS3215_REG_ACCELERATION (0x29).
 *
 * Using one frame eliminates the risk of the servo acting on a partially
 * updated position/speed pair that individual writes would create.
 *
 * Equivalent to Protocol Manual Example 7 (with acceleration prefix).
 */
int16_t STS3215_BuildMotionCmd(uint8_t *buf, uint16_t cap,
                                uint8_t id, const STS3215_MotionCmd_t *cmd)
{
    if (buf == NULL || cmd == NULL)  { return STS3215_ERR_NULL_PTR; }
    /* n_params = reg_addr(1) + accel(1) + pos(2) + time(2) + speed(2) = 8 */
    if (!prv_cap_ok(cap, 8U))        { return STS3215_ERR_BUFFER_SMALL; }

    /* Cast target_pos to uint16_t for safe bit manipulation */
    uint16_t pos = (uint16_t)cmd->target_pos;

    uint8_t params[8] = {
        STS3215_REG_ACCELERATION,
        cmd->acceleration,
        (uint8_t)(pos & 0xFFU),                          /* pos  low  */
        (uint8_t)((pos >> 8U) & 0xFFU),                  /* pos  high */
        (uint8_t)(cmd->running_time & 0xFFU),             /* time low  */
        (uint8_t)((cmd->running_time >> 8U) & 0xFFU),    /* time high */
        (uint8_t)(cmd->running_speed & 0xFFU),            /* spd  low  */
        (uint8_t)((cmd->running_speed >> 8U) & 0xFFU),   /* spd  high */
    };
    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_WRITE_DATA, params, 8U);
}

/* =========================================================================
 * Packet Builders — Deferred / Synchronised
 * ========================================================================= */

/**
 * REG WRITE — Buffer a write for deferred execution via ACTION.
 *
 * [Protocol Manual §1.3.4]
 * Identical wire format to WRITE DATA but with opcode 0x04.
 * The servo stores the payload and sets its async-write flag.
 * All buffered writes execute simultaneously on receipt of ACTION.
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
 * REG WRITE — Buffered atomic motion command.
 *
 * Use this for multi-servo synchronised motion:
 *   1. Send STS3215_BuildRegWriteMotionCmd() to each servo individually.
 *   2. Send STS3215_BuildAction(buf, cap, STS3215_BROADCAST_ID) once.
 *   → All servos start moving simultaneously.
 *
 * [Protocol Manual §1.3.4 + §1.3.5, Example 5 + Example 6]
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
 * ACTION — Trigger all previously buffered REG_WRITE commands.
 *
 * [Protocol Manual §1.3.5]
 * Typically sent to STS3215_BROADCAST_ID so all servos fire at once.
 * No reply is generated on broadcast.
 * Frame: FF FF FE 02 05 FA
 */
int16_t STS3215_BuildAction(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_ACTION, NULL, 0U);
}

/* =========================================================================
 * Packet Builders — Bulk (SYNC)
 * ========================================================================= */

/**
 * SYNC WRITE — Write the same register block to N servos in one frame.
 *
 * [Protocol Manual §1.3.6]
 *
 * All entries must provide exactly `data_len` bytes of pre-formatted
 * (little-endian packed) payload data. The caller is responsible for
 * packing multi-byte values correctly before passing entries[].
 *
 * Frame: FF FF FE <(L+1)*N+4> 83 <reg> <L> <id1> <d..> <id2> <d..> <chk>
 * Always uses broadcast ID; no reply generated.
 *
 * @param reg       First register address (same for all servos)
 * @param data_len  Payload bytes per servo (L, max STS3215_SYNC_DATA_MAX_LEN)
 * @param entries   Array of N {id, data[]} pairs
 * @param n         Number of servos (1 ≤ n ≤ STS3215_SYNC_MAX_SERVOS)
 */
int16_t STS3215_BuildSyncWrite(uint8_t *buf, uint16_t cap,
                                uint8_t reg, uint8_t data_len,
                                const STS3215_SyncEntry_t *entries, uint8_t n)
{
    if (buf == NULL || entries == NULL)                    { return STS3215_ERR_NULL_PTR; }
    if (n == 0U || n > STS3215_SYNC_MAX_SERVOS)           { return STS3215_ERR_INVALID_PARAM; }
    if (data_len == 0U || data_len > STS3215_SYNC_DATA_MAX_LEN) {
        return STS3215_ERR_INVALID_PARAM;
    }

    /* n_params = reg(1) + data_len(1) + N*(id(1) + L) */
    uint8_t n_params = 2U + (uint8_t)(n * (1U + data_len));
    if (!prv_cap_ok(cap, n_params)) { return STS3215_ERR_BUFFER_SMALL; }

    /* Build params array in local buffer (max size is bounded at compile time) */
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
 * SYNC READ — Query the same register block from N servos sequentially.
 *
 * [Protocol Manual §1.3.7]
 *
 * Servos reply in the order their IDs appear in the frame.
 * Each reply must be parsed individually with STS3215_ParseReply().
 *
 * Frame: FF FF FE <N+4> 82 <reg> <data_len> <id1> ... <idN> <checksum>
 *
 * @param reg       First register address
 * @param data_len  Bytes to read per servo
 * @param ids       Array of N servo IDs
 * @param n         Number of servos (1 ≤ n ≤ STS3215_SYNC_MAX_SERVOS)
 */
int16_t STS3215_BuildSyncRead(uint8_t *buf, uint16_t cap,
                               uint8_t reg, uint8_t data_len,
                               const uint8_t *ids, uint8_t n)
{
    if (buf == NULL || ids == NULL)           { return STS3215_ERR_NULL_PTR; }
    if (n == 0U || n > STS3215_SYNC_MAX_SERVOS) { return STS3215_ERR_INVALID_PARAM; }
    if (data_len == 0U)                        { return STS3215_ERR_INVALID_PARAM; }

    /* n_params = reg(1) + data_len(1) + N id bytes */
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
 * Packet Builders — Control & Housekeeping Wrappers
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
 * Force the servo's internal position correction to center (2048 steps).
 * Writes 128 to STS3215_REG_TORQUE_SWITCH per memory table specification.
 */
int16_t STS3215_BuildCenterServo(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return STS3215_BuildWrite1B(buf, cap, id,
                                STS3215_REG_TORQUE_SWITCH,
                                (uint8_t)STS3215_TORQUE_CENTER);
}

/**
 * Unlock EEPROM: write STS3215_LOCK_OPEN (0) to STS3215_REG_LOCK_MARK.
 *
 * ⚠ Must be called before writing any EEPROM-area register if you want
 * the value to persist across power cycles. [Memory Table §0x37]
 */
int16_t STS3215_BuildUnlockEEPROM(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return STS3215_BuildWrite1B(buf, cap, id,
                                STS3215_REG_LOCK_MARK,
                                (uint8_t)STS3215_LOCK_OPEN);
}

/**
 * Lock EEPROM: write STS3215_LOCK_CLOSED (1) to STS3215_REG_LOCK_MARK.
 *
 * Call after completing EEPROM configuration to guard against accidental writes.
 */
int16_t STS3215_BuildLockEEPROM(uint8_t *buf, uint16_t cap, uint8_t id)
{
    return STS3215_BuildWrite1B(buf, cap, id,
                                STS3215_REG_LOCK_MARK,
                                (uint8_t)STS3215_LOCK_CLOSED);
}

/**
 * RESET — Restore factory defaults from EEPROM.
 *
 * [Protocol Manual §1.3.8]
 * Frame: FF FF <id> 02 06 <checksum>
 */
int16_t STS3215_BuildReset(uint8_t *buf, uint16_t cap, uint8_t id)
{
    if (buf == NULL)           { return STS3215_ERR_NULL_PTR; }
    if (!prv_cap_ok(cap, 0U)) { return STS3215_ERR_BUFFER_SMALL; }

    return (int16_t)prv_pack_frame(buf, id, STS3215_INSTR_RESET, NULL, 0U);
}

/**
 * RESET TURNS — Clear the multi-turn cycle counter.
 *
 * NOTE: The memory table states "Power failure does not save cycle count",
 * so this instruction only matters within a powered session.
 * [Memory Table §Examples, Example 9]
 * Frame: FF FF <id> 02 0A <checksum>
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
 * @brief Parse a raw UART RX buffer into a STS3215_Reply_t.
 *
 * Checksum covers: ID + Length + ERROR + DataParam1 + ... + DataParamN
 * [Protocol Manual §1.2 — verified against examples §1.3.1 and §1.3.2]
 *
 * Return matrix:
 *  STS3215_OK            → frame structurally valid, ERROR = 0
 *  STS3215_ERR_SERVO_FAULT → frame valid, ERROR != 0; reply populated
 *  STS3215_ERR_FRAME_SHORT → buf_len < 6 or inconsistent with LENGTH field
 *  STS3215_ERR_BAD_HEADER  → first two bytes != 0xFF 0xFF
 *  STS3215_ERR_BAD_CHECKSUM → calculated != received checksum
 *  STS3215_ERR_NULL_PTR    → buf or reply is NULL
 */
STS3215_Status_t STS3215_ParseReply(const uint8_t *buf, uint16_t buf_len,
                                     STS3215_Reply_t *reply)
{
    if (buf == NULL || reply == NULL) { return STS3215_ERR_NULL_PTR; }

    /* Minimum valid reply: FF FF ID LEN ERROR CHECKSUM = 6 bytes */
    if (buf_len < 6U) { return STS3215_ERR_FRAME_SHORT; }

    /* Header guard */
    if (buf[STS3215_REPLY_OFF_HEADER0] != STS3215_HEADER_BYTE ||
        buf[STS3215_REPLY_OFF_HEADER1] != STS3215_HEADER_BYTE) {
        return STS3215_ERR_BAD_HEADER;
    }

    uint8_t id     = buf[STS3215_REPLY_OFF_ID];
    uint8_t length = buf[STS3215_REPLY_OFF_LENGTH]; /* = n_data + 2 */
    uint8_t error  = buf[STS3215_REPLY_OFF_ERROR];

    /* length field must encode at least error byte + checksum byte */
    if (length < 2U) { return STS3215_ERR_FRAME_SHORT; }

    /* Total expected frame = header(2) + id(1) + length_field(1) + length bytes */
    uint16_t expected_total = (uint16_t)length + 4U;
    if (buf_len < expected_total) { return STS3215_ERR_FRAME_SHORT; }

    /* Checksum verification
     *   sum  = ID + LENGTH + ERROR + data_param[0..n_data-1]
     *   chk  = ~sum & 0xFF
     * n_data = length - 2  (excludes error byte and checksum byte)
     */
    uint8_t  n_data = length - 2U;
    uint32_t sum    = (uint32_t)id + (uint32_t)length + (uint32_t)error;

    for (uint8_t i = 0U; i < n_data; i++) {
        sum += (uint32_t)buf[STS3215_REPLY_OFF_DATA + i];
    }

    uint8_t expected_chk = (uint8_t)(~sum & 0xFFU);
    uint8_t actual_chk   = buf[STS3215_REPLY_OFF_DATA + n_data]; /* last byte */

    if (actual_chk != expected_chk) { return STS3215_ERR_BAD_CHECKSUM; }

    /* Populate output — always written so caller can log on SERVO_FAULT */
    reply->id       = id;
    reply->error    = error;
    reply->data_len = n_data;

    if (n_data > 0U) {
        uint8_t copy_len = (n_data <= (uint8_t)STS3215_REPLY_DATA_MAX)
                           ? n_data
                           : (uint8_t)STS3215_REPLY_DATA_MAX;
        memcpy(reply->data, &buf[STS3215_REPLY_OFF_DATA], copy_len);
    }

    return (error != 0U) ? STS3215_ERR_SERVO_FAULT : STS3215_OK;
}

/* =========================================================================
 * Utility / Unit Conversion
 * ========================================================================= */

/** Unpack 2 consecutive bytes (little-endian) into uint16_t. */
uint16_t STS3215_UnpackU16LE(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8U);
}

/** Unpack 2 consecutive bytes (little-endian) into int16_t (signed). */
int16_t STS3215_UnpackS16LE(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8U));
}

/** Convert step count to degrees (float). */
float STS3215_StepsToDeg(int16_t steps)
{
    return (float)steps * STS3215_DEG_PER_STEP_F;
}

/** Convert step count to radians (float). */
float STS3215_StepsToRad(int16_t steps)
{
    return (float)steps * STS3215_RAD_PER_STEP_F;
}

/** Convert degrees to nearest step count (rounds toward zero). */
int16_t STS3215_DegToSteps(float deg)
{
    return (int16_t)(deg / STS3215_DEG_PER_STEP_F);
}

/** Convert raw current register value to milliamps (float). */
float STS3215_RawToMilliamps(uint16_t raw)
{
    return (float)raw * STS3215_CURRENT_LSB_MA_F;
}

/** Convert raw voltage register value to volts (float). */
float STS3215_RawToVolts(uint8_t raw)
{
    return (float)raw * STS3215_VOLTAGE_LSB_V_F;
}
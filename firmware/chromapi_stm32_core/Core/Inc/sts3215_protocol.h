/**
 * @file    sts3215_protocol.h
 * @brief   STS3215 — Pure C protocol layer (packet builders + reply parser)
 *
 *  Fully unit-testable on host PC with any C compiler.
 *
 */

#ifndef STS3215_PROTOCOL_H
#define STS3215_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "sts3215_regs.h"

/* =========================================================================
 * Buffer sizing
 * ========================================================================= */

/* --- Maximum servos addressable in a single SYNC frame --- */
#define STS3215_SYNC_MAX_SERVOS     (16U)

/* --- Maximum data bytes per servo in a SYNC_WRITE --- */
#define STS3215_SYNC_DATA_MAX_LEN   (8U)

/**
 * Universal TX buffer size (sized for worst-case SYNC_WRITE):
 *   header(2) + id(1) + len(1) + instr(1) + reg(1) + dlen(1)
 *   + N * (id(1) + data(L)) + checksum(1)
 */
#define STS3215_TX_BUF_SIZE (7U + STS3215_SYNC_MAX_SERVOS * (1U + STS3215_SYNC_DATA_MAX_LEN))

/**
 * RX buffer per reply frame:
 *   header(2) + id(1) + len(1) + error(1) + data(N) + checksum(1)
 * Sized for largest single-servo read (full feedback block = 8B data).
 */
#define STS3215_RX_BUF_SIZE (16U)

/* =========================================================================
 * Status / Error Codes
 * ========================================================================= */

typedef enum {
    STS3215_OK                = 0,
    STS3215_ERR_NULL_PTR      = -1,
    STS3215_ERR_INVALID_PARAM = -2,  /* Argument out of valid range */
    STS3215_ERR_BUFFER_SMALL  = -3,
    STS3215_ERR_BAD_HEADER    = -4,
    STS3215_ERR_BAD_CHECKSUM  = -5,
    STS3215_ERR_SERVO_FAULT   = -6,  /* Frame valid but ERROR byte non-zero */
    STS3215_ERR_FRAME_SHORT   = -7,  /* buf_len < expected frame length */
} STS3215_Status_t;

/* =========================================================================
 * Data Structures
 * ========================================================================= */

/**
 * @brief Parsed representation of a servo reply packet.
 */
typedef struct {
    uint8_t id;                              
    uint8_t error;                          
    uint8_t data[STS3215_REPLY_DATA_MAX]; 
    uint8_t data_len;   
} STS3215_Reply_t;

/**
 * @brief Motion command parameters
 */
typedef struct {
    uint8_t  acceleration;
    int16_t  target_pos;
    uint16_t running_time;
    uint16_t running_speed;
} STS3215_MotionCmd_t;

/**
 * @brief One servo entry for SYNC_WRITE operations.
 */
typedef struct {
    uint8_t id;
    uint8_t data[STS3215_SYNC_DATA_MAX_LEN];
} STS3215_SyncEntry_t;

/* =========================================================================
 * Packet Builder - All functions return the frame length (> 0) on success,
 * or a STS3215_Status_t value (< 0) on error.
 * ========================================================================= */

/* --- Instructions frames --- */

/* --- PING --- */
int16_t STS3215_BuildPing(uint8_t *buf, uint16_t cap, uint8_t id);

/* --- READ DATA --- */
int16_t STS3215_BuildRead(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint8_t len);

/* --- WRITE DATA --- */
/* Write a byte to register `reg` */
int16_t STS3215_BuildWrite1B(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint8_t val);

/* Write a 16-bit value to register `reg` (little-endian). */
int16_t STS3215_BuildWrite2B(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, uint16_t val);

/* Write a raw byte array starting at register `reg`. */
int16_t STS3215_BuildWriteRaw(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, const uint8_t *data, uint8_t data_len);

/* Atomic motion command builder */
int16_t STS3215_BuildMotionCmd(uint8_t *buf, uint16_t cap, uint8_t id, const STS3215_MotionCmd_t *cmd);

/* --- REG WRITE --- */
int16_t STS3215_BuildRegWrite(uint8_t *buf, uint16_t cap, uint8_t id, uint8_t reg, const uint8_t *data, uint8_t data_len);

/* Atomic motion command builder */
int16_t STS3215_BuildRegWriteMotionCmd(uint8_t *buf, uint16_t cap, uint8_t id, const STS3215_MotionCmd_t *cmd);

/* --- ACTION --- */
int16_t STS3215_BuildAction(uint8_t *buf, uint16_t cap, uint8_t id);

/* --- SYNC WRITE --- */
int16_t STS3215_BuildSyncWrite(uint8_t *buf, uint16_t cap, uint8_t reg, uint8_t data_len, const STS3215_SyncEntry_t *entries, uint8_t n);

/* --- SYNC READ --- */
int16_t STS3215_BuildSyncRead(uint8_t *buf, uint16_t cap, uint8_t reg, uint8_t data_len, const uint8_t *ids, uint8_t n);

/* --- Control --- */
int16_t STS3215_BuildEnableTorque(uint8_t *buf, uint16_t cap, uint8_t id);
int16_t STS3215_BuildDisableTorque(uint8_t *buf, uint16_t cap, uint8_t id);

/* Force position correction to mechanical center (2048 steps). */
int16_t STS3215_BuildCenterServo(uint8_t *buf, uint16_t cap, uint8_t id);

/* --- EEPROM Control --- */
int16_t STS3215_BuildUnlockEEPROM(uint8_t *buf, uint16_t cap, uint8_t id);
int16_t STS3215_BuildLockEEPROM(uint8_t *buf, uint16_t cap, uint8_t id);

/* --- RESET --- */
int16_t STS3215_BuildReset(uint8_t *buf, uint16_t cap, uint8_t id);

/* Clear the multi-turn cycle counter (volatile) */
int16_t STS3215_BuildResetTurns(uint8_t *buf, uint16_t cap, uint8_t id);

/* =========================================================================
 * Reply Parser
 * ========================================================================= */

/**
 * @brief Parse a raw RX buffer into a STS3215_Reply_t.
 *
 * Validates header, frame length consistency, and checksum.
 * On STS3215_ERR_SERVO_FAULT the reply struct is still fully populated —
 * inspect reply->error with STS3215_FAULT_* masks.
 *
 * @param buf      Raw bytes from UART DMA buffer
 * @param buf_len  Number of received bytes
 * @param reply    Output structure; written on OK or SERVO_FAULT
 */
STS3215_Status_t STS3215_ParseReply(const uint8_t *buf, uint16_t buf_len,
                                     STS3215_Reply_t *reply);

/* =========================================================================
 * Utility / Unit Conversion
 * ========================================================================= */

/* Little-endian 2 bytes -> uint16_t */
uint16_t STS3215_UnpackU16LE(const uint8_t *buf);

/* Little-endian 2 bytes -> int16_t (signed positions, speeds) */
int16_t STS3215_UnpackS16LE(const uint8_t *buf);

/* Steps -> degrees */
float STS3215_StepsToDeg(int16_t steps);

/* Steps -> radians */
float STS3215_StepsToRad(int16_t steps);

/* Degrees -> nearest step */
int16_t STS3215_DegToSteps(float deg);

/* Raw current register -> mA */
float STS3215_RawToMilliamps(uint16_t raw);

/* Raw voltage register -> volts */
float STS3215_RawToVolts(uint8_t raw);

#endif /* STS3215_PROTOCOL_H */
/**
 * @file    sts3215_regs.h
 * @brief   STS3215 Feetech Servo — Complete Register Map & Protocol Constants
 *
 * Sources:
 *  [1] STS3215 Memory Table Parameters - v3.6 https://github.com/Mowibox/stm32-sts3215-lib/blob/main/docs/sts3215_memory_table.md
 *  [2] Smart Bus Servo Communication Protocol Manual https://github.com/Mowibox/stm32-sts3215-lib/blob/main/docs/protocol_manual.md
 *
 *  Magnetic Encoding series: Little-Endian (low byte first).
 *  2-byte registers: address = low byte, address+1 = high byte.
 */

#ifndef STS3215_REGS_H
#define STS3215_REGS_H

#include <stdint.h>

/* =========================================================================
 * 1. PROTOCOL FRAME CONSTANTS (Protocol Manual 1.1, 1.2)
 * ========================================================================= */

/** Header byte value */
#define STS3215_HEADER_BYTE             (0xFFU)

/** Broadcast ID: all servos receive, none reply (except with PING instruction) */
#define STS3215_BROADCAST_ID            (0xFEU)

/** ID range: from 0 (0x00) to 253 (0xFD) */
#define STS3215_ID_MIN                  (0x00U)
#define STS3215_ID_MAX                  (0xFDU)

/**
 * Packet overhead = Header(2) + ID(1) + Length(1) + Instruction(1) + Checksum(1)
 * Data Length field = N + 2  (instruction byte + checksum byte)
 */
#define STS3215_PACKET_OVERHEAD         (6U)
#define STS3215_PARAM_MAX               (16U)
#define STS3215_TX_BUF_SIZE             (STS3215_PACKET_OVERHEAD + STS3215_PARAM_MAX)

/**
 * Reply packet overhead = Header(2) + ID(1) + Length(1) + ERROR(1) + Checksum(1)
 * Maximum feedback read: 8 bytes data (SYNC READ example) + overhead
 */
#define STS3215_REPLY_OVERHEAD          (6U)
#define STS3215_REPLY_DATA_MAX          (8U)
#define STS3215_RX_BUF_SIZE             (STS3215_REPLY_OVERHEAD + STS3215_REPLY_DATA_MAX)

/** Byte offsets inside a received reply buffer */
#define STS3215_REPLY_OFF_HEADER1       (1U)
#define STS3215_REPLY_OFF_ID            (2U)
#define STS3215_REPLY_OFF_HEADER0       (0U)
#define STS3215_REPLY_OFF_LENGTH        (3U)
#define STS3215_REPLY_OFF_ERROR         (4U)
#define STS3215_REPLY_OFF_DATA          (5U)
/* Checksum is last byte: buf[REPLY_OFF_LENGTH + 2] */

/* =========================================================================
 * 2. INSTRUCTIONS  (Protocol Manual 1.3)
 * ========================================================================= */

#define STS3215_INSTR_PING              (0x01U)
#define STS3215_INSTR_READ_DATA         (0x02U)
#define STS3215_INSTR_WRITE_DATA        (0x03U)
#define STS3215_INSTR_REG_WRITE_DATA    (0x04U)
#define STS3215_INSTR_ACTION            (0x05U)
#define STS3215_INSTR_RESET             (0x06U)
#define STS3215_INSTR_SYNC_READ_DATA         (0x82U)
#define STS3215_INSTR_SYNC_WRITE_DATA        (0x83U)

/**
 * Clear multi-turn counter. 
 * See https://github.com/Mowibox/stm32-sts3215-lib/blob/main/docs/sts3215_memory_table.md#9-number-of-turns-cleared-command
 */
#define STS3215_INSTR_RESET_TURNS       (0x0AU)

/* =========================================================================
 * 3. EEPROM REGISTERS  (STS3215 Memory Table)
 * ========================================================================= */

/* --- Firmware & Hardware Version (read-only) --- */
#define STS3215_REG_FW_MAJOR            (0x00U)
#define STS3215_REG_FW_SUB              (0x01U)
#define STS3215_REG_SERVO_MAIN_VER      (0x03U)
#define STS3215_REG_SERVO_SUB_VER       (0x04U)

/* --- Bus Configuration (read&write) --- */
#define STS3215_REG_ID                  (0x05U)         
#define STS3215_REG_BAUD_RATE           (0x06U)
#define STS3215_REG_RETURN_DELAY        (0x07U)
#define STS3215_REG_RESP_LEVEL          (0x08U)

/* --- Motion Limits (read&write) --- */
#define STS3215_REG_MIN_ANGLE           (0x09U)
#define STS3215_REG_MAX_ANGLE           (0x0BU)
#define STS3215_REG_MAX_TEMP            (0x0DU)
#define STS3215_REG_MAX_VOLTAGE         (0x0EU) 
#define STS3215_REG_MIN_VOLTAGE         (0x0FU) 
#define STS3215_REG_MAX_TORQUE          (0x10U) 

/* --- Internal Control (do not modify without explicit requirement) --- */
#define STS3215_REG_PHASE               (0x12U)

/* --- Fault Protection Masks (read&write) --- */
#define STS3215_REG_UNLOAD_COND         (0x13U)
#define STS3215_REG_LED_ALARM           (0x14U) 

/* --- Position Loop PID Gains (read&write) --- */
#define STS3215_REG_P_COEFF             (0x15U) 
#define STS3215_REG_D_COEFF             (0x16U) 
#define STS3215_REG_I_COEFF             (0x17U) 

/* --- Startup & Deadband (read&write) --- */
#define STS3215_REG_MIN_STARTUP         (0x18U) 
#define STS3215_REG_CW_DEADBAND         (0x1AU) 
#define STS3215_REG_CCW_DEADBAND        (0x1BU)

/* --- Overcurrent Protection (read&write) --- */
#define STS3215_REG_PROT_CURRENT        (0x1CU)

/* --- Multi-turn / Resolution Extension (read&write) --- */
#define STS3215_REG_ANGULAR_RES         (0x1EU) 

/* --- Zero Calibration (read&write) --- */
#define STS3215_REG_POS_CORRECTION      (0x1FU) 

/* --- Control Mode (read&write) --- */
#define STS3215_REG_OPERATION_MODE      (0x21U)

/* --- Overload Protection (read&write) --- */
#define STS3215_REG_PROT_TORQUE         (0x22U)
#define STS3215_REG_PROT_TIME           (0x23U) 
#define STS3215_REG_OVERLOAD_TORQUE     (0x24U)

/* --- Velocity Loop PID & Over-current protection time --- */
#define STS3215_REG_SPEED_P             (0x25U) 
#define STS3215_REG_OC_PROT_TIME        (0x26U) 
#define STS3215_REG_SPEED_I             (0x27U)

/* =========================================================================
 * 4. SRAM REGISTERS  (STS3215 Memory Table)
 * ========================================================================= */

/* --- Torque output control (read&write) --- */
#define STS3215_REG_TORQUE_SWITCH       (0x28U) /**< 1B | R/W                    */

/* --- Motion Control (read&write) --- */
#define STS3215_REG_ACCELERATION        (0x29U) 
#define STS3215_REG_TARGET_POS          (0x2AU) 
#define STS3215_REG_RUNNING_TIME        (0x2CU) 

/* --- Torque limit (read&write) --- */
#define STS3215_REG_TORQUE_LIMIT        (0x30U)

/* --- EEPROM write-lock mark (read&write) --- */
#define STS3215_REG_LOCK_MARK           (0x37U)

/* --- Read-Only Feedback (read-only) --- */
#define STS3215_REG_CURRENT_POS         (0x38U)
#define STS3215_REG_CURRENT_SPEED       (0x3AU)
#define STS3215_REG_CURRENT_LOAD        (0x3CU) 
#define STS3215_REG_CURRENT_VOLTAGE     (0x3EU) 
#define STS3215_REG_CURRENT_TEMP        (0x3FU) 
#define STS3215_REG_ASYNC_FLAG          (0x40U)
#define STS3215_REG_SERVO_STATUS        (0x41U)
#define STS3215_REG_MOVING              (0x42U) 
#define STS3215_REG_CURRENT_CURRENT     (0x45U) 

/* =========================================================================
 * 5. REGISTER WIDTH HELPER  (compile-time, no runtime table)
 * ========================================================================= */

/**
 * @brief Returns the byte-width (1 or 2) of a register at compile time.
 */
#define STS3215_REG_WIDTH(addr)  \
		(((addr) == STS3215_REG_MIN_ANGLE         || \
				(addr) == STS3215_REG_MAX_ANGLE         || \
				(addr) == STS3215_REG_MAX_TORQUE        || \
				(addr) == STS3215_REG_MIN_STARTUP       || \
				(addr) == STS3215_REG_PROT_CURRENT      || \
				(addr) == STS3215_REG_POS_CORRECTION    || \
				(addr) == STS3215_REG_TARGET_POS        || \
				(addr) == STS3215_REG_RUNNING_TIME      || \
				(addr) == STS3215_REG_RUNNING_SPEED     || \
				(addr) == STS3215_REG_TORQUE_LIMIT      || \
				(addr) == STS3215_REG_CURRENT_POS       || \
				(addr) == STS3215_REG_CURRENT_SPEED     || \
				(addr) == STS3215_REG_CURRENT_LOAD      || \
				(addr) == STS3215_REG_CURRENT_CURRENT)  ? 2U : 1U)

/* =========================================================================
 * 6. DEFAULT (FACTORY) VALUES  (See Initial Value column in Memory Table)
 * ========================================================================= */

#define STS3215_DEFAULT_FW_MAJOR        (3U)
#define STS3215_DEFAULT_FW_SUB          (6U)
#define STS3215_DEFAULT_SERVO_MAIN_VER  (9U)
#define STS3215_DEFAULT_SERVO_SUB_VER   (3U)
#define STS3215_DEFAULT_ID              (1U)
#define STS3215_DEFAULT_BAUD_RATE       (0U)
#define STS3215_DEFAULT_RETURN_DELAY    (0U)
#define STS3215_DEFAULT_RESP_LEVEL      (1U)
#define STS3215_DEFAULT_MIN_ANGLE       (0U)
#define STS3215_DEFAULT_MAX_ANGLE       (4095U)
#define STS3215_DEFAULT_MAX_TEMP        (70U)
#define STS3215_DEFAULT_MAX_VOLTAGE     (80U)
#define STS3215_DEFAULT_MIN_VOLTAGE     (40U)
#define STS3215_DEFAULT_MAX_TORQUE      (1000U)
#define STS3215_DEFAULT_PHASE           (12U)
#define STS3215_DEFAULT_UNLOAD_COND     (44U)
#define STS3215_DEFAULT_LED_ALARM       (47U)
#define STS3215_DEFAULT_P_COEFF         (32U)
#define STS3215_DEFAULT_D_COEFF         (32U)
#define STS3215_DEFAULT_I_COEFF         (0U)
#define STS3215_DEFAULT_MIN_STARTUP     (16U)
#define STS3215_DEFAULT_CW_DEADBAND     (1U)
#define STS3215_DEFAULT_CCW_DEADBAND    (1U)
#define STS3215_DEFAULT_PROT_CURRENT    (500U)
#define STS3215_DEFAULT_ANGULAR_RES     (1U)
#define STS3215_DEFAULT_POS_CORRECTION  (0U)
#define STS3215_DEFAULT_OPERATION_MODE  (0U)
#define STS3215_DEFAULT_PROT_TORQUE     (20U)
#define STS3215_DEFAULT_PROT_TIME       (200U)
#define STS3215_DEFAULT_OVERLOAD_TORQUE (80U)
#define STS3215_DEFAULT_SPEED_P         (10U)
#define STS3215_DEFAULT_OC_PROT_TIME    (200U)
#define STS3215_DEFAULT_SPEED_I         (10U)

/* =========================================================================
 * 7. ENUMERATIONS
 * ========================================================================= */

/** --- Baudrates --- */
typedef enum {
	STS3215_BAUD_1000000 = 0,
	STS3215_BAUD_500000  = 1,
	STS3215_BAUD_250000  = 2,
	STS3215_BAUD_128000  = 3,
	STS3215_BAUD_115200  = 4,
	STS3215_BAUD_76800   = 5,
	STS3215_BAUD_57600   = 6,
	STS3215_BAUD_38400   = 7,
} STS3215_BaudRate_t;

/* --- Reply policy --- */
typedef enum {
	STS3215_RESP_READ_PING_ONLY = 0, /* Reply only to READ and PING */
	STS3215_RESP_ALL            = 1, /* Reply to every instruction */
} STS3215_RespLevel_t;

/* --- Control mode --- */
typedef enum {
	STS3215_MODE_POSITION = 0, /* Closed-loop position servo */
	STS3215_MODE_VELOCITY = 1, /* Constant speed */
	STS3215_MODE_PWM      = 2, /* Open-loop PWM */
	STS3215_MODE_STEP     = 3, /* Step count */
} STS3215_OperationMode_t;

/* --- Torque switch values --- */
typedef enum {
	STS3215_TORQUE_OFF    = 0,
	STS3215_TORQUE_ON     = 1,
	STS3215_TORQUE_CENTER = 128, /*  Force position correction to 2048 */
} STS3215_TorqueSwitch_t;

/* --- EEPROM write-lock --- */
typedef enum {
	STS3215_LOCK_OPEN   = 0,
	STS3215_LOCK_CLOSED = 1,
} STS3215_LockMark_t;

/* =========================================================================
 * 8. FAULT / STATUS BIT MASKS
 * ========================================================================= */

#define STS3215_FAULT_VOLTAGE     (1U << 0) /*  Bit0: over/under voltage */
#define STS3215_FAULT_SENSOR      (1U << 1) /*  Bit1: position sensor fault */
#define STS3215_FAULT_TEMPERATURE (1U << 2) /*  Bit2: over temperature */
#define STS3215_FAULT_CURRENT     (1U << 3) /*  Bit3: overcurrent */
#define STS3215_FAULT_ANGLE       (1U << 4) /*  Bit4: angle limit exceeded */
#define STS3215_FAULT_OVERLOAD    (1U << 5) /*  Bit5: mechanical overload */
#define STS3215_FAULT_ALL_MASK    (0x3FU)   /*  All 6 fault bits */

/* =========================================================================
 * 9. PHYSICAL CONSTANTS & UNIT CONVERSION
 * ========================================================================= */

#define STS3215_STEPS_PER_REV        (4096U)
#define STS3215_DEG_PER_STEP_F       (0.087890625f)      /* 360/4096  */
#define STS3215_RAD_PER_STEP_F       (0.001533980787f)   /* 2π/4096 */
#define STS3215_CENTER_ POSITION      (2048U)             /* Mechanical zero */
#define STS3215_CURRENT_LSB_MA_F     (6.5f)              /* mA per LSB */
#define STS3215_VOLTAGE_LSB_V_F      (0.1f)              /* V per LSB */
#define STS3215_ACCEL_LSB_STEP_SS_F  (100.0f)            /* step/s² per LSB */
#define STS3215_PROT_TIME_LSB_MS_F   (10.0f)             /* ms per LSB */
#define STS3215_RETURN_DELAY_LSB_US_F (2.0f)             /* µs per LSB */


#define STS3215_DEG_TO_STEPS(deg)  ((uint16_t)((deg) / STS3215_DEG_PER_STEP_F))
#define STS3215_STEPS_TO_DEG_F(steps) ((float)(steps) * STS3215_DEG_PER_STEP_F)
#define STS3215_RAW_TO_MA_F(raw)   ((float)(raw) * STS3215_CURRENT_LSB_MA_F)
#define STS3215_RAW_TO_V_F(raw)    ((float)(raw) * STS3215_VOLTAGE_LSB_V_F)

#endif /* STS3215_REGS_H */

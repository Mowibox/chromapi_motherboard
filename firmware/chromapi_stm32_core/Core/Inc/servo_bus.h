/* SPDX-License-Identifier: Apache-2.0 */
/**
 * @file    servo_bus.h
 * @brief   The 12-servo bus: the only owner of the STS3215 handle.
 *
 */
#ifndef SERVO_BUS_H
#define SERVO_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

#define SERVO_BUS_COUNT 12U
#define SERVO_BUS_READ_MAX 16U  // longest register read relayed by ServoBus_ReadRegister() [bytes]

typedef struct {
	uint16_t position;
	int16_t  speed;
	int16_t  load;
	uint8_t  temperature;
	uint8_t  voltage;
	int16_t  current;
} ServoFeedback_t;

typedef struct {
	ServoFeedback_t servo[SERVO_BUS_COUNT];  // index i = servo ID i+1
	uint16_t fresh_mask;                     // bit i set = servo i+1 answered the last SyncRead
} ServoBusState_t;

/* Init: binds the UART and releases every servo (they keep their torque across an MCU reset). */
void ServoBus_Init(UART_HandleTypeDef *huart);
/* Blocking ping of IDs 1..12 at boot; only the servos that answer join the SyncRead. */
void ServoBus_ScanPresent(void);

/* TIM ISR: start a bus cycle. */
void ServoBus_TickCycle(void);
/* Main loop. True when a bus cycle just completed: ServoBus_State() is fresh, stream it. */
bool ServoBus_Process(uint32_t now_us, uint16_t *cycle_us);
const ServoBusState_t *ServoBus_State(void);

/* Pending writes, sent at the start of the next cycle. */
void ServoBus_SetTargets(const uint8_t raw_le[2U * SERVO_BUS_COUNT]);
void ServoBus_SetTorque(uint16_t mask, uint16_t on);
/* Drop the pending targets and release all 12 servos on the next cycle. */
void ServoBus_ReleaseAll(void);
/* Min_Voltage (0x0F) of all 12 servos [0.1 V], in RAM only (EEPROM locked): over the bus voltage, the
   voltage alarm lights the servo LED. Never cuts the torque while Unload_Condition (0x13) bit 0 is clear. */
void ServoBus_SetMinVoltage(uint8_t decivolts);

/* Blocking configuration commands (wait for the bus, run one transaction): up to ~225 ms. */
bool ServoBus_Ping(uint8_t id);
bool ServoBus_SetId(uint8_t old_id, uint8_t new_id);
bool ServoBus_WriteRegister(uint8_t id, uint8_t reg, uint8_t size, uint16_t value);
bool ServoBus_ReadRegister(uint8_t id, uint8_t reg, uint8_t size, uint8_t *out);

/* Route from the HAL UART callbacks for the servo UART. */
void ServoBus_UartTxCplt(void);
void ServoBus_UartRxEvent(uint16_t size);
void ServoBus_UartError(void);

#endif /* SERVO_BUS_H */

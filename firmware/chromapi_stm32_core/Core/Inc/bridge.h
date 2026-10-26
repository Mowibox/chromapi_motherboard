/* SPDX-License-Identifier: Apache-2.0 */
#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "mahony.h"

#define BRIDGE_SYNC_1 0x55
#define BRIDGE_SYNC_2 0xAA

// Answered by GET_VERSION. Bump BRIDGE_PROTOCOL_VERSION whenever a frame layout or a command's
// meaning changes: the SDK refuses to drive a firmware speaking another protocol.
#define BRIDGE_PROTOCOL_VERSION 1
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  1
#define FIRMWARE_VERSION_PATCH  0

typedef enum {
	PING_BRIDGE          = 0x01,
	SET_POSITIONS        = 0x02,
	SET_TORQUE           = 0x03,  // u16 mask (bit i = servo i+1), u16 on-bits
	PING_SERVO           = 0x04,
	SET_SERVO_ID         = 0x06,
	SET_LED_COLOR        = 0x07,
	SET_LED_RING_BULK    = 0x08,
	WRITE_SERVO_REGISTER = 0x0A,
	READ_SERVO_REGISTER  = 0x0B,
	GET_VERSION          = 0x0C,  // -> BRIDGE_VERSION
} Command;

typedef enum {
	BRIDGE_OK              = 0x80,
	BRIDGE_ERROR           = 0x81,
	BRIDGE_STATE_SNAPSHOT  = 0x83,
	BRIDGE_SERVO_REG_VALUE = 0x84,
	BRIDGE_VERSION         = 0x85,  // protocol, major, minor, patch
} Response;

typedef enum {
	LED_RING_ALL    = 0x01,  // r,g,b
	LED_RING_SINGLE = 0x02,  // index,r,g,b
} LedTarget;

typedef struct __attribute__((packed)) {
	uint8_t  seq;       // +1 per bus cycle, a gap on the Pi side = a dropped frame
	uint32_t t_us;      // MCU time the servo read completed [µs], wraps every ~71 min
	uint16_t cycle_us;  // duration of that bus cycle, write + read [µs]

	int32_t bus_uV;
	int32_t current_uA;
	int32_t power_uW;

	struct __attribute__((packed)) {
		uint16_t position;
		int16_t  speed;
		int16_t  load;
		uint8_t  temperature;
		uint8_t  voltage;
		int16_t  current;
	} servos[12];

	int16_t imu_acc[3];        // m/s² * 100
	int16_t imu_gyro[3];       // raw gyro, rad/s * 1000
	int16_t imu_gyro_corr[3];  // bias-corrected gyro, rad/s * 1000
	int16_t imu_quat[4];       // w,x,y,z * 32767

	uint8_t  switches_mask;
	uint16_t servo_fresh_mask; // bit i set = servo i+1 answered this cycle
} RobotFeedback_t;

// Power reading streamed in the snapshot (INA226, refreshed by the application at 10 Hz)
typedef struct {
	int32_t bus_uV;
	int32_t current_uA;
	int32_t power_uW;
} PowerReading_t;

void Bridge_Init(UART_HandleTypeDef *huart);
void Bridge_Process(void);
void Bridge_CheckLink(void);
void Bridge_SendStateSnapshot(uint32_t t_us, uint16_t cycle_us,
		const BMI088 *imu, const MahonyFilter_t *ahrs, const PowerReading_t *power);

void Bridge_TxCpltCallback(void);
void Bridge_RxEventCallback(uint16_t size);
void Bridge_ErrorCallback(void);

#endif /* BRIDGE_H */

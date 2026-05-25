#ifndef BRIDGE_H
#define BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

#define BRIDGE_SYNC_1 0x55
#define BRIDGE_SYNC_2 0xAA

typedef enum {
	PING_BRIDGE    = 0x01,
	SET_POSITIONS   = 0x02,
	STATE_FEEDBACK  = 0x03,
	PING_SERVO      = 0x04,
	GET_SERVO_INFO  = 0x05,
	SET_SERVO_ID    = 0x06,
	SET_LED_COLOR   = 0x07,
	GET_POWER       = 0x08
} Command;

typedef enum {
	BRIDGE_OK              = 0x80,
	BRIDGE_ERROR           = 0x81,
	BRIDGE_POWER_READING   = 0x82,
	BRIDGE_STATE_SNAPSHOT  = 0x83
} Response;

typedef struct __attribute__((packed)) {
	int32_t bus_uV;
	int32_t current_uA;

	struct __attribute__((packed)) {
		uint16_t position;
		int16_t  speed;
		int16_t  load;
		uint8_t  temperature;
		uint8_t  voltage;
	} servos[12];

	int16_t imu_acc[3];
	int16_t imu_gyro[3];

	uint8_t switches_mask;
} RobotFeedback_t;

extern RobotFeedback_t g_robot_state;

void Bridge_Init(UART_HandleTypeDef *huart);
void Bridge_Process(void);
void Bridge_UpdateBatteryLed(void);

void Bridge_TxCpltCallback(void);
void Bridge_RxEventCallback(uint16_t size);
void Bridge_ErrorCallback(void);

#endif /* BRIDGE_H */

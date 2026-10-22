/* SPDX-License-Identifier: Apache-2.0 */
#include "bridge.h"
#include <string.h>
#include <ws2812b.h>

#include "sts3215_protocol.h"
#include "sts3215_regs.h"
#include "sts3215_hal.h"
#include "ina226.h"
#include "bmi088.h"
#include "mahony.h"

extern STS3215_HAL_Handle_t hservo;
extern AutoFox_INA226 gINA226;
extern BMI088 gIMU;
extern MahonyFilter_t g_mahony;
extern volatile uint8_t g_reply_received;
extern volatile uint8_t g_reply_data_len;
extern volatile uint8_t g_reply_data[];
extern volatile servo_txn_type_t g_servo_txn;

#define BRIDGE_RX_BUF_SIZE 256
#define BRIDGE_TX_BUF_SIZE 256
#define BRIDGE_TX_QUEUE_LEN 4

#define BRIDGE_MAX_FRAME_SIZE  64U
#define BRIDGE_FRAME_BUF_SIZE  (BRIDGE_RX_BUF_SIZE + BRIDGE_MAX_FRAME_SIZE)

static UART_HandleTypeDef *bridge_uart;
static uint8_t rx_buf[BRIDGE_RX_BUF_SIZE];
typedef struct {
	uint8_t  data[BRIDGE_TX_BUF_SIZE];
	uint16_t len;
} bridge_tx_frame_t;

static bridge_tx_frame_t tx_queue[BRIDGE_TX_QUEUE_LEN];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;
static volatile bool    tx_busy = false;

volatile uint32_t g_bridge_tx_overflow_count = 0;
static volatile uint32_t g_bridge_tx_dma_fail_count = 0;

static volatile uint16_t rx_head = 0;
static uint16_t rx_tail = 0;

static uint8_t  frame_buf[BRIDGE_FRAME_BUF_SIZE];
static uint16_t frame_buf_len = 0;

static bool s_pending_positions_valid = false;
static uint8_t s_pending_positions_raw[24];
static uint16_t s_pending_torque_mask = 0;
static uint16_t s_pending_torque_on = 0;
static int32_t s_cached_bus_uV = 0;
static int32_t s_cached_current_uA = 0;
static int32_t s_cached_power_uW = 0;

RobotFeedback_t g_robot_state = {0};

static uint32_t g_led_override_until_ms = 0;

#define VBAT_MAX_UV      8400000L   // 8.4V
#define VBAT_GREEN_UV    7800000L   // 7.8V
#define VBAT_YELLOW_UV   7400000L   // 7.4V
#define VBAT_ORANGE_UV   7000000L   // 7.0V
#define VBAT_RED_UV      6800000L   // 6.8V
#define VBAT_CRIT_UV     6600000L   // < 6.6V

#define LED_OVERRIDE_DURATION_MS  5000U

// CRC-16/CCITT-FALSE, matches Python's binascii.crc_hqx(data, 0xFFFF).
static uint16_t crc16(const uint8_t *data, uint16_t len) {
	uint16_t crc = 0xFFFF;
	while (len--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (uint8_t b = 0; b < 8; b++) {
			crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static void tx_start_next(void) {
	if (tx_head == tx_tail) {
		tx_busy = false;
		return;
	}
	tx_busy = true;
	if (HAL_UART_Transmit_DMA(bridge_uart, tx_queue[tx_head].data, tx_queue[tx_head].len) != HAL_OK) {
		tx_busy = false;
		g_bridge_tx_dma_fail_count++;
	}
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len) {
	uint8_t next = (uint8_t)((tx_tail + 1U) % BRIDGE_TX_QUEUE_LEN);
	if (next == tx_head) {
		g_bridge_tx_overflow_count++;
		return;
	}

	uint8_t *d = tx_queue[tx_tail].data;
	d[0] = BRIDGE_SYNC_1;
	d[1] = BRIDGE_SYNC_2;
	d[2] = payload_len + 1;
	d[3] = cmd;
	if (payload_len > 0 && payload != NULL) {
		memcpy(&d[4], payload, payload_len);
	}
	uint16_t crc = crc16(&d[2], payload_len + 2U);
	d[4 + payload_len] = (uint8_t)(crc & 0xFF);
	d[5 + payload_len] = (uint8_t)(crc >> 8);
	tx_queue[tx_tail].len = payload_len + 6U;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	tx_tail = next;
	if (!tx_busy) {
		tx_start_next();
	}
	__set_PRIMASK(primask);
}

static void send_ack(bool ok) {
	send_frame(ok ? BRIDGE_OK : BRIDGE_ERROR, NULL, 0);
}

static void handle_cmd_set_positions(const uint8_t *payload, uint8_t len) {
	if (len < 24) { send_ack(false); return; } // 12 * uint16_t = 24 bytes

	memcpy(s_pending_positions_raw, payload, sizeof(s_pending_positions_raw));
	s_pending_positions_valid = true;
}

static void handle_cmd_set_torque(const uint8_t *payload, uint8_t len) {
	if (len < 4) { send_ack(false); return; }

	uint16_t mask = (uint16_t)(payload[0] | (payload[1] << 8)) & 0x0FFFU;
	uint16_t on   = (uint16_t)(payload[2] | (payload[3] << 8));
	s_pending_torque_on   = (uint16_t)((s_pending_torque_on & ~mask) | (on & mask));
	s_pending_torque_mask |= mask;
}

bool Bridge_DispatchPendingWrites(void) {
	STS3215_SyncEntry_t entries[12];
	uint8_t n = 0;
	uint8_t reg, len;

	if (s_pending_positions_valid) {
		s_pending_positions_valid = false;
		for (uint8_t i = 0; i < 12; i++, n++) {
			entries[n].id = i + 1;
			entries[n].data[0] = s_pending_positions_raw[i * 2];
			entries[n].data[1] = s_pending_positions_raw[i * 2 + 1];
		}
		reg = STS3215_REG_TARGET_POS;
		len = 2;
	} else if (s_pending_torque_mask != 0U) {
		for (uint8_t i = 0; i < 12; i++) {
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

	uint8_t sync_tx_buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildSyncWrite(sync_tx_buf, sizeof(sync_tx_buf), reg, len, entries, n);
	if (frame_len > 0) {
		g_servo_txn = SERVO_TXN_COMMAND;
		if (STS3215_HAL_SendFrame(&hservo, sync_tx_buf, frame_len, true, 0) != STS3215_OK) {
			g_servo_txn = SERVO_TXN_NONE;
		}
	}
	return true;
}

static void handle_cmd_set_led_color(const uint8_t *payload, uint8_t len) {
	if (len < 1) { send_ack(false); return; }

	uint8_t target = payload[0];

	switch (target) {
	case LED_RING_ALL: {
		if (len < 4) { send_ack(false); return; }
		led_set_ring_all_RGB(payload[1], payload[2], payload[3]);
		break;
	}
	case LED_RING_SINGLE: {
		if (len < 5) { send_ack(false); return; }
		uint8_t idx = payload[1];
		if (idx >= LED_RING_COUNT) { send_ack(false); return; }
		led_set_RGB(LED_RING_START_INDEX + idx, payload[2], payload[3], payload[4]);
		break;
	}
	default:
		send_ack(false);
		return;
	}

	led_render();
	g_led_override_until_ms = HAL_GetTick() + LED_OVERRIDE_DURATION_MS;
	send_ack(true);
}

static void handle_cmd_set_led_ring_bulk(const uint8_t *payload, uint8_t len) {
	if (len < (LED_RING_COUNT * 3)) { send_ack(false); return; }

	for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
		uint8_t r = payload[i * 3 + 0];
		uint8_t g = payload[i * 3 + 1];
		uint8_t b = payload[i * 3 + 2];
		led_set_RGB(LED_RING_START_INDEX + i, r, g, b);
	}

	led_render();
	g_led_override_until_ms = HAL_GetTick() + LED_OVERRIDE_DURATION_MS;
	send_ack(true);
}

void Bridge_UpdateCachedPower(void) {
	s_cached_bus_uV     = AutoFox_INA226_GetBusVoltage_uV(&gINA226);
	s_cached_current_uA = AutoFox_INA226_GetCurrent_uA(&gINA226);
	s_cached_power_uW   = AutoFox_INA226_GetPower_uW(&gINA226);
}

void Bridge_SendStateSnapshot(uint32_t t_us, uint16_t cycle_us) {
	g_robot_state.seq++;
	g_robot_state.t_us     = t_us;
	g_robot_state.cycle_us = cycle_us;

	g_robot_state.bus_uV     = s_cached_bus_uV;
	g_robot_state.current_uA = s_cached_current_uA;
	g_robot_state.power_uW   = s_cached_power_uW;

	for (uint8_t i = 0; i < 3; i++) {
		g_robot_state.imu_acc[i]  = (int16_t)(gIMU.acc_mps2[i] * 100.0f);
		g_robot_state.imu_gyro[i] = (int16_t)(gIMU.gyr_rps[i]  * 1000.0f);
	}
	g_robot_state.imu_gyro_corr[0] = (int16_t)(g_mahony.wx * 1000.0f);
	g_robot_state.imu_gyro_corr[1] = (int16_t)(g_mahony.wy * 1000.0f);
	g_robot_state.imu_gyro_corr[2] = (int16_t)(g_mahony.wz * 1000.0f);
	float q[4] = {g_mahony.q0, g_mahony.q1, g_mahony.q2, g_mahony.q3};
	for (uint8_t i = 0; i < 4; i++) {
		int32_t raw = (int32_t)(q[i] * 32767.0f);
		if (raw > 32767) raw = 32767;
		if (raw < -32768) raw = -32768;
		g_robot_state.imu_quat[i] = (int16_t)raw;
	}

	uint8_t tl = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t tr = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t bl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t br = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) ? 0 : 1;

	g_robot_state.switches_mask = (tl << 0) | (tr << 1) | (bl << 2) | (br << 3);

	send_frame(BRIDGE_STATE_SNAPSHOT, (uint8_t*)&g_robot_state, sizeof(RobotFeedback_t));
}

static bool wait_servo_idle(uint32_t timeout_ms)
{
	uint32_t t0 = HAL_GetTick();
	while (!STS3215_HAL_IsIdle(&hservo)) {
		STS3215_HAL_Process(&hservo);
		if ((HAL_GetTick() - t0) > timeout_ms) {
			STS3215_HAL_Abort(&hservo);
			return false;
		}
	}
	return true;
}

static bool claim_servo_bus(void)
{
	if (!wait_servo_idle(55)) return false;
	hservo.last_error = STS3215_HAL_ERR_NONE;
	g_servo_txn = SERVO_TXN_COMMAND;
	return true;
}

static bool servo_txn(const uint8_t *buf, int16_t frame_len, bool broadcast, uint8_t replies, uint32_t timeout_ms)
{
	if (frame_len <= 0) return false;
	hservo.last_error = STS3215_HAL_ERR_NONE;
	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, broadcast, replies) != STS3215_OK) return false;
	return wait_servo_idle(timeout_ms) && hservo.last_error == STS3215_HAL_ERR_NONE;
}

static void handle_cmd_set_servo_id(const uint8_t *payload, uint8_t len)
{
	if (len < 2 || !claim_servo_bus()) { send_ack(false); return; }

	uint8_t old_id = payload[0];
	uint8_t new_id = payload[1];
	uint8_t buf[STS3215_TX_BUF_SIZE];
	bool bcast = (old_id == STS3215_BROADCAST_ID);

	if (!servo_txn(buf, STS3215_BuildUnlockEEPROM(buf, sizeof(buf), old_id), bcast, 0, 50) ||
	    !servo_txn(buf, STS3215_BuildWrite1B(buf, sizeof(buf), old_id, STS3215_REG_ID, new_id), bcast, 0, 50) ||
	    !servo_txn(buf, STS3215_BuildLockEEPROM(buf, sizeof(buf), STS3215_BROADCAST_ID), true, 0, 20)) {
		send_ack(false);
		return;
	}

	HAL_Delay(20);

	g_reply_received = 0;
	bool ok = servo_txn(buf, STS3215_BuildPing(buf, sizeof(buf), new_id), false, 1, 30);
	send_ack(ok && g_reply_received == 1);
}

static void handle_cmd_ping_servo(const uint8_t *payload, uint8_t len)
{
	if (len < 1 || !claim_servo_bus()) { send_ack(false); return; }

	uint8_t buf[STS3215_TX_BUF_SIZE];
	g_reply_received = 0;
	bool ok = servo_txn(buf, STS3215_BuildPing(buf, sizeof(buf), payload[0]), false, 1, 20);
	send_ack(ok && g_reply_received == 1);
}

static void handle_cmd_write_servo_register(const uint8_t *payload, uint8_t len) {
	if (len < 5 || !claim_servo_bus()) { send_ack(false); return; }

	uint8_t id = payload[0];
	uint8_t reg = payload[1];
	uint8_t size = payload[2];
	uint16_t val = (uint16_t)payload[3] | ((uint16_t)payload[4] << 8);

	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = 0;
	if (size == 1) {
		frame_len = STS3215_BuildWrite1B(buf, sizeof(buf), id, reg, (uint8_t)val);
	} else if (size == 2) {
		frame_len = STS3215_BuildWrite2B(buf, sizeof(buf), id, reg, val);
	}

	bool is_broadcast = (id == STS3215_BROADCAST_ID);
	send_ack(servo_txn(buf, frame_len, is_broadcast, is_broadcast ? 0U : 1U, 50));
}

static void handle_cmd_read_servo_register(const uint8_t *payload, uint8_t len) {
	if (len < 3 || payload[2] == 0 || payload[2] > STS3215_REPLY_DATA_MAX || !claim_servo_bus()) {
		send_ack(false);
		return;
	}

	uint8_t id = payload[0];
	uint8_t reg = payload[1];
	uint8_t size = payload[2];

	uint8_t buf[STS3215_TX_BUF_SIZE];
	g_reply_received = 0;
	bool ok = servo_txn(buf, STS3215_BuildRead(buf, sizeof(buf), id, reg, size), false, 1, 10);

	if (ok && g_reply_received && g_reply_data_len >= size) {
		send_frame(BRIDGE_SERVO_REG_VALUE, (uint8_t*)g_reply_data, size);
	} else {
		send_ack(false);
	}
}


static void parse_rx_buffer(void) {
	uint16_t i = 0;
	if (frame_buf_len < 2) {
		return;
	}

	while (i + 1 < frame_buf_len) {
		if (frame_buf[i] != BRIDGE_SYNC_1 || frame_buf[i+1] != BRIDGE_SYNC_2) {
			i++;
			continue;
		}

		if (i + 2 >= frame_buf_len) {
			break;
		}

		uint8_t  expected_len     = frame_buf[i+2];
		uint16_t total_frame_size = 5U + (uint16_t)expected_len;

		if (expected_len == 0 || total_frame_size > BRIDGE_MAX_FRAME_SIZE) {
			i++;
			continue;
		}

		if (i + total_frame_size > frame_buf_len) {
			break;
		}

		uint16_t received_crc = (uint16_t)frame_buf[i + total_frame_size - 2]
		                      | ((uint16_t)frame_buf[i + total_frame_size - 1] << 8);

		if (received_crc == crc16(&frame_buf[i+2], expected_len + 1U)) {
			uint8_t cmd = frame_buf[i+3];
			const uint8_t *payload = &frame_buf[i+4];
			uint8_t payload_len = expected_len - 1;

			switch (cmd) {
			case PING_BRIDGE:           send_ack(true); break;
			case SET_POSITIONS:         handle_cmd_set_positions(payload, payload_len); break;
			case SET_TORQUE:            handle_cmd_set_torque(payload, payload_len); break;
			case PING_SERVO:            handle_cmd_ping_servo(payload, payload_len); break;
			case SET_SERVO_ID:          handle_cmd_set_servo_id(payload, payload_len); break;
			case SET_LED_COLOR:         handle_cmd_set_led_color(payload, payload_len); break;
			case SET_LED_RING_BULK:     handle_cmd_set_led_ring_bulk(payload, payload_len); break;
			case WRITE_SERVO_REGISTER:  handle_cmd_write_servo_register(payload, payload_len); break;
			case READ_SERVO_REGISTER:   handle_cmd_read_servo_register(payload, payload_len); break;
			default:                    send_ack(false); break;
			}
			i += total_frame_size;
		} else {
			i++;
		}
	}

	uint16_t remaining = frame_buf_len - i;
	if (remaining > 0) {
		memmove(frame_buf, &frame_buf[i], remaining);
	}
	frame_buf_len = remaining;
}

static void start_rx(void) {
	rx_head = 0;
	rx_tail = 0;
	__HAL_UART_CLEAR_FLAG(bridge_uart, UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
	HAL_UARTEx_ReceiveToIdle_DMA(bridge_uart, rx_buf, BRIDGE_RX_BUF_SIZE);
	CLEAR_BIT(bridge_uart->Instance->CR3, USART_CR3_EIE);
}

void Bridge_Init(UART_HandleTypeDef *huart) {
	bridge_uart = huart;
	tx_busy = false;
	frame_buf_len = 0;
	start_rx();
}

void Bridge_Process(void) {
	uint16_t head = rx_head;
	if (head == rx_tail) return;

	while (rx_tail != head) {
		if (frame_buf_len >= BRIDGE_FRAME_BUF_SIZE) frame_buf_len = 0;
		frame_buf[frame_buf_len++] = rx_buf[rx_tail];
		rx_tail = (uint16_t)((rx_tail + 1U) % BRIDGE_RX_BUF_SIZE);
	}

	parse_rx_buffer();
}


void Bridge_UpdateBatteryLed(void)
{
	int32_t uV = s_cached_bus_uV;  // refreshed at 10 Hz

	uint8_t r, g, b;

	if (uV >= VBAT_GREEN_UV) {
		int32_t ratio = ((uV - VBAT_GREEN_UV) * 50L) / (VBAT_MAX_UV - VBAT_GREEN_UV);
		if (ratio > 50) ratio = 50;
		r = 0;
		g = 50;
		b = (uint8_t)ratio;
	}
	else if (uV >= VBAT_YELLOW_UV) {
		int32_t ratio = ((uV - VBAT_YELLOW_UV) * 50L) / (VBAT_GREEN_UV - VBAT_YELLOW_UV);
		if (ratio > 50) ratio = 50;
		r = (uint8_t)(50L - ratio);
		g = 50;
		b = 0;
	}
	else if (uV >= VBAT_ORANGE_UV) {
		int32_t ratio = ((uV - VBAT_ORANGE_UV) * 50L) / (VBAT_YELLOW_UV - VBAT_ORANGE_UV);
		if (ratio > 50) ratio = 50;
		r = 50;
		g = (uint8_t)(15L + ratio * 35L / 50L);
		b = 0;
	}
	else if (uV >= VBAT_RED_UV) {
		int32_t ratio = ((uV - VBAT_RED_UV) * 50L) / (VBAT_ORANGE_UV - VBAT_RED_UV);
		if (ratio > 50) ratio = 50;
		r = 50;
		g = (uint8_t)(ratio * 15L / 50L);
		b = 0;
	}
	else if (uV >= VBAT_CRIT_UV) {
		r = 50; g = 0; b = 0;
	}
	else {
		uint8_t blink = (HAL_GetTick() % 200U) < 100U ? 50 : 0;
		r = blink; g = 0; b = 0;
	}

	led_set_RGB(LED_STATUS_START_INDEX, r, g, b);
	led_render();
}

void Bridge_TxCpltCallback(void) {
	tx_head = (uint8_t)((tx_head + 1U) % BRIDGE_TX_QUEUE_LEN);
	tx_start_next();
}

void Bridge_RxEventCallback(uint16_t size) {
	rx_head = (size >= BRIDGE_RX_BUF_SIZE) ? 0U : size;
}

void Bridge_ErrorCallback(void) {
	HAL_UART_DMAStop(bridge_uart);
	tx_busy = false;
	start_rx();
}

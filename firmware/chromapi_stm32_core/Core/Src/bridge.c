/* SPDX-License-Identifier: Apache-2.0 */
#include "bridge.h"
#include <string.h>
#include <ws2812b.h>

#include "servo_bus.h"

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

_Static_assert((BRIDGE_RX_BUF_SIZE & (BRIDGE_RX_BUF_SIZE - 1)) == 0, "RX buffer size must be a power of two");
static volatile uint32_t rx_written = 0;    // ISR
static uint16_t rx_last_pos = 0;            // ISR
static volatile bool rx_restarted = false;  // ISR -> main: DMA re-armed at index 0
static uint32_t rx_read = 0;                // main loop
volatile uint32_t g_bridge_rx_overflow_count = 0;

static uint8_t  frame_buf[BRIDGE_FRAME_BUF_SIZE];
static uint16_t frame_buf_len = 0;

static RobotFeedback_t s_snapshot = {0};

// Link watchdog: the Pi streams targets at 100 Hz, so 100 ms of silence = 10 lost frames.
#define BRIDGE_LINK_TIMEOUT_MS  100U
static bool     s_link_up = false;
static uint32_t s_last_rx_ms = 0;
volatile uint32_t g_bridge_link_lost_count = 0;

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
	if (len < 2U * SERVO_BUS_COUNT) { send_ack(false); return; } // 12 * uint16_t = 24 bytes

	ServoBus_SetTargets(payload);
}

static void handle_cmd_set_torque(const uint8_t *payload, uint8_t len) {
	if (len < 4) { send_ack(false); return; }

	ServoBus_SetTorque((uint16_t)(payload[0] | (payload[1] << 8)), (uint16_t)(payload[2] | (payload[3] << 8)));
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
	send_ack(true);
}

void Bridge_SendStateSnapshot(uint32_t t_us, uint16_t cycle_us,
		const BMI088 *imu, const MahonyFilter_t *ahrs, const PowerReading_t *power) {
	__COMPILER_BARRIER();
	const ServoBusState_t *bus = ServoBus_State();
	for (uint8_t i = 0; i < SERVO_BUS_COUNT; i++) {
		s_snapshot.servos[i].position    = bus->servo[i].position;
		s_snapshot.servos[i].speed       = bus->servo[i].speed;
		s_snapshot.servos[i].load        = bus->servo[i].load;
		s_snapshot.servos[i].temperature = bus->servo[i].temperature;
		s_snapshot.servos[i].voltage     = bus->servo[i].voltage;
		s_snapshot.servos[i].current     = bus->servo[i].current;
	}
	s_snapshot.servo_fresh_mask = bus->fresh_mask;

	s_snapshot.seq++;
	s_snapshot.t_us     = t_us;
	s_snapshot.cycle_us = cycle_us;

	s_snapshot.bus_uV     = power->bus_uV;
	s_snapshot.current_uA = power->current_uA;
	s_snapshot.power_uW   = power->power_uW;

	for (uint8_t i = 0; i < 3; i++) {
		s_snapshot.imu_acc[i]  = (int16_t)(imu->acc_mps2[i] * 100.0f);
		s_snapshot.imu_gyro[i] = (int16_t)(imu->gyr_rps[i]  * 1000.0f);
	}
	s_snapshot.imu_gyro_corr[0] = (int16_t)(ahrs->wx * 1000.0f);
	s_snapshot.imu_gyro_corr[1] = (int16_t)(ahrs->wy * 1000.0f);
	s_snapshot.imu_gyro_corr[2] = (int16_t)(ahrs->wz * 1000.0f);
	float q[4] = {ahrs->q0, ahrs->q1, ahrs->q2, ahrs->q3};
	for (uint8_t i = 0; i < 4; i++) {
		int32_t raw = (int32_t)(q[i] * 32767.0f);
		if (raw > 32767) raw = 32767;
		if (raw < -32768) raw = -32768;
		s_snapshot.imu_quat[i] = (int16_t)raw;
	}

	uint8_t tl = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t tr = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t bl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t br = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) ? 0 : 1;

	s_snapshot.switches_mask = (tl << 0) | (tr << 1) | (bl << 2) | (br << 3);

	send_frame(BRIDGE_STATE_SNAPSHOT, (uint8_t*)&s_snapshot, sizeof(RobotFeedback_t));
}

// Servo commands relayed to the bus: each one blocks the main loop until its transaction ends.
static void handle_cmd_ping_servo(const uint8_t *payload, uint8_t len) {
	send_ack(len >= 1 && ServoBus_Ping(payload[0]));
}

static void handle_cmd_set_servo_id(const uint8_t *payload, uint8_t len) {
	send_ack(len >= 2 && ServoBus_SetId(payload[0], payload[1]));
}

static void handle_cmd_write_servo_register(const uint8_t *payload, uint8_t len) {
	send_ack(len >= 5 && ServoBus_WriteRegister(payload[0], payload[1], payload[2],
			(uint16_t)payload[3] | ((uint16_t)payload[4] << 8)));
}

static void handle_cmd_read_servo_register(const uint8_t *payload, uint8_t len) {
	uint8_t value[SERVO_BUS_READ_MAX];
	if (len >= 3 && payload[2] <= SERVO_BUS_READ_MAX &&
			ServoBus_ReadRegister(payload[0], payload[1], payload[2], value)) {
		send_frame(BRIDGE_SERVO_REG_VALUE, value, payload[2]);
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
			s_link_up = true;
			s_last_rx_ms = HAL_GetTick();
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
	// The DMA restarts at index 0: realign the counter on it and tell the main loop to drop
	// whatever it has not read yet.
	rx_written = (rx_written + BRIDGE_RX_BUF_SIZE - 1U) & ~(uint32_t)(BRIDGE_RX_BUF_SIZE - 1U);
	rx_last_pos = 0;
	rx_restarted = true;
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
	uint32_t written = rx_written;
	if (rx_restarted) {  // checked after reading rx_written, which the ISR updates first
		rx_restarted = false;
		rx_read = rx_written;
		frame_buf_len = 0;
		return;
	}
	if (written == rx_read) return;

	if (written - rx_read > BRIDGE_RX_BUF_SIZE) {  // the DMA lapped us: what is left is garbage
		g_bridge_rx_overflow_count++;
		rx_read = written;
		frame_buf_len = 0;
		return;
	}

	while (rx_read != written) {
		if (frame_buf_len >= BRIDGE_FRAME_BUF_SIZE) frame_buf_len = 0;
		frame_buf[frame_buf_len++] = rx_buf[rx_read % BRIDGE_RX_BUF_SIZE];
		rx_read++;
	}

	parse_rx_buffer();
}

// No valid frame from the Pi for BRIDGE_LINK_TIMEOUT_MS: drop the pending targets and release
// all 12 servos on the next bus cycle, instead of holding the last pose under torque forever.
void Bridge_CheckLink(void) {
	if (!s_link_up || (HAL_GetTick() - s_last_rx_ms) < BRIDGE_LINK_TIMEOUT_MS) return;

	s_link_up = false;
	g_bridge_link_lost_count++;
	ServoBus_ReleaseAll();
}


void Bridge_TxCpltCallback(void) {
	tx_head = (uint8_t)((tx_head + 1U) % BRIDGE_TX_QUEUE_LEN);
	tx_start_next();
}

void Bridge_RxEventCallback(uint16_t size) {
	uint16_t pos = (size >= BRIDGE_RX_BUF_SIZE) ? 0U : size;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	rx_written += (uint16_t)(pos - rx_last_pos) & (BRIDGE_RX_BUF_SIZE - 1U);
	rx_last_pos = pos;
	__set_PRIMASK(primask);
}

void Bridge_ErrorCallback(void) {
	HAL_UART_DMAStop(bridge_uart);
	tx_busy = false;
	start_rx();
}

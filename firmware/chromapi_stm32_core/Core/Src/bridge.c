#include "bridge.h"
#include <string.h>
#include <ws2812b.h>

#include "sts3215_protocol.h"
#include "sts3215_regs.h"
#include "sts3215_hal.h"
#include "ina226.h"
#include "bmi088.h"

extern STS3215_HAL_Handle_t hservo;
extern AutoFox_INA226 gINA226;
extern BMI088 gIMU;
extern volatile uint8_t g_reply_received;
extern volatile uint8_t g_reply_data_len;
extern volatile uint8_t g_reply_data[];

#define BRIDGE_RX_BUF_SIZE 256
#define BRIDGE_TX_BUF_SIZE 256
#define BRIDGE_TX_QUEUE_LEN 3

#define BRIDGE_MAX_FRAME_SIZE  64U
#define BRIDGE_FRAME_BUF_SIZE  (BRIDGE_RX_BUF_SIZE + BRIDGE_MAX_FRAME_SIZE)

static UART_HandleTypeDef *bridge_uart;
static uint8_t rx_buf[BRIDGE_RX_BUF_SIZE];
typedef struct {
	uint8_t  data[BRIDGE_TX_BUF_SIZE];
	uint16_t len;
} bridge_tx_frame_t;

static bridge_tx_frame_t tx_queue[BRIDGE_TX_QUEUE_LEN];
static volatile uint8_t  tx_queue_head  = 0;
static volatile uint8_t  tx_queue_tail  = 0;
static volatile uint8_t  tx_queue_count = 0;

volatile uint32_t g_bridge_tx_overflow_count = 0;

static volatile bool rx_ready = false;
static volatile uint16_t rx_len = 0;
static volatile bool tx_busy = false;
static volatile uint32_t g_bridge_tx_dma_fail_count = 0;

static uint8_t  frame_buf[BRIDGE_FRAME_BUF_SIZE];
static uint16_t frame_buf_len = 0;

static bool wait_servo_bus_idle(uint32_t timeout_ms);

RobotFeedback_t g_robot_state = {0};

static uint32_t g_led_override_until_ms = 0;

#define VBAT_MAX_UV      8400000L   // 8.4V
#define VBAT_GREEN_UV    7800000L   // 7.8V
#define VBAT_YELLOW_UV   7400000L   // 7.4V
#define VBAT_ORANGE_UV   7000000L   // 7.0V
#define VBAT_RED_UV      6800000L   // 6.8V
#define VBAT_CRIT_UV     6600000L   // < 6.6V

#define LED_OVERRIDE_DURATION_MS  5000U

static uint8_t calculate_crc(const uint8_t *data, uint16_t len) {
	uint8_t crc = 0;
	for (uint16_t i = 0; i < len; i++) {
		crc ^= data[i];
	}
	return crc;
}

static void tx_queue_start_next(void) {
	if (tx_queue_count == 0) return;
	tx_busy = true;
	if (HAL_UART_Transmit_DMA(bridge_uart,
			tx_queue[tx_queue_head].data,
			tx_queue[tx_queue_head].len) != HAL_OK) {
		tx_busy = false;
		g_bridge_tx_dma_fail_count++;
	}
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len) {
	if (tx_queue_count >= BRIDGE_TX_QUEUE_LEN) {
		g_bridge_tx_overflow_count++;
		return;
	}

	bridge_tx_frame_t *slot = &tx_queue[tx_queue_tail];
	slot->data[0] = BRIDGE_SYNC_1;
	slot->data[1] = BRIDGE_SYNC_2;
	slot->data[2] = payload_len + 1;
	slot->data[3] = cmd;

	if (payload_len > 0 && payload != NULL) {
		memcpy(&slot->data[4], payload, payload_len);
	}

	slot->data[4 + payload_len] = calculate_crc(&slot->data[2], payload_len + 2);
	slot->len = payload_len + 5;

	tx_queue_tail = (tx_queue_tail + 1) % BRIDGE_TX_QUEUE_LEN;
	tx_queue_count++;

	if (!tx_busy) {
		tx_queue_start_next();
	}
}

static void send_ack(bool ok) {
	uint8_t status = ok ? BRIDGE_OK : BRIDGE_ERROR;
	send_frame(status, NULL, 0);
}

static void handle_cmd_set_positions(const uint8_t *payload, uint8_t len) {
	if (len < 24) { send_ack(false); return; } // 12 * uint16_t = 24 bytes

	if (!wait_servo_bus_idle(55)) {
		send_ack(false);
		return;
	}

	STS3215_SyncEntry_t entries[12];

	for (uint8_t i = 0; i < 12; i++) {
		entries[i].id = i + 1;
		entries[i].data[0] = payload[i * 2];
		entries[i].data[1] = payload[i * 2 + 1];
	}

	uint8_t sync_tx_buf[STS3215_TX_BUF_SIZE];

	int16_t frame_len = STS3215_BuildSyncWrite(
			sync_tx_buf,
			sizeof(sync_tx_buf),
			STS3215_REG_TARGET_POS,
			2,
			entries,
			12
	);

	bool sent_ok = false;
	if (frame_len > 0) {
		sent_ok = (STS3215_HAL_SendFrame(&hservo, sync_tx_buf, frame_len, true, 0) == STS3215_OK);
	}
	send_ack(sent_ok);
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

static void handle_cmd_get_power(void) {
	int32_t bus_uV = AutoFox_INA226_GetBusVoltage_uV(&gINA226);
	int32_t curr_uA = AutoFox_INA226_GetCurrent_uA(&gINA226);
	int32_t power_uW = AutoFox_INA226_GetPower_uW(&gINA226);

	uint8_t resp[12];

	resp[0] = (uint8_t)(bus_uV & 0xFF);
	resp[1] = (uint8_t)((bus_uV >> 8) & 0xFF);
	resp[2] = (uint8_t)((bus_uV >> 16) & 0xFF);
	resp[3] = (uint8_t)((bus_uV >> 24) & 0xFF);

	resp[4] = (uint8_t)(curr_uA & 0xFF);
	resp[5] = (uint8_t)((curr_uA >> 8) & 0xFF);
	resp[6] = (uint8_t)((curr_uA >> 16) & 0xFF);
	resp[7] = (uint8_t)((curr_uA >> 24) & 0xFF);

	resp[8] = (uint8_t)(power_uW & 0xFF);
	resp[9] = (uint8_t)((power_uW >> 8) & 0xFF);
	resp[10] = (uint8_t)((power_uW >> 16) & 0xFF);
	resp[11] = (uint8_t)((power_uW >> 24) & 0xFF);

	send_frame(BRIDGE_POWER_READING, resp, 12);
}

static void handle_cmd_feedback(void) {
	g_robot_state.bus_uV     = AutoFox_INA226_GetBusVoltage_uV(&gINA226);
	g_robot_state.current_uA = AutoFox_INA226_GetCurrent_uA(&gINA226);
	g_robot_state.power_uW   = AutoFox_INA226_GetPower_uW(&gINA226);

	BMI088_ReadAccelerometer(&gIMU);
	BMI088_ReadGyroscope(&gIMU);

	for (uint8_t i = 0; i < 3; i++) {
		g_robot_state.imu_acc[i]  = (int16_t)(gIMU.acc_mps2[i] * 100.0f);
		g_robot_state.imu_gyro[i] = (int16_t)(gIMU.gyr_rps[i]  * 1000.0f);
	}

	uint8_t tl = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t tr = (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t bl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET) ? 0 : 1;
	uint8_t br = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) ? 0 : 1;

	g_robot_state.switches_mask = (tl << 0) | (tr << 1) | (bl << 2) | (br << 3);

	send_frame(BRIDGE_STATE_SNAPSHOT, (uint8_t*)&g_robot_state, sizeof(RobotFeedback_t));
}

static bool wait_servo_idle(STS3215_HAL_Handle_t *h, uint32_t timeout_ms)
{
	uint32_t t0 = HAL_GetTick();

	while (!STS3215_HAL_IsIdle(h))
	{
		STS3215_HAL_Process(h);

		if ((HAL_GetTick() - t0) > timeout_ms)
		{
			STS3215_HAL_Abort(h);
			return false;
		}
	}
	return (h->last_error == STS3215_HAL_ERR_NONE);
}

extern volatile servo_txn_type_t g_servo_txn;

static bool wait_servo_bus_idle(uint32_t timeout_ms)
{
	uint32_t t0 = HAL_GetTick();
	while (!STS3215_HAL_IsIdle(&hservo)) {
		STS3215_HAL_Process(&hservo);
		if ((HAL_GetTick() - t0) > timeout_ms) {
			STS3215_HAL_Abort(&hservo);
			return false;
		}
	}
	hservo.last_error = STS3215_HAL_ERR_NONE;
	g_servo_txn = SERVO_TXN_COMMAND;
	return true;
}

static void handle_cmd_set_servo_id(const uint8_t *payload, uint8_t len)
{
	if (len < 2) { send_ack(false); return; }

	if (!wait_servo_bus_idle(55)) {
		send_ack(false);
		return;
	}

	uint8_t old_id = payload[0];
	uint8_t new_id = payload[1];
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len;

	const uint8_t target_id = (old_id == STS3215_BROADCAST_ID) ? STS3215_BROADCAST_ID : old_id;

	frame_len = STS3215_BuildUnlockEEPROM(buf, sizeof(buf), target_id);
	hservo.last_error = STS3215_HAL_ERR_NONE;
	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, (target_id == STS3215_BROADCAST_ID), 0) != STS3215_OK) {
		send_ack(false); return;
	}
	if (!wait_servo_idle(&hservo, 50)) { send_ack(false); return; }

	frame_len = STS3215_BuildWrite1B(buf, sizeof(buf), target_id, STS3215_REG_ID, new_id);
	hservo.last_error = STS3215_HAL_ERR_NONE;
	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, (target_id == STS3215_BROADCAST_ID), 0) != STS3215_OK) {
		send_ack(false); return;
	}
	if (!wait_servo_idle(&hservo, 50)) { send_ack(false); return; }

	frame_len = STS3215_BuildLockEEPROM(buf, sizeof(buf), STS3215_BROADCAST_ID);
	hservo.last_error = STS3215_HAL_ERR_NONE;
	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, true, 0) != STS3215_OK) {
		send_ack(false); return;
	}
	if (!wait_servo_idle(&hservo, 20)) { send_ack(false); return; }

	HAL_Delay(20);

	g_reply_received = 0;
	frame_len = STS3215_BuildPing(buf, sizeof(buf), new_id);
	hservo.last_error = STS3215_HAL_ERR_NONE;
	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, false, 1) != STS3215_OK) {
		send_ack(false); return;
	}
	if (!wait_servo_idle(&hservo, 30)) { send_ack(false); return; }

	send_ack(g_reply_received == 1);
}

static void handle_cmd_ping_servo(const uint8_t *payload, uint8_t len)
{
	if (len < 1) { send_ack(false); return; }

	if (!wait_servo_bus_idle(55)) {
		send_ack(false);
		return;
	}

	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildPing(buf, sizeof(buf), payload[0]);
	if (frame_len <= 0) { send_ack(false); return; }

	g_reply_received = 0;
	hservo.last_error = STS3215_HAL_ERR_NONE;

	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, false, 1) != STS3215_OK) {
		send_ack(false); return;
	}

	if (!wait_servo_idle(&hservo, 20)) {
		send_ack(false);
		return;
	}

	send_ack((hservo.last_error == STS3215_HAL_ERR_NONE) && (g_reply_received == 1));
}

static void handle_cmd_write_servo_register(const uint8_t *payload, uint8_t len) {
	if (len < 5) { send_ack(false); return; }

	if (!wait_servo_bus_idle(55)) {
		send_ack(false);
		return;
	}

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

	if (frame_len <= 0) { send_ack(false); return; }

	hservo.last_error = STS3215_HAL_ERR_NONE;
	bool is_broadcast = (id == STS3215_BROADCAST_ID);
	uint8_t expected_replies = is_broadcast ? 0U : 1U;

	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, is_broadcast, expected_replies) != STS3215_OK) {
		send_ack(false); return;
	}

	if (!wait_servo_idle(&hservo, 50)) {
		send_ack(false);
		return;
	}

	send_ack(hservo.last_error == STS3215_HAL_ERR_NONE);
}

static void handle_cmd_read_servo_register(const uint8_t *payload, uint8_t len) {
	if (len < 3) { send_ack(false); return; }

	if (!wait_servo_bus_idle(55)) {
		send_ack(false);
		return;
	}

	uint8_t id = payload[0];
	uint8_t reg = payload[1];
	uint8_t size = payload[2];

	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t frame_len = STS3215_BuildRead(buf, sizeof(buf), id, reg, size);

	if (frame_len <= 0) { send_ack(false); return; }

	g_reply_received = 0;
	hservo.last_error = STS3215_HAL_ERR_NONE;

	if (STS3215_HAL_SendFrame(&hservo, buf, frame_len, false, 1) == STS3215_OK) {
		uint32_t t0 = HAL_GetTick();
		while (!g_reply_received && (HAL_GetTick() - t0 < 10)) {
			STS3215_HAL_Process(&hservo);
		}

		if (g_reply_received && g_reply_data_len >= size) {
			send_frame(0x84, (uint8_t*)g_reply_data, size);
		} else {
			STS3215_HAL_Abort(&hservo);
			send_ack(false);
		}
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
		uint16_t total_frame_size = 4U + (uint16_t)expected_len;

		if (total_frame_size > BRIDGE_MAX_FRAME_SIZE) {
			i++;
			continue;
		}

		if (i + total_frame_size > frame_buf_len) {
			break; /* trame réellement incomplète : on attend l'IDLE suivant */
		}

		uint8_t received_crc = frame_buf[i + total_frame_size - 1];
		uint8_t calc_crc     = calculate_crc(&frame_buf[i+2], expected_len + 1);

		if (received_crc == calc_crc) {
			uint8_t cmd = frame_buf[i+3];
			const uint8_t *payload = &frame_buf[i+4];
			uint8_t payload_len = expected_len - 1;

			switch (cmd) {
			case PING_BRIDGE:           send_ack(true); break;
			case SET_POSITIONS:         handle_cmd_set_positions(payload, payload_len); break;
			case STATE_FEEDBACK:        handle_cmd_feedback(); break;
			case PING_SERVO:            handle_cmd_ping_servo(payload, payload_len); break;
			case GET_SERVO_INFO:        send_ack(false); break;
			case SET_SERVO_ID:          handle_cmd_set_servo_id(payload, payload_len); break;
			case SET_LED_COLOR:         handle_cmd_set_led_color(payload, payload_len); break;
			case SET_LED_RING_BULK:     handle_cmd_set_led_ring_bulk(payload, payload_len); break;
			case GET_POWER:             handle_cmd_get_power(); break;
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


void Bridge_Init(UART_HandleTypeDef *huart) {
	bridge_uart = huart;
	tx_busy = false;
	rx_ready = false;
	frame_buf_len = 0;
	HAL_UARTEx_ReceiveToIdle_DMA(bridge_uart, rx_buf, BRIDGE_RX_BUF_SIZE);
}

void Bridge_Process(void) {
	if (!rx_ready) return;

	uint16_t n = rx_len;

	if (frame_buf_len + n <= BRIDGE_FRAME_BUF_SIZE) {
		memcpy(&frame_buf[frame_buf_len], rx_buf, n);
		frame_buf_len += n;
	} else {
		frame_buf_len = 0;
	}

	rx_ready = false;

	HAL_UARTEx_ReceiveToIdle_DMA(bridge_uart, rx_buf, BRIDGE_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(bridge_uart->hdmarx, DMA_IT_HT);

	parse_rx_buffer();

	if (!tx_busy && tx_queue_count > 0) {
		tx_queue_start_next();
	}
}


void Bridge_UpdateBatteryLed(void)
{
	int32_t uV = AutoFox_INA226_GetBusVoltage_uV(&gINA226);

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
	tx_queue_head = (tx_queue_head + 1) % BRIDGE_TX_QUEUE_LEN;
	if (tx_queue_count > 0) {
		tx_queue_count--;
	}
	tx_busy = false;
	tx_queue_start_next();
}

void Bridge_RxEventCallback(uint16_t size) {
	rx_len = size;
	rx_ready = true;
}

void Bridge_ErrorCallback(void) {
	HAL_UART_DMAStop(bridge_uart);
	rx_ready = false;
	tx_busy = false;
	HAL_UARTEx_ReceiveToIdle_DMA(bridge_uart, rx_buf, BRIDGE_RX_BUF_SIZE);
}

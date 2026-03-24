#include "logic.h"

#include <stdint.h>

#include "ibus.h"
#include "main.h"
#include "can.h"
#include "util.h"


static logic_state_t* g_logic_state_ptr = NULL;


void logic_init(logic_state_t* state) {
	state->mode = LOGIC_MODE_STARTING;
	state->boot_time = HAL_GetTick();
	state->estop_triggered_time = option_u32_none(); // not currently triggered
	debounce_controller_init(
		&state->estop_debounce,
		false,
		ESTOP_RISING_DEBOUNCE,
		ESTOP_FALLING_DEBOUNCE,
		ESTOP_ACCUMULATING_DEBOUNCE
	);

	ibus_init(&state->ibus);
	state->last_can_tx_time = 0;

	g_logic_state_ptr = state;
}

void logic_handle_control(const can_control_msg_t* cmd) {
	if (g_logic_state_ptr == NULL) {
		return;
	}

	g_logic_state_ptr->can_current_throttle = cmd->throttle;
	g_logic_state_ptr->can_current_steering = cmd->steering;
	g_logic_state_ptr->last_control_timestamp = HAL_GetTick();
}

void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef *sbus_huart,
	CAN_HandleTypeDef *hcan
) {
	// Process iBUS data
	ibus_process(&state->ibus, sbus_huart);

	uint32_t now = HAL_GetTick();

	// Check for remote ESTOP trigger
	uint16_t estop_channel_value = state->ibus.channels[IBUS_CHANNEL_ESTOP];
	bool estop_raw_high = estop_channel_value >= ESTOP_PWM_THRESHOLD;
	bool estop_debounced_high = debounce_controller_update(&state->estop_debounce, estop_raw_high, now);
	if (estop_debounced_high) {
		state->estop_triggered_time = option_u32_some(now);
		state->mode = LOGIC_MODE_ESTOPPED;

		HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
	}

	// Boot state machine logic
	switch (state->mode) {
		case LOGIC_MODE_STARTING:
			if (now >= state->boot_time + PRECHARGE_START_DELAY) {
				state->mode = LOGIC_MODE_PRECHARGING;
				// Start precharge: Precharge EN goes high
				HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			}
			break;

		case LOGIC_MODE_PRECHARGING:
			if (now >= state->boot_time + PRECHARGE_START_DELAY + PRECHARGE_DURATION) {
				state->mode = LOGIC_MODE_CONTACTOR_CLOSING;
				// Close contactor: Main Coil EN goes high
				HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);
			}
			break;

		case LOGIC_MODE_CONTACTOR_CLOSING:
			if (now >= state->boot_time + PRECHARGE_START_DELAY + PRECHARGE_DURATION + CONTACTOR_CLOSED_DELAY) {
				state->mode = LOGIC_MODE_RUNNING;
				// Contactor closed = precharge complete, Precharge EN can go low
				HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			}
			break;

		case LOGIC_MODE_RUNNING:
			// Normal operation, nothing to do here for now
			break;
		case LOGIC_MODE_ESTOPPED:
			// For safety, force set main coil and precharge off
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (option_u32_is_some(state->estop_triggered_time)
				&& (now >= option_u32_unwrap(state->estop_triggered_time) + ESTOP_TRIGGERED_DELAY)) {
				// After waiting for ESTOP_TRIGGERED_DELAY, restart precharge sequence
				state->mode = LOGIC_MODE_STARTING;
				state->estop_triggered_time = option_u32_none(); // reset estop triggered time
				// Reset to booting state (both precharge and main coil off),
				// then the logic will set precharge on in the next step of the state machine
				HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			}
			break;
	}

	// Periodically send CAN status messages
	if (now - state->last_can_tx_time >= CAN_TX_PERIOD) {
		can_status_msg_t status = {
			.precharge = (state->mode == LOGIC_MODE_PRECHARGING),
			.contactor = (state->mode == LOGIC_MODE_RUNNING),
			.rc_mode = 0, // TODO
			.throttle_pwm = state->can_current_throttle * 20, // scale 0-1000 to 0-20000us
			.steering_pwm = state->can_current_steering * 20, // scale 0-1000 to 0-20000us
		};
		send_can_status(&status, hcan);
		state->last_can_tx_time = now;
	}
	// REST OF LOGIC
}
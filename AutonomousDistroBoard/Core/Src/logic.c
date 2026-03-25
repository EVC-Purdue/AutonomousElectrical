#include "logic.h"

#include <stdint.h>

#include "ibus.h"
#include "main.h"
#include "can.h"
#include "util.h"


static logic_state_t* g_logic_state_ptr = NULL;


void logic_init(logic_state_t* state) {
	state->mode = LOGIC_MODE_STARTING;
	state->start_time = HAL_GetTick();
	state->estop_triggered_time = option_u32_none(); // not currently triggered
	
	debounce_controller_init(
		&state->estop_debounce,
		false,
		ESTOP_RISING_DEBOUNCE,
		ESTOP_FALLING_DEBOUNCE,
		ESTOP_ACCUMULATING_DEBOUNCE
	);
	debounce_controller_init(
		&state->mode_debounce,
		false,
		MODE_DEBOUNCE_MS,
		MODE_DEBOUNCE_MS,
		MODE_ACCUMULATING_DEBOUNCE_MS
	);

	ibus_init(&state->ibus);
	state->last_can_tx_time = 0;

	state->can_current_throttle_pwm = THROTTLE_PWM_LOW;
	state->can_current_steering_pwm = (STEERING_PWM_LOW + STEERING_PWM_HIGH) / 2;
	state->last_control_timestamp = 0;

	g_logic_state_ptr = state;
}

void logic_handle_control(const can_control_msg_t* cmd) {
	if (g_logic_state_ptr == NULL) {
		return;
	}

	g_logic_state_ptr->can_current_throttle_pwm = can_throttle_to_pwm(cmd);
	g_logic_state_ptr->can_current_steering_pwm = can_steering_to_pwm(cmd);
	g_logic_state_ptr->last_control_timestamp = HAL_GetTick();
}

void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef *sbus_huart,
	CAN_HandleTypeDef *hcan,
	TIM_HandleTypeDef *throttle_htim,
	TIM_HandleTypeDef *steering_htim
) {
	// Process iBUS data
	ibus_process(&state->ibus, sbus_huart);

	uint32_t now = HAL_GetTick();

	// All states: Check for RC connection timeout
	if (!ibus_is_connected(&state->ibus, now, RC_CONNECTION_TIMEOUT)) {
		state->mode = LOGIC_MODE_RC_DISCONNECTED;
	}

	// All states: Check for remote ESTOP trigger
	if (ibus_is_connected(&state->ibus, now, RC_CONNECTION_TIMEOUT)) {
		uint16_t estop_channel_value = state->ibus.channels[IBUS_CHANNEL_ESTOP];
		bool estop_raw_high = estop_channel_value >= ESTOP_PWM_THRESHOLD;
		bool estop_debounced_high = debounce_controller_update(&state->estop_debounce, estop_raw_high, now);
		if (estop_debounced_high) {
			state->estop_triggered_time = option_u32_some(now);
			state->mode = LOGIC_MODE_ESTOPPED;
			// Note: if it was high and remains high, we want to continue to update estop_triggered_time
			// so the ESTOP_TRIGGERED_DELAY doesn't start until the remote estop goes low again
		}
	}

	uint16_t output_throttle_pwm = 0;
	uint16_t output_steering_pwm = 0;
	// TODO: handle for when not in running mode

	// Precharge/contactor state machine logic
	switch (state->mode) {
		case LOGIC_MODE_STARTING: {
			// Precharge and contactor low
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (util_has_elapsed(now, state->start_time, PRECHARGE_START_DELAY)) {
				state->mode = LOGIC_MODE_PRECHARGING;
			}
			break;
		}
		case LOGIC_MODE_PRECHARGING: {
			// Precharge on, contactor off
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			uint32_t total_delay = PRECHARGE_START_DELAY + PRECHARGE_DURATION;
			if (util_has_elapsed(now, state->start_time, total_delay)) {
				state->mode = LOGIC_MODE_CONTACTOR_CLOSING;
			}
			break;
		}
		case LOGIC_MODE_CONTACTOR_CLOSING: {
			// Precharge on, contactor on
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);

			uint32_t total_delay = PRECHARGE_START_DELAY + PRECHARGE_DURATION + CONTACTOR_CLOSED_DELAY;
			if (util_has_elapsed(now, state->start_time, total_delay)) {
				state->mode = LOGIC_MODE_RUNNING;
			}
			break;
		}
		case LOGIC_MODE_RUNNING: { //-------------------------------------------------//
			// Precharge off, contactor on
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);

			// Normal operation!

			// MODE switching logic
			// Note: don't need to check for iBUS connection here because if we lost connection
			// we would have already switched to RC_DISCONNECTED mode
			uint16_t mode_channel_value = state->ibus.channels[IBUS_CHANNEL_MODE];
			bool mode_raw_high = mode_channel_value >= MODE_PWM_THRESHOLD;
			debounce_controller_update(&state->mode_debounce, mode_raw_high, now);

			bool autonomous_mode = debounce_controller_get_state(&state->mode_debounce);

			// In autonomous mode, ignore iBUS throttle and steering and only use CAN commands
			if (autonomous_mode) {
				output_throttle_pwm = state->can_current_throttle_pwm;
				output_steering_pwm = state->can_current_steering_pwm;
			} 
			// In RC mode, ignore CAN commands and only use iBUS throttle and steering
			else {
				uint16_t ibus_throttle_pwm = state->ibus.channels[IBUS_CHANNEL_THROTTLE];
				uint16_t ibus_steering_pwm = state->ibus.channels[IBUS_CHANNEL_STEERING];

				output_throttle_pwm = THROTTLE_PWM_LOW + (ibus_throttle_pwm - THROTTLE_PWM_LOW) / RC_MODE_THROTTLE_DIVISOR;
				output_steering_pwm = ibus_steering_pwm;
			}


			break;
		} //--------------------------------------------------------------------------//
		case LOGIC_MODE_ESTOPPED: {
			// STOP: precharge off, contactor off
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (option_u32_is_some(state->estop_triggered_time)
			    && util_has_elapsed(now, option_u32_unwrap(state->estop_triggered_time), ESTOP_TRIGGERED_DELAY)) {
				// After waiting for ESTOP_TRIGGERED_DELAY, restart precharge sequence
				state->estop_triggered_time = option_u32_none(); // reset estop triggered time
				state->mode = LOGIC_MODE_STARTING;
				state->start_time = now;
			}
			break;
		}
		case LOGIC_MODE_RC_DISCONNECTED: {
			// STOP: precharge off, contactor off
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (ibus_is_connected(&state->ibus, now, RC_CONNECTION_TIMEOUT)) {
				// If we receive a valid iBUS frame, consider the RC connection to be restored and restart precharge sequence
				state->mode = LOGIC_MODE_STARTING;
				state->start_time = now;
			}
			break;
		}
	}

	// Clamp the output PWM values to their valid ranges just in case
	output_throttle_pwm = clamp_u16(output_throttle_pwm, THROTTLE_PWM_LOW, THROTTLE_PWM_HIGH);
	output_steering_pwm = clamp_u16(output_steering_pwm, STEERING_PWM_LOW, STEERING_PWM_HIGH);

	// Output the PWM values
	__HAL_TIM_SET_COMPARE(throttle_htim, TIM_CHANNEL_1, output_throttle_pwm);
	__HAL_TIM_SET_COMPARE(steering_htim, TIM_CHANNEL_1, output_steering_pwm);

	// Periodically send CAN status messages
	if (util_has_elapsed(now, state->last_can_tx_time, CAN_TX_PERIOD)) {
		can_status_msg_t status = {
			.mode = (uint8_t)state->mode,
			.rc_mode = debounce_controller_get_state(&state->mode_debounce), // if mode_debounce is low, we are in RC mode
			.throttle_pwm = output_throttle_pwm,
			.steering_pwm = output_steering_pwm
		};
		send_can_status(&status, hcan);
		state->last_can_tx_time = now;
	}
}
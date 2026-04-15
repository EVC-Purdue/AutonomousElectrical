#include "logic.h"

#include <stdint.h>

#include "ibus.h"
#include "main.h"
#include "can.h"
#include "util.h"


static logic_state_t* g_logic_state_ptr = NULL;


void logic_init(logic_state_t* state) {
	state->mode = LOGIC_MODE_STARTING;
	state->mode_start_time = HAL_GetTick();
	
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

	state->output_throttle_pwm = THROTTLE_PWM_LOW;
	state->output_steering_pwm = (STEERING_PWM_LOW + STEERING_PWM_HIGH) / 2;
	state->throttle_enabled = true; // HAL init sets up the PWM

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

void logic_switch_mode(logic_state_t* state, logic_mode_t new_mode) {
	if (state == NULL) {
		return;
	}

	state->mode = new_mode;
	state->mode_start_time = HAL_GetTick();
}

void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef* sbus_huart,
	CAN_HandleTypeDef* hcan,
	TIM_HandleTypeDef* throttle_htim,
	TIM_HandleTypeDef* steering_htim
) {
	// Process iBUS data
	ibus_process(&state->ibus, sbus_huart);

	uint32_t now = HAL_GetTick();

	// All states: Check for RC connection timeout
	if (!ibus_is_connected(&state->ibus, now, RC_CONNECTION_TIMEOUT)) {
		logic_switch_mode(state, LOGIC_MODE_RC_DISCONNECTED);
	} else {
		// All states: update RC debounce controllers
		uint16_t estop_channel_value = state->ibus.channels[IBUS_CHANNEL_ESTOP];
		bool estop_raw_high = estop_channel_value >= ESTOP_PWM_THRESHOLD;
		debounce_controller_update(&state->estop_debounce, estop_raw_high, now);

		uint16_t mode_channel_value = state->ibus.channels[IBUS_CHANNEL_MODE];
		bool mode_raw_high = mode_channel_value >= MODE_PWM_THRESHOLD;
		debounce_controller_update(&state->mode_debounce, mode_raw_high, now);

		// All states: check remote estop
		bool estop_debounced_high = debounce_controller_get_state(&state->estop_debounce);
		if (estop_debounced_high) {
			logic_switch_mode(state, LOGIC_MODE_ESTOPPED);
			// Note: if it was high and remains high, we want to continue to force
		}
	}

	// Precharge/contactor state machine logic
	switch (state->mode) {
		case LOGIC_MODE_STARTING: {
			// Precharge and contactor low
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (util_has_elapsed(now, state->mode_start_time, PRECHARGE_START_DELAY)) {
				logic_switch_mode(state, LOGIC_MODE_PRECHARGING);
			}
			break;
		}
		case LOGIC_MODE_PRECHARGING: {
			// Precharge on, contactor off
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (util_has_elapsed(now, state->mode_start_time, PRECHARGE_DURATION)) {
				logic_switch_mode(state, LOGIC_MODE_CONTACTOR_CLOSING);
			}
			break;
		}
		case LOGIC_MODE_CONTACTOR_CLOSING: {
			// Precharge on, contactor on
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);

			if (util_has_elapsed(now, state->mode_start_time, CONTACTOR_CLOSED_DELAY)) {
				logic_switch_mode(state, LOGIC_MODE_RUNNING);
			}
			break;
		}
		case LOGIC_MODE_RUNNING: { //-------------------------------------------------//
			// Precharge off, contactor on
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);

			// Normal operation!

			bool autonomous_mode = debounce_controller_get_state(&state->mode_debounce);

			// In autonomous mode, ignore iBUS throttle and steering and only use CAN commands
			if (autonomous_mode) {
				if (!util_has_elapsed(now, state->last_control_timestamp, CAN_RX_TIMEOUT)) {
					// If we have received a control message recently, consider the CAN connection to be
					// healthy and use CAN commands for throttle and steering
					state->output_throttle_pwm = state->can_current_throttle_pwm;
					state->output_steering_pwm = state->can_current_steering_pwm;
				} else {
					// If we have not received a control message recently, consider the CAN connection to
					// be lost and switch to CAN_DISCONNECTED mode
					logic_switch_mode(state, LOGIC_MODE_CAN_DISCONNECTED);
					state->output_throttle_pwm = THROTTLE_PWM_LOW;
					state->output_steering_pwm = STEERING_PWM_CENTER;
				}
			} 
			// In RC mode, ignore CAN commands and only use iBUS throttle and steering
			else {
				uint16_t ibus_throttle_pwm = state->ibus.channels[IBUS_CHANNEL_THROTTLE];
				uint16_t ibus_steering_pwm = state->ibus.channels[IBUS_CHANNEL_STEERING];

				state->output_throttle_pwm = map_u16(ibus_throttle_pwm, 1500, 2000, THROTTLE_PWM_LOW, THROTTLE_PWM_HIGH);
				state->output_steering_pwm = ibus_steering_pwm;
			}


			break;
		} //--------------------------------------------------------------------------//
		case LOGIC_MODE_ESTOPPED: {
			// STOP: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_pwm = THROTTLE_PWM_LOW;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			bool estop_debounced_high = debounce_controller_get_state(&state->estop_debounce);
			if (!estop_debounced_high) {
				logic_switch_mode(state, LOGIC_MODE_RECOVERING);
			}
			break;
		}
		case LOGIC_MODE_RC_DISCONNECTED: {
			// STOP: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_pwm = THROTTLE_PWM_LOW;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			if (ibus_is_connected(&state->ibus, now, RC_CONNECTION_TIMEOUT)) {
				logic_switch_mode(state, LOGIC_MODE_RECOVERING);
			}
			break;
		}
		case LOGIC_MODE_CAN_DISCONNECTED: {
			// CAN disconnect in autonomous mode = stop the kart
			// STOP: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_pwm = THROTTLE_PWM_LOW;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			uint16_t mode_channel_value = state->ibus.channels[IBUS_CHANNEL_MODE];
			bool mode_raw_high = mode_channel_value >= MODE_PWM_THRESHOLD;
			debounce_controller_update(&state->mode_debounce, mode_raw_high, now);

			bool autonomous_mode = debounce_controller_get_state(&state->mode_debounce);
			bool can_connection_ok = !util_has_elapsed(now, state->last_control_timestamp, CAN_RX_TIMEOUT);
			bool ibus_connection_ok = ibus_is_connected(&state->ibus, now, RC_CONNECTION_TIMEOUT);

			if ((autonomous_mode && can_connection_ok)
				|| (!autonomous_mode && ibus_connection_ok)) {
				logic_switch_mode(state, LOGIC_MODE_RECOVERING);
			}
			break;
		}
		case LOGIC_MODE_RECOVERING: {
			// Still stopped: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_pwm = THROTTLE_PWM_LOW;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			// Note: when, for example, we have RC_DISCONNECTED error it will keep
			// calling logic_switch_mode with RC_DISCONNECTED every loop, which will reset
			// the mode_start_time and correctly prevent us from ever leaving the RECOVERING mode
			if (util_has_elapsed(now, state->mode_start_time, RECOVERING_DELAY)) {
				logic_switch_mode(state, LOGIC_MODE_STARTING);
			}

			break;
		}
	}

	// Clamp the output PWM values to their valid ranges just in case
	state->output_throttle_pwm = clamp_u16(state->output_throttle_pwm, THROTTLE_PWM_LOW, THROTTLE_PWM_HIGH);
	state->output_steering_pwm = clamp_u16(state->output_steering_pwm, STEERING_PWM_LOW, STEERING_PWM_HIGH);

	__HAL_TIM_SET_COMPARE(throttle_htim, TIM_CHANNEL_1, state->output_throttle_pwm);
	__HAL_TIM_SET_COMPARE(steering_htim, TIM_CHANNEL_1, state->output_steering_pwm);

	// Periodically send CAN status messages
	if (util_has_elapsed(now, state->last_can_tx_time, CAN_TX_PERIOD)) {
		can_status_msg_t status = {
			.mode = (uint8_t)state->mode,
			.rc_mode = debounce_controller_get_state(&state->mode_debounce), // if mode_debounce is low, we are in RC mode
			.throttle_pwm = state->output_throttle_pwm,
			.steering_pwm = state->output_steering_pwm
		};
		send_can_status(&status, hcan);
		state->last_can_tx_time = now;
	}

	// Blink LED
	uint16_t led_period = 0;
	switch (state->mode) {
		case LOGIC_MODE_STARTING:          led_period = LED_STARTING_PERIOD;          break;
		case LOGIC_MODE_PRECHARGING:       led_period = LED_PRECHARGING_PERIOD;       break;
		case LOGIC_MODE_CONTACTOR_CLOSING: led_period = LED_CONTACTOR_CLOSING_PERIOD; break;
		case LOGIC_MODE_RUNNING: {
			bool autonomous_mode = debounce_controller_get_state(&state->mode_debounce);
			if (autonomous_mode) {
				led_period = LED_RUNNING_AUTONOMOUS_PERIOD;
			} else {
				led_period = LED_RUNNING_RC_PERIOD;
			}
			break;
		}
		case LOGIC_MODE_ESTOPPED:	       led_period = LED_ESTOPPED_PERIOD;          break;
		case LOGIC_MODE_RC_DISCONNECTED:   led_period = LED_RC_DISCONNECTED_PERIOD;   break;
		case LOGIC_MODE_CAN_DISCONNECTED:  led_period = LED_CAN_DISCONNECTED_PERIOD;  break;
		case LOGIC_MODE_RECOVERING:        led_period = LED_RECOVERING_PERIOD;        break;
	}
	if (led_period == 0) {
		// Solid on
		HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
	} else {
		if (util_has_elapsed(now, state->led_blink_timestamp, led_period)) {
			HAL_GPIO_TogglePin(LED_OUT_GPIO_Port, LED_OUT_Pin);
			state->led_blink_timestamp = now;
		}
	}
}
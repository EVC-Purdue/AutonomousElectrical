#include "logic.h"

#include <stdint.h>

#include "ibus.h"
#include "main.h"
#include "can.h"
#include "util.h"


static logic_state_t* g_logic_state_ptr = NULL;


void logic_init(logic_state_t* state) {
	state->mode = LOGIC_MODE_STARTING;
	state->last_mode_set_time = NOW();
	
	debounce_controller_init(
		&state->estop_debounce,
		SW_ESTOP_STATE_LOW,
		SW_ESTOP_DEBOUNCE,
		SW_ESTOP_ACCUMULATING_DEBOUNCE
	);
	debounce_controller_init(
		&state->mode_debounce,
		LOGIC_RUNNING_RC,
		SW_MODE_DEBOUNCE_MS,
		SW_MODE_ACCUMULATING_DEBOUNCE_MS
	);

	ibus_init(&state->ibus);
	state->last_can_status_tx_time = 0;

	state->can_current_throttle_erpm = 0;
	state->can_current_steering_pwm = STEERING_PWM_CENTER;
	state->last_control_timestamp = 0;

	state->heartbeat_counter = 0;
	state->last_heartbeat_timestamp = 0;

	state->vesc_current_erpm = 0;
	state->vesc_last_status_timestamp = 0;

	state->output_throttle_erpm = 0;
	state->output_throttle_pwm = THROTTLE_PWM_LOW;
	state->output_steering_pwm = STEERING_PWM_CENTER;
	state->last_can_vesc_set_rpm_tx_time = 0;

	state->t0 = NOW();
	state->t1 = NOW();

	g_logic_state_ptr = state;
}

uint16_t logic_erpm_to_pwm(int32_t erpm) {
	// Full range of value ERPM values is REVERSE_ERPM_MIN to VESC_ERPM_MAX, that is mapped that to the full range of PWM values (1000-2000)
	int32_t throttle_i32 = map_i32(erpm, REVERSE_ERPM_MIN, VESC_ERPM_MAX, THROTTLE_PWM_LOW, THROTTLE_PWM_HIGH);
	// Safe to directly cast to uint16_t as the map() clamps and the output range is within uint16_t limits
	return (uint16_t)throttle_i32;
}

void logic_handle_control(const can_control_msg_t* cmd) {
	if (g_logic_state_ptr == NULL) {
		return;
	}

	g_logic_state_ptr->can_current_throttle_erpm = cmd->throttle_erpm;
	g_logic_state_ptr->can_current_steering_pwm = map_u16(cmd->steering, CAN_STEERING_MIN, CAN_STEERING_MAX, STEERING_PWM_LOW, STEERING_PWM_HIGH);
	g_logic_state_ptr->last_control_timestamp = NOW();
}

void logic_handle_heartbeat(const can_heartbeat_msg_t* heartbeat) {
	if (g_logic_state_ptr == NULL) {
		return;
	}

	g_logic_state_ptr->heartbeat_counter = heartbeat->counter;
	g_logic_state_ptr->last_heartbeat_timestamp = NOW();
}

void logic_handle_vesc_status_1(const can_vesc_status_1_msg_t* vesc_status_1) {
	if (g_logic_state_ptr == NULL) {
		return;
	}

	g_logic_state_ptr->vesc_current_erpm = vesc_status_1->erpm;
	g_logic_state_ptr->vesc_last_status_timestamp = NOW();
}

void logic_switch_mode(logic_state_t* state, logic_mode_t new_mode, uint32_t now) {
	if (state == NULL) {
		return;
	}

	state->mode = new_mode;
	state->last_mode_set_time = now;
}

logic_running_submode_t pwm_value_to_running_submode(uint16_t mode_pwm_value) {
	if (mode_pwm_value >= SW_MODE_IDLE_PWM_VALUE - SW_MODE_PWM_TOLERANCE && mode_pwm_value <= SW_MODE_IDLE_PWM_VALUE + SW_MODE_PWM_TOLERANCE) {
		return LOGIC_RUNNING_IDLE;
	} else if (mode_pwm_value >= SW_MODE_AUTONOMOUS_PWM_VALUE - SW_MODE_PWM_TOLERANCE && mode_pwm_value <= SW_MODE_AUTONOMOUS_PWM_VALUE + SW_MODE_PWM_TOLERANCE) {
		return LOGIC_RUNNING_AUTONOMOUS;
	} else if (mode_pwm_value >= SW_MODE_RC_PWM_VALUE - SW_MODE_PWM_TOLERANCE && mode_pwm_value <= SW_MODE_RC_PWM_VALUE + SW_MODE_PWM_TOLERANCE) {
		return LOGIC_RUNNING_RC;
	} else {
		// If the value doesn't match any known submode, default to IDLE (safety choice, slowly stop)
		return LOGIC_RUNNING_IDLE;
	}
}

void logic_clear_can_control(logic_state_t* state) {
	// Important because otherwise when we go into software mode we may begin using the stale CAN values + an up to date heartbeat value

	if (state == NULL) {
		return;
	}

	uint32_t primask = __get_PRIMASK();
 	__disable_irq();
	state->can_current_throttle_erpm = 0;
	state->can_current_steering_pwm = STEERING_PWM_CENTER;
	state->last_control_timestamp = 0;
	if (primask == 0U) {
		__enable_irq();
 	}
}

void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef* sbus_huart,
	CAN_HandleTypeDef* hcan,
	TIM_HandleTypeDef* throttle_htim,
	TIM_HandleTypeDef* steering_htim
) {
	logic_mode_t prev_mode = state->mode;

	// Process iBUS data
	ibus_process(&state->ibus, sbus_huart);

	state->t0 = state->t1;
	state->t1 = NOW();

	// All states: Check for RC connection timeout
	if (!ibus_is_connected(&state->ibus, NOW(), RC_CONNECTION_TIMEOUT)) {
		logic_switch_mode(state, LOGIC_MODE_RC_DISCONNECTED, NOW());
	} else {
		// All states: update RC debounce controllers
		uint16_t estop_channel_value = state->ibus.channels[IBUS_CHANNEL_ESTOP];
		bool is_estop_high = estop_channel_value >= SW_ESTOP_PWM_THRESHOLD;
		debounce_state_t estop_raw = is_estop_high ? SW_ESTOP_STATE_HIGH : SW_ESTOP_STATE_LOW;
		debounce_controller_update(&state->estop_debounce, estop_raw, NOW());

		uint16_t mode_channel_value = state->ibus.channels[IBUS_CHANNEL_MODE];
		debounce_state_t mode_raw = pwm_value_to_running_submode(mode_channel_value);
		debounce_controller_update(&state->mode_debounce, mode_raw, NOW());

		// All states: check remote estop
		debounce_state_t estop_debounced = debounce_controller_get_state(&state->estop_debounce);
		if (estop_debounced != SW_ESTOP_STATE_LOW) {
			logic_switch_mode(state, LOGIC_MODE_ESTOPPED, NOW());
			// Note: if it was high and remains high, we want to continue to force
		}
	}

	// Precharge/contactor state machine logic
	switch (state->mode) {
		case LOGIC_MODE_STARTING: {
			// Precharge and contactor low
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (util_has_elapsed(NOW(), state->last_mode_set_time, PRECHARGE_START_DELAY)) {
				logic_switch_mode(state, LOGIC_MODE_PRECHARGING, NOW());
			}
			break;
		}
		case LOGIC_MODE_PRECHARGING: {
			// Precharge on, contactor off
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);

			if (util_has_elapsed(NOW(), state->last_mode_set_time, PRECHARGE_DURATION)) {
				logic_switch_mode(state, LOGIC_MODE_CONTACTOR_CLOSING, NOW());
			}
			break;
		}
		case LOGIC_MODE_CONTACTOR_CLOSING: {
			// Precharge on, contactor on
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);

			if (util_has_elapsed(NOW(), state->last_mode_set_time, CONTACTOR_CLOSED_DELAY)) {
				logic_switch_mode(state, LOGIC_MODE_RUNNING, NOW());
			}
			break;
		}
		case LOGIC_MODE_RUNNING: { //-------------------------------------------------//
			// Precharge off, contactor on
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_SET);

			logic_running_submode_t running_submode = (logic_running_submode_t)debounce_controller_get_state(&state->mode_debounce);
			switch (running_submode) {
				case LOGIC_RUNNING_RC: {
					// RC mode: use iBUS inputs for throttle and steering
					// Don't need to check RC connection here because if it was lost we would
					// have already switched to RC_DISCONNECTED mode in the earlier check
					uint16_t ibus_throttle_pwm = state->ibus.channels[IBUS_CHANNEL_THROTTLE];
					uint16_t ibus_steering_pwm = state->ibus.channels[IBUS_CHANNEL_STEERING];

					bool throttle_stick_idle = abs_i32((int32_t)ibus_throttle_pwm - THROTTLE_STICK_IDLE) <= THROTTLE_STICK_DEADBAND;
					if (throttle_stick_idle) {
						state->output_throttle_erpm = 0;
					} else if (ibus_throttle_pwm > THROTTLE_STICK_IDLE) {
						// Forward throttle
						state->output_throttle_erpm = map_i32((int32_t)ibus_throttle_pwm, THROTTLE_STICK_IDLE, THROTTLE_STICK_MAX, 0, RC_ERPM_MAX);
					} else {
						// Reverse throttle
						state->output_throttle_erpm = map_i32((int32_t)ibus_throttle_pwm, THROTTLE_STICK_IDLE, THROTTLE_STICK_MIN, 0, REVERSE_ERPM_MIN);
					}

					state->output_steering_pwm = ibus_steering_pwm; // iBUS steering is already in the form of a PWM value, just pass it through directly
					
					logic_clear_can_control(state); 

					break;
				}
				case LOGIC_RUNNING_AUTONOMOUS: {
					// Autonomous mode: use CAN throttle and steering
					if (!util_has_elapsed(NOW(), state->last_heartbeat_timestamp, CAN_RX_HB_TIMEOUT)) {
						// If we have received a heartbeat message recently, consider the CAN connection to be
						// healthy and use CAN commands for throttle and steering
						state->output_throttle_erpm = state->can_current_throttle_erpm;
						state->output_steering_pwm = state->can_current_steering_pwm;
					} else {
						// If we have not received a heartbeat message recently, consider the CAN connection to
						// be lost and switch to CAN_DISCONNECTED mode
						logic_switch_mode(state, LOGIC_MODE_CAN_DISCONNECTED, NOW());
						state->output_throttle_erpm = 0;
						state->output_steering_pwm = STEERING_PWM_CENTER;
					}
					break;
				}
				case LOGIC_RUNNING_IDLE: {
					// Idle mode: ignore throttle and steering inputs
					// Rate-limited resetting of throttle and steering (throttle to 0, steering to center)
					float dt = (state->t1 - state->t0) / 1000.0f;

					// Decrease ERPM at a fixed deceleration rate but not below 0
					int32_t throttle_erpm_change = (int32_t)(IDLE_ERPM_DECEL * dt);
					state->output_throttle_erpm = (int32_t)max_i32(state->output_throttle_erpm - throttle_erpm_change, 0);

					// Steer towards center at a fixed rate
					float steering_pwm_change = IDLE_STEERING_PWM_VEL * dt;
					if (state->output_steering_pwm < STEERING_PWM_CENTER) {
						state->output_steering_pwm = (uint16_t)min_i32((int32_t)state->output_steering_pwm + (int32_t)steering_pwm_change, STEERING_PWM_CENTER);
					} else if (state->output_steering_pwm > STEERING_PWM_CENTER) {
						state->output_steering_pwm = (uint16_t)max_i32((int32_t)state->output_steering_pwm - (int32_t)steering_pwm_change, STEERING_PWM_CENTER);
					}

					logic_clear_can_control(state);

					break;
				}
			}
			break;
		} //--------------------------------------------------------------------------//
		case LOGIC_MODE_ESTOPPED: {
			// STOP: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_erpm = 0;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			debounce_state_t estop_debounced = debounce_controller_get_state(&state->estop_debounce);
			if (estop_debounced == SW_ESTOP_STATE_LOW) {
				logic_switch_mode(state, LOGIC_MODE_RECOVERING, NOW());
			}
			break;
		}
		case LOGIC_MODE_RC_DISCONNECTED: {
			// STOP: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_erpm = 0;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			if (ibus_is_connected(&state->ibus, NOW(), RC_CONNECTION_TIMEOUT)) {
				logic_switch_mode(state, LOGIC_MODE_RECOVERING, NOW());
			}
			break;
		}
		case LOGIC_MODE_CAN_DISCONNECTED: {
			// CAN disconnect in autonomous mode = stop the kart
			// STOP: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_erpm = 0;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			debounce_state_t mode_debounced = (logic_running_submode_t)debounce_controller_get_state(&state->mode_debounce);
			bool autonomous_mode = mode_debounced == LOGIC_RUNNING_AUTONOMOUS;
			bool can_connection_ok = !util_has_elapsed(NOW(), state->last_heartbeat_timestamp, CAN_RX_HB_TIMEOUT);
			bool ibus_connection_ok = ibus_is_connected(&state->ibus, NOW(), RC_CONNECTION_TIMEOUT);

			if ((autonomous_mode && can_connection_ok)
				|| (!autonomous_mode && ibus_connection_ok)) {
				logic_switch_mode(state, LOGIC_MODE_RECOVERING, NOW());
			}
			break;
		}
		case LOGIC_MODE_RECOVERING: {
			// Still stopped: precharge off, contactor off, throttle low, steering straight
			HAL_GPIO_WritePin(PRECHARGE_EN_GPIO_Port, PRECHARGE_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MAIN_COIL_EN_GPIO_Port, MAIN_COIL_EN_Pin, GPIO_PIN_RESET);
			state->output_throttle_erpm = 0;
			state->output_steering_pwm = STEERING_PWM_CENTER;

			if (util_has_elapsed(NOW(), state->last_mode_set_time, RECOVERING_DELAY)) {
				logic_switch_mode(state, LOGIC_MODE_STARTING, NOW());
			}

			break;
		}
	}

	// Clamp the output values to their valid ranges just in case
	logic_running_submode_t mode_debounced = debounce_controller_get_state(&state->mode_debounce);
	bool non_rc_mode = mode_debounced != LOGIC_RUNNING_RC;
	int32_t erpm_max = non_rc_mode ? AUTONOMOUS_ERPM_MAX : RC_ERPM_MAX;
	int32_t erpm_min = non_rc_mode ? 0 : REVERSE_ERPM_MIN; // No reverse allowed in non-RC modes
	state->output_throttle_erpm = clamp_i32(state->output_throttle_erpm, erpm_min, erpm_max);
	state->output_steering_pwm = clamp_u16(state->output_steering_pwm, STEERING_PWM_LOW, STEERING_PWM_HIGH);

	// Set the throttle PWM as a function of the output ERPM
	state->output_throttle_pwm = logic_erpm_to_pwm(state->output_throttle_erpm);

	// Periodically send CAN throttle commands to the VESC
	if (
		util_has_elapsed(NOW(), state->last_can_vesc_set_rpm_tx_time, CAN_VESC_SET_RPM_TX_PERIOD) &&
		HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0
	) {
		can_vesc_set_rpm_msg_t set_rpm_msg = {
			.erpm = state->output_throttle_erpm
		};
		send_can_vesc_set_rpm(&set_rpm_msg, hcan);
		state->last_can_vesc_set_rpm_tx_time = NOW();
	}

	// Steering deadband
	bool within_deadband = abs_i32((int32_t)state->output_steering_pwm - STEERING_PWM_CENTER) <= STEERING_PWM_DEADBAND;
	if (within_deadband) {
		state->output_steering_pwm = STEERING_PWM_CENTER;
	}

	// Set the PWM outputs
	__HAL_TIM_SET_COMPARE(throttle_htim, TIM_CHANNEL_1, state->output_throttle_pwm);
	__HAL_TIM_SET_COMPARE(steering_htim, TIM_CHANNEL_1, state->output_steering_pwm);

	// Periodically send CAN status messages or if state changed
	bool state_changed = prev_mode != state->mode;
	bool status_msg_needed = state_changed || util_has_elapsed(NOW(), state->last_can_status_tx_time, CAN_STATUS_TX_PERIOD);
	if (status_msg_needed && HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
		can_status_msg_t status = {
			.logic_state = (uint8_t)state->mode,
			.running_mode = (uint8_t)debounce_controller_get_state(&state->mode_debounce),
			.throttle_pwm = state->output_throttle_pwm,
			.steering_pwm = state->output_steering_pwm
		};
		send_can_status(&status, hcan);
		state->last_can_status_tx_time = NOW();
	}

	// Blink LED
	uint16_t led_period = 0;
	switch (state->mode) {
		case LOGIC_MODE_STARTING:          led_period = LED_STARTING_PERIOD;          break;
		case LOGIC_MODE_PRECHARGING:       led_period = LED_PRECHARGING_PERIOD;       break;
		case LOGIC_MODE_CONTACTOR_CLOSING: led_period = LED_CONTACTOR_CLOSING_PERIOD; break;
		case LOGIC_MODE_RUNNING: {
			logic_running_submode_t running_submode = (logic_running_submode_t)debounce_controller_get_state(&state->mode_debounce);
			switch (running_submode) {
				case LOGIC_RUNNING_RC:         led_period = LED_RUNNING_RC_PERIOD;         break;
				case LOGIC_RUNNING_AUTONOMOUS: led_period = LED_RUNNING_AUTONOMOUS_PERIOD; break;
				case LOGIC_RUNNING_IDLE:       led_period = LED_RUNNING_IDLE_PERIOD;       break;
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
		bool led_on = (NOW() / led_period) % 2 == 0;
		HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	// Set CAN err if there is one
	uint32_t can_err = HAL_CAN_GetError(hcan);
	if (can_err != 0) {
		state->can_err = can_err;
	}
}
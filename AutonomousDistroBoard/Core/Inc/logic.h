#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>
#include <assert.h>

#include "stm32f4xx_hal.h"

#include "ibus.h"
#include "can.h"
#include "util.h"


#define PRECHARGE_START_DELAY  (200) // ms, wait from boot before starting precharge
#define PRECHARGE_DURATION     (5000) // ms, how long to run precharge before closing contactor
#define CONTACTOR_CLOSED_DELAY (100) // ms, how long to wait after contactor is requested to be closed before considering it fully closed
#define RECOVERING_DELAY       (5000) // ms, how long to wait after transistioning after a fault (E-STOP, RC disconnect, CAN disconnect) before allowing to transition back to STARTING mode/precharge sequence

#define IBUS_CHANNEL_THROTTLE (1) // 1500 = full stop, 2000 = full throttle forward
#define IBUS_CHANNEL_STEERING (3) // 1000 = full left, 1500 = center, 2000 = full right
#define IBUS_CHANNEL_MODE     (4) // ~1000 = RC mode, ~1500 = autonomous mode, ~2000 = IDLE mode
#define IBUS_CHANNEL_ESTOP    (5) // ~1000 = not pressed, ~2000 = estop

#define THROTTLE_STICK_IDLE (1500) // throttle stick resting value
#define THROTTLE_STICK_MAX  (2000) // maximum throttle stick value

#define SW_ESTOP_PWM_THRESHOLD         (1500) // if the ESTOP channel goes above this value, consider the remote estop to be triggered
#define SW_ESTOP_DEBOUNCE              (200)  // ms, require the ESTOP channel to be above the threshold for at least this long before considering the remote estop to be triggered
#define SW_ESTOP_ACCUMULATING_DEBOUNCE (30)   // ms, when the rising ESTOP is debouncing/accumulating, require the ESTOP channel to be below the threshold for at least this long before resetting the debounce timer
#define SW_ESTOP_STATE_LOW              (0)   // the value ESTOP low correlates to in relation to the debounce controller
#define SW_ESTOP_STATE_HIGH             (1)   // the value ESTOP high correlates to in relation to the debounce controller

#define SW_MODE_RC_PWM_VALUE             (1000) // the value to map the MODE channel to when in RC mode
#define SW_MODE_AUTONOMOUS_PWM_VALUE     (1500) // the value to map the MODE channel to when in autonomous mode
#define SW_MODE_IDLE_PWM_VALUE           (2000) // the value to map the MODE channel to when in IDLE mode
#define SW_MODE_PWM_TOLERANCE            (100)  // the range above and below the target MODE channel values to still consider it a valid reading for that mode
#define SW_MODE_DEBOUNCE_MS              (200)  // ms, require the MODE channel to be consistently in a valid range for at least this long before switching modes
#define SW_MODE_ACCUMULATING_DEBOUNCE_MS (30)   // ms, when the MODE channel is debouncing/accumulating, require it to be out of the valid range for the target mode for at least this long before resetting the debounce timer

#define CAN_STATUS_TX_PERIOD       (10) // ms
#define CAN_VESC_SET_RPM_TX_PERIOD (3) // ms
#define CAN_RX_HB_TIMEOUT          (500) // ms, for the heartbeat message

#define RC_CONNECTION_TIMEOUT (100) // ms, if we did not get a valid iBUS frame within this time, consider the RC connection to be lost

#define VESC_ERPM_MAX       (14218)          // Maximum ERPM set in VESC tool (12 meters/second)
#define AUTONOMOUS_ERPM_MAX (VESC_ERPM_MAX) // Maximum ERPM allowed in software/autonomous mode. Software allowed to command full range.
#define RC_ERPM_MAX         (7000)          // Maximum ERPM allowed in RC mode (~6 meters/second)

#define IDLE_ERPM_DECEL (4000)       // ERPM/s, max deceleration (rate of change of ERPM) when in IDLE
#define IDLE_STEERING_PWM_VEL (1000) // PWM/s, max rate of change of steering when in IDLE
static_assert(IDLE_ERPM_DECEL >= 1000, "dt resolution is 1ms so (int32_t)(dt * IDLE_ERPM_DECEL) needs to be at least 1 to ensure that we can actually decrease the ERPM");
static_assert(IDLE_STEERING_PWM_VEL >= 1000, "dt resolution is 1ms so (int32_t)(dt * IDLE_STEERING_PWM_VEL) needs to be at least 1 to ensure that we can actually change the steering PWM");

// Really these are half periods b/c it is the rate at which the LED toggles
#define LED_STARTING_PERIOD           (100)  // ms
#define LED_PRECHARGING_PERIOD        (400)  // ms
#define LED_CONTACTOR_CLOSING_PERIOD  (50)   // ms
#define LED_RUNNING_RC_PERIOD         (1000) // ms
#define LED_RUNNING_AUTONOMOUS_PERIOD (250)  // ms
#define LED_RUNNING_IDLE_PERIOD       (125)  // ms
#define LED_ESTOPPED_PERIOD           (0)    // solid on
#define LED_RC_DISCONNECTED_PERIOD    (0)    // solid on
#define LED_CAN_DISCONNECTED_PERIOD   (0)    // solid on
#define LED_RECOVERING_PERIOD         (2000) // ms

#define THROTTLE_PWM_LOW  (1000)
#define THROTTLE_PWM_HIGH (2000)
#define STEERING_PWM_LOW  (1000)
#define STEERING_PWM_HIGH (2000)
#define STEERING_PWM_CENTER ((STEERING_PWM_LOW + STEERING_PWM_HIGH) / 2)

#define THROTTLE_PWM_GPIO_Port (GPIOB)
#define THROTTLE_PWM_Pin (GPIO_PIN_9)
#define THROTTLE_PWM_AF (GPIO_AF2_TIM4)

#define STEERING_PWM_GPIO_Port (GPIOE)
#define STEERING_PWM_Pin (GPIO_PIN_9)
#define STEERING_PWM_AF (GPIO_AF3_TIM11)

typedef enum {
	LOGIC_MODE_STARTING = 0,
	LOGIC_MODE_PRECHARGING,
	LOGIC_MODE_CONTACTOR_CLOSING,
	LOGIC_MODE_RUNNING,
	LOGIC_MODE_ESTOPPED,
	LOGIC_MODE_RC_DISCONNECTED,
	LOGIC_MODE_CAN_DISCONNECTED,
	LOGIC_MODE_RECOVERING,
} logic_mode_t;

typedef enum {
	LOGIC_RUNNING_RC         = 0,
	LOGIC_RUNNING_AUTONOMOUS = 1,
	LOGIC_RUNNING_IDLE       = 2,
} logic_running_submode_t;


typedef struct {
	logic_mode_t mode; // Use logic_switch_mode() for normal mode transitions so last_mode_set_time is updated; direct assignment is only for initialization/internal setup that also handles last_mode_set_time appropriately
	uint32_t last_mode_set_time; // HAL_GetTick() timestamp of when we last set the current mode (set not switched)

	ibus_t ibus;
	uint32_t last_can_status_tx_time; // time of the last sent CAN status message (to Jetson/Pi)
	
	debounce_controller_t estop_debounce; // debounce controller for the remote estop channel
	debounce_controller_t mode_debounce; // debounce controller for the remote mode channel

	volatile uint16_t can_current_throttle_erpm; // 0-MAX_ERPM, updated by CAN RX callback
	volatile uint16_t can_current_steering_pwm; // 1000-2000, updated by CAN RX callback
	volatile uint32_t last_control_timestamp; // HAL_GetTick() timestamp of last received control message

	volatile uint8_t heartbeat_counter; // updated by CAN RX callback when we receive a heartbeat message from E_Comms
	volatile uint32_t last_heartbeat_timestamp; // HAL_GetTick() timestamp of last received heartbeat message from E_Comms

	volatile int32_t vesc_current_erpm; // updated by CAN RX callback when we receive a VESC status 1 message
	volatile uint32_t vesc_last_status_timestamp; // HAL_GetTick() timestamp of last received VESC status 1 message

	int32_t output_throttle_erpm; // the ERPM value sent to the motor controller in the current/last iteration
	uint16_t output_throttle_pwm; // 1000-2000, the PWM value sent to the motor controller in the current/last iteration. Always set as a function of output_throttle_erpm.
	uint16_t output_steering_pwm; // 1000-2000, PWM value sent to the steering servo in the current/last iteration
	uint32_t last_can_vesc_set_rpm_tx_time; // time of the last sent CAN set (E)RPM message (to VESC)

	uint32_t last_loop_time; // HAL_GetTick() timestamp of the start of the last logic_run() iteration, used for calculation of delta time in IDLE deceleration and steering rate limiting
	uint32_t now; // HAL_GetTick() timestamp of the current logic_run() iteration, used for all timeouts and timing-related logic in logic_run() and to set last_loop_time
} logic_state_t;

void logic_init(logic_state_t* state);

void logic_switch_mode(logic_state_t* state, logic_mode_t new_mode, uint32_t now);

// Convert the 3-position MODE switch PWM value to the corresponding logic_running_submode_t value
// Returns IDLE if the value doesn't match any range
logic_running_submode_t pwm_value_to_running_submode(uint16_t mode_pwm_value);

// Clear the CAN control values in the logic state (set throttle ERPM to 0, steering PWM to center, and reset the control timestamp)
void clear_can_control(logic_state_t* state);

// Called AFAP (as fast as possible) in the main loop
// It is also the only thing called in the main loop
void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef* sbus_huart,
	CAN_HandleTypeDef* hcan,
	TIM_HandleTypeDef* throttle_htim,
	TIM_HandleTypeDef* steering_htim
);

// Called from CAN RX callback when a control message is received
// Uses the global static pointer to the logic state, since the CAN callback
// doesn't have a way to pass user data
void logic_handle_control(const can_control_msg_t* cmd);

// Called from CAN RX callback when a heartbeat message is received
// Uses the global static pointer to the logic state, since the CAN callback
// doesn't have a way to pass user data
void logic_handle_heartbeat(const can_heartbeat_msg_t* heartbeat);

// Called from CAN RX callback when a VESC status 1 message is received
// Uses the global static pointer to the logic state, since the CAN callback
// doesn't have a way to pass user data
void logic_handle_vesc_status_1(const can_vesc_status_1_msg_t* vesc_status_1);


uint16_t logic_erpm_to_pwm(int32_t erpm);



#endif // LOGIC_H
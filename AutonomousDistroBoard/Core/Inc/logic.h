#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "ibus.h"
#include "can.h"
#include "util.h"


#define PRECHARGE_START_DELAY  (200) // ms, wait from boot before starting precharge
#define PRECHARGE_DURATION     (3000) // ms, how long to run precharge before closing contactor
#define CONTACTOR_CLOSED_DELAY (100) // ms, how long to wait after contactor is requested to be closed before considering it fully closed
#define ESTOP_TRIGGERED_DELAY  (3000) // ms, after remote estop is triggered, how long to wait before restarting the precharge sequence

#define IBUS_CHANNEL_THROTTLE (0) // 1000 = full stop, 2000 = full throttle forward
#define IBUS_CHANNEL_STEERING (1) // 1000 = full left, 1500 = center, 2000 = full right
#define IBUS_CHANNEL_MODE     (2) // ~1000 = RC mode, ~2000 = autonomous mode
#define IBUS_CHANNEL_ESTOP    (3) // ~1000 = not pressed, ~2000 = estop

#define ESTOP_PWM_THRESHOLD         (1500) // if the ESTOP channel goes above this value, consider the remote estop to be triggered
#define ESTOP_RISING_DEBOUNCE       (300) // ms, require the ESTOP channel to be above the threshold for at least this long before considering the remote estop to be triggered
#define ESTOP_ACCUMULATING_DEBOUNCE (30) // ms, when the rising ESTOP is debouncing/accumulating, require the ESTOP channel to be below the threshold for at least this long before resetting the debounce timer
#define ESTOP_FALLING_DEBOUNCE      (50) // ms, require the ESTOP channel to be below the threshold for at least this long before considering the remote estop to be no longer triggered

#define MODE_PWM_THRESHOLD (1500) // if the MODE channel is above this value, consider it to be in autonomous mode, otherwise RC mode
#define MODE_DEBOUNCE_MS (500) // ms, require the MODE channel to be consistently above or below the threshold for at least this long before switching modes
#define MODE_ACCUMULATING_DEBOUNCE_MS (50) // ms, when the rising MODE is debouncing/accumulating, require the MODE channel to be below the threshold for at least this long before resetting the debounce timer

#define CAN_TX_PERIOD  (10) // ms
#define CAN_RX_TIMEOUT (50) // ms

#define RC_CONNECTION_TIMEOUT (100) // ms, if we did not get a valid iBUS frame within this time, consider the RC connection to be lost

#define LED_STARTING_PERIOD (20) // ms
#define LED_PRECHARGING_PERIOD (100) // ms
#define LED_CONTACTOR_CLOSING_PERIOD (20) // ms
#define LED_RUNNING_RC_PERIOD (1000) // ms
#define LED_RUNNING_AUTONOMOUS_PERIOD (500) // ms
#define LED_ESTOPPED_PERIOD (0) // solid on
#define LED_RC_DISCONNECTED_PERIOD (0) // solid on
#define LED_CAN_DISCONNECTED_PERIOD (0) // solid on

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

#define RC_MODE_THROTTLE_DIVISOR (3) // divide the iBUS throttle PWM by this much

typedef enum {
	LOGIC_MODE_STARTING = 0,
	LOGIC_MODE_PRECHARGING,
	LOGIC_MODE_CONTACTOR_CLOSING,
	LOGIC_MODE_RUNNING,
	LOGIC_MODE_ESTOPPED,
	LOGIC_MODE_RC_DISCONNECTED,
	LOGIC_MODE_CAN_DISCONNECTED
} logic_mode_t;


typedef struct {
	logic_mode_t mode;

	ibus_t ibus;
	uint32_t last_can_tx_time;

	uint32_t start_time; // HAL_GetTick() timestamp of when the precharge sequence started
	
	debounce_controller_t estop_debounce; // debounce controller for the remote estop channel
	debounce_controller_t mode_debounce; // low = RC mode, high = autonomous mode

	volatile uint16_t can_current_throttle_pwm; // 1000-2000, updated by CAN RX callback
	volatile uint16_t can_current_steering_pwm; // 1000-2000, updated by CAN RX callback
	volatile uint32_t last_control_timestamp; // HAL_GetTick() timestamp of last received control message

	uint32_t led_blink_timestamp; // HAL_GetTick() timestamp of last LED toggle

	uint16_t output_throttle_pwm; // 1000-2000, the throttle PWM value that we will output (either from iBUS or CAN depending on mode)
	uint16_t output_steering_pwm; // 1000-2000, the steering PWM value that we will output (either from iBUS or CAN depending on mode)
	bool throttle_enabled; // whether the PWM channel for the throttle is enabled vs in input/high-impedance mode
} logic_state_t;

void logic_init(logic_state_t* state);


// Called once in the main loop
void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef *sbus_huart,
	CAN_HandleTypeDef *hcan,
	TIM_HandleTypeDef *throttle_htim,
	TIM_HandleTypeDef *steering_htim
);

// Called from CAN RX callback when a control message is received
// Uses the global static pointer to the logic state, since the CAN callback
// doesn't have a way to pass user data
void logic_handle_control(const can_control_msg_t* cmd);

void pwm_disable(TIM_HandleTypeDef* htim, GPIO_TypeDef* gpio_port, uint32_t gpio_pin);
void pwm_enable(TIM_HandleTypeDef* htim, GPIO_TypeDef* gpio_port, uint32_t gpio_pin, uint32_t alternate_function);





#endif // LOGIC_H
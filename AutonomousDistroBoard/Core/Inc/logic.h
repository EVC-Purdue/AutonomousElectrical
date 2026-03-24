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
#define ESTOP_ACCUMULATING_DEBOUNCE (30) // ms, when the rising estop is debouncing/accumulating, require the ESTOP channel to be below the threshold for at least this long before resetting the debounce timer
#define ESTOP_FALLING_DEBOUNCE      (50) // ms, require the ESTOP channel to be below the threshold for at least this long before considering the remote estop to be no longer triggered

#define CAN_TX_PERIOD  (10) // ms
// #define CAN_RX_TIMEOUT (50) // ms

#define RC_CONNECTION_TIMEOUT (100) // ms, if we did not get a valid iBUS frame within this time, consider the RC connection to be lost


typedef enum {
	LOGIC_MODE_STARTING = 0,
	LOGIC_MODE_PRECHARGING,
	LOGIC_MODE_CONTACTOR_CLOSING,
	LOGIC_MODE_RUNNING,
	LOGIC_MODE_ESTOPPED,
	LOGIC_MODE_RC_DISCONNECTED
} logic_mode_t;


typedef struct {
	logic_mode_t mode;

	ibus_t ibus;
	uint32_t last_can_tx_time;

	uint32_t boot_time; // HAL_GetTick() timestamp of when the system initialized
	debounce_controller_t estop_debounce; // debounce controller for the remote estop channel
	option_u32_t estop_triggered_time; // HAL_GetTick() timestamp of when the remote estop was triggered or none if it is not currently triggered

	volatile uint16_t can_current_throttle; // 0-1000
	volatile uint16_t can_current_steering; // 0-1000
	volatile uint32_t last_control_timestamp; // HAL_GetTick() timestamp of last received control message
} logic_state_t;

void logic_init(logic_state_t* state);


// Called once in the main loop
void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef *sbus_huart,
	CAN_HandleTypeDef *hcan
);

// Called from CAN RX callback when a control message is received
// Uses the global static pointer to the logic state, since the CAN callback
// doesn't have a way to pass user data
void logic_handle_control(const can_control_msg_t* cmd);





#endif // LOGIC_H
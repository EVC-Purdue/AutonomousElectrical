#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "ibus.h"
#include "can.h"


#define PRECHARGE_START_DELAY (200) // ms, wait from boot before starting precharge
#define PRECHARGE_DURATION (3000) // ms, how long to run precharge before closing contactor
#define CONTACTOR_CLOSED_DELAY (100) // ms, how long to wait after contactor is requested to be closed before considering it fully closed

typedef enum {
	LOGIC_MODE_BOOTING = 0,
	LOGIC_MODE_PRECHARGING,
	LOGIC_MODE_CONTACTOR_CLOSING,
	LOGIC_MODE_RUNNING,
} logic_mode_t;


typedef struct {
	logic_mode_t mode;

	ibus_t ibus;
	uint32_t last_can_tx_time;

	uint32_t boot_time; // HAL_GetTick() timestamp of when the system initialized

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
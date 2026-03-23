#include "logic.h"

void logic_init(logic_state_t* state) {
	ibus_init(&state->ibus);
	state->last_can_tx_time = 0;
}

void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef *sbus_huart,
	CAN_HandleTypeDef *hcan
) {
	// Process iBUS data
	ibus_process(&state->ibus, sbus_huart);
}
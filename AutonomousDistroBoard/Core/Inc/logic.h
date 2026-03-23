#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#include "ibus.h"

typedef struct {
	ibus_t ibus;
	uint32_t last_can_tx_time;
} logic_state_t;

void logic_init(logic_state_t* state);


// Called once in the main loop
void logic_run(
	logic_state_t* state,
	UART_HandleTypeDef *sbus_huart,
	CAN_HandleTypeDef *hcan
);





#endif // LOGIC_H
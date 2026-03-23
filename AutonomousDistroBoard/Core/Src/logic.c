#include "logic.h"


void logic_run(ibus_t* ibus, UART_HandleTypeDef *sbus_huart) {
	// Process iBUS data
	ibus_process(ibus, sbus_huart);
}
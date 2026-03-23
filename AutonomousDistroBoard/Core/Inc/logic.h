#ifndef LOGIC_H
#define LOGIC_H

#include "stm32f4xx_hal.h"

#include "ibus.h"


// Called once in the main loop
void logic_run(ibus_t* ibus, UART_HandleTypeDef *sbus_huart);





#endif // LOGIC_H
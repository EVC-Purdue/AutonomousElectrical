#ifndef IBUS_H

#include <stdint.h>

// #include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal.h"


#define IBUS_CHANNEL_COUNT (14)
#define IBUS_FRAME_SIZE (32)
#define IBUS_DMA_BUFFER_SIZE (128)


extern uint8_t g_ibus_dma_buffer[IBUS_DMA_BUFFER_SIZE];

extern uint8_t g_ibus_frame[IBUS_FRAME_SIZE];
extern uint8_t g_ibus_index;



// Should be called in the main loop / 1kHz task to process incoming iBUS data
void ibus_process(UART_HandleTypeDef *sbus_huart);
// Single-byte feed (sync finder style), called by ibus_process
void ibus_parse_byte(uint8_t byte);







#endif // IBUS_H
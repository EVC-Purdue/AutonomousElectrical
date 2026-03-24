#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>

#include "stm32f4xx_hal.h"


#define IBUS_CHANNEL_COUNT (14)
#define IBUS_FRAME_SIZE (32)
#define IBUS_DMA_BUFFER_SIZE (128)

#define IBUS_HEADER_DLC (0x20)
#define IBUS_HEADER_COMMAND (0x40)

#define IBUS_ESTOP_THRESHOLD (1500) // if the ESTOP channel goes above this value, consider the remote estop to be triggered
#define IBUS_ESTOP_RISING_DEBOUNCE_TIME (500) // ms, require the ESTOP channel to be above the threshold for at least this long before considering the remote estop to be triggered
#define IBUS_ESTOP_FALLING_DEBOUNCE_TIME (50) // ms, when the rising estop is debouncing/accumulating, require the ESTOP channel to be below the threshold for at least this long before resetting the debounce timer


// iBUS data structure
typedef struct {
	uint8_t dma_buffer[IBUS_DMA_BUFFER_SIZE]; // DMA buffer for incoming UART data
	uint8_t frame[IBUS_FRAME_SIZE]; // Buffer for assembling a single iBUS frame
	uint8_t frame_index; // Current index in the frame buffer
	uint16_t dma_last_pos; // Last processed position in the DMA buffer
	uint16_t channels[IBUS_CHANNEL_COUNT]; // PWM values
} ibus_t;


void ibus_init(ibus_t* ibus);


// Should be called in the main loop / 1kHz task to process incoming iBUS data
void ibus_process(ibus_t* ibus, UART_HandleTypeDef *sbus_huart);
// Single-byte feed (sync finder style), called by ibus_process
void ibus_parse_byte(ibus_t* ibus, uint8_t byte);







#endif // IBUS_H
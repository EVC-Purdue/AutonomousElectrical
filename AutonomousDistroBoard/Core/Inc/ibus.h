#ifndef IBUS_H
#define IBUS_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"


#define IBUS_CHANNEL_COUNT   (14)
#define IBUS_FRAME_SIZE      (32)
#define IBUS_DMA_BUFFER_SIZE (128)

#define IBUS_HEADER_DLC     (0x20)
#define IBUS_HEADER_COMMAND (0x40)


// iBUS data structure
typedef struct {
	uint32_t last_frame_time; // HAL_GetTick() timestamp of when the last valid frame was received
	uint8_t dma_buffer[IBUS_DMA_BUFFER_SIZE]; // DMA buffer for incoming UART data
	uint8_t frame[IBUS_FRAME_SIZE]; // Buffer for assembling a single iBUS frame
	uint8_t frame_index; // Current index in the frame buffer
	uint16_t dma_last_pos; // Last processed position in the DMA buffer
	uint16_t channels[IBUS_CHANNEL_COUNT]; // PWM values
} ibus_t;


void ibus_init(ibus_t* ibus);

bool ibus_is_connected(const ibus_t* ibus, uint32_t now, uint32_t timeout_ms);


// Should be called in the main loop / 1kHz task to process incoming iBUS data
void ibus_process(ibus_t* ibus, UART_HandleTypeDef *sbus_huart);
// Single-byte feed (sync finder style), called by ibus_process
void ibus_parse_byte(ibus_t* ibus, uint8_t byte);







#endif // IBUS_H
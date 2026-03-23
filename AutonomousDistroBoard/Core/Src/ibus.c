#include "ibus.h"

#include <stdint.h>
#include <string.h>


void ibus_init(ibus_t *ibus) {
    memset(ibus, 0, sizeof(*ibus));
}


void ibus_process(ibus_t* ibus, UART_HandleTypeDef* ibus_huart) {
    uint16_t dma_pos = IBUS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(ibus_huart->hdmarx);

    while (ibus->dma_last_pos != dma_pos) {
        uint8_t byte = ibus->dma_buffer[ibus->dma_last_pos++];

        if (ibus->dma_last_pos >= IBUS_DMA_BUFFER_SIZE) {
            ibus->dma_last_pos = 0;
		}

        ibus_parse_byte(ibus, byte);
    }
}



void ibus_parse_byte(ibus_t *ibus, uint8_t byte) {
	// Byte 0:     Length = 0x20 (32)
	// Byte 1:     Command = 0x40
	// Byte 2-29:  14 channels, little-endian uint16
	// Byte 30-31: checksum (little-endian)

    if (ibus->frame_index == 0 && byte != 0x20) {
        return; // wait for frame start
	}

    ibus->frame[ibus->frame_index++] = byte;
    if (ibus->frame_index == IBUS_FRAME_SIZE) {
        ibus->frame_index = 0;
        if (ibus->frame[1] == 0x40) {
            uint16_t checksum = 0xFFFF;
            for (int i = 0; i < 30; i++) {
				checksum -= ibus->frame[i];
			}

            uint16_t received = ibus->frame[30] | (ibus->frame[31] << 8);
            if (checksum == received) {
                for (int i = 0; i < IBUS_CHANNEL_COUNT; i++) {
					ibus->channels[i] = ibus->frame[2 + i*2] | (ibus->frame[3 + i*2] << 8);
				}
            }
        }
    }
}
#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"


#define CAN_BUS (CAN2)

#define CAN_TX_PERIOD (10) // ms

// - ID = `0x100` - **Control commands** (RX)
// 	- Byte 0-1: throttle (uint16_t, little endian), 0-1000, where 1000 = full throttle
// 	- Byte 2-3: steering (uint16_t, little endian), 0-1000, where 1000 = full right, 500 = center, 0 = full left
// 	- Byte 4-7: reserved / future use
#define CAN_ID_CONTROL (0x100)
typedef struct {
	uint16_t throttle; // 0-1000
	uint16_t steering; // 0-1000
} can_control_msg_t;


// - ID = `0x101` - **Status update** (TX)
// 	- Byte 0: precharge + contactor + rc mode (bit flags)
// 		- Bit0 = precharge
// 		- Bit1 = contactor
// 		- Bit2 = RC mode
// 		- Bits3-7 = 0
// 	- Byte 1-2: throttle PWM (uint16_t, little endian)
// 	- Byte 3-4: steering PWM (uint16_t, little endian)
// 	- Byte 5-7: reserved / future use
#define CAN_ID_STATUS (0x101)
typedef struct {
	bool precharge;
	bool contactor;
	bool rc_mode;
	uint16_t throttle_pwm;
	uint16_t steering_pwm;
} can_status_msg_t;


can_control_msg_t parse_can_control(const uint8_t* data);
void send_can_status(const can_status_msg_t* status, CAN_HandleTypeDef* hcan);



#endif // CAN_H
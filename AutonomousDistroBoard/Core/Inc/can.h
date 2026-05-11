#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"


#define CAN_BUS (CAN2)

#define CAN_VESC_ID (0xFF) // ID_ALL
#define CAN_VESC_SET_RPM_MSG_NUM (3)
#define CAN_VESC_STATUS_1_MSG_NUM (9)

#define CAN_VESC_MSG_NUM_TO_EXT_ID(msg_num) (((uint32_t)(msg_num) << 8) | ((uint32_t)(CAN_VESC_ID)))


// - ID = `0x100` - **Control commands** (RX)
// 	- Byte 0-1: throttle (uint16_t, little endian), 0-1000, where 1000 = full throttle
// 	- Byte 2-3: steering (uint16_t, little endian), 0-1000, where 1000 = full right, 500 = center, 0 = full left
// 	- Byte 4-7: reserved / future use
#define CAN_ID_CONTROL (0x100)
#define CAN_CONTROL_MIN_BYTES (4)
#define CAN_THROTTLE_MAX (1000)
#define CAN_STEERING_MAX (1000)
typedef struct {
	uint16_t throttle; // 0-1000
	uint16_t steering; // 0-1000
} can_control_msg_t;


// - ID = `0x101` - **Status update** (TX)
// 	- Byte 0: state machine mode + rc mode
// 		- Bits0-3 = state machine mode (see logic.h::logic_mode_t)
// 		- Bit4 = RC mode (0 = rc mode, 1 = autonomous mode)
// 		- Bits5-7 = reserved
// 	- Byte 1-2: throttle PWM (uint16_t, little endian), the actual PWM value being sent to the ESC for throttle (1000-2000)
// 	- Byte 3-4: steering PWM (uint16_t, little endian), the actual PWM value being sent to the servo for steering (1000-2000)
// 	- Byte 5-7: reserved / future use
#define CAN_ID_STATUS (0x101)
typedef struct {
	uint8_t mode; // logic_mode_t
	bool rc_mode; // false = rc mode, true = autonomous mode
	uint16_t throttle_pwm; // actual PWM value being sent to the ESC for throttle (1000-2000)
	uint16_t steering_pwm; // actual PWM value being sent to the servo for steering (1000-2000)
} can_status_msg_t;


// - ID = `0x102` - **E_Comms heartbeat** (RX)
// 	- Byte 0: E_Comms heartbeat counter (uint8_t)
// 	- Byte 1-7: reserved / future use
#define CAN_ID_HEARTBEAT (0x102)
#define CAN_HEARTBEAT_MIN_BYTES (1)
typedef struct {
	uint8_t counter; // E_Comms heartbeat counter
} can_heartbeat_msg_t;


bool can_is_able_to_parse(const CAN_RxHeaderTypeDef* rx_header, uint32_t expected_id, uint8_t expected_min_dlc);

can_control_msg_t parse_can_control(const uint8_t* data);
can_heartbeat_msg_t parse_can_heartbeat(const uint8_t* data);

void send_can_status(const can_status_msg_t* status, CAN_HandleTypeDef* hcan);

uint16_t can_throttle_to_pwm(const can_control_msg_t* cmd);
uint16_t can_steering_to_pwm(const can_control_msg_t* cmd);


#endif // CAN_H
#ifndef CAN_H
#define CAN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"


#define CAN_BUS (CAN2)

#define CAN_VESC_ID (7) // ID of the VESC
#define CAN_VESC_SET_RPM_MSG_NUM (3)
#define CAN_VESC_STATUS_1_MSG_NUM (9)

#define CAN_VESC_MSG_NUM_TO_EXT_ID(msg_num) (((uint32_t)(msg_num) << 8) | ((uint32_t)(CAN_VESC_ID)))


// - ID = `0x100` - **Control commands** (RX)
//	- Byte 0-1: throttle ERPM (uint16_t, little endian), 0-AUTONOMOUS_ERPM_MAX (0 = full stop, AUTONOMOUS_ERPM_MAX = max speed)
// 	- Byte 2-3: steering (uint16_t, little endian), 0-1000, where 1000 = full right, 500 = center, 0 = full left
// 	- Byte 4-7: reserved / future use
#define CAN_ID_CONTROL (0x100)
#define CAN_CONTROL_IS_EXT_ID (false)
#define CAN_CONTROL_MIN_BYTES (4)
#define CAN_STEERING_MIN (0)
#define CAN_STEERING_MAX (1000)
typedef struct {
	uint16_t throttle_erpm; // 0-AUTONOMOUS_ERPM_MAX
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
#define CAN_STATUS_IS_EXT_ID (false)
#define CAN_STATUS_MIN_BYTES (5)
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
#define CAN_HEARTBEAT_IS_EXT_ID (false)
#define CAN_HEARTBEAT_MIN_BYTES (1)
typedef struct {
	uint8_t counter; // E_Comms heartbeat counter
} can_heartbeat_msg_t;


// - ID = `CAN_VESC_MSG_NUM_TO_EXT_ID(9 = CAN_VESC_STATUS_1_MSG_NUM)` (ext id) - **VESC status 1** (RX)
// 	- Byte 0-3: VESC ERPM (BE)
// 	- Byte 4-5: VESC current (in 0.1A, so 100 = 10A) (BE)
// 	- Byte 6-7: VESC duty cycle (in 0.001, so 1000 = 100%) (BE)
#define CAN_ID_VESC_STATUS_1 (CAN_VESC_MSG_NUM_TO_EXT_ID(CAN_VESC_STATUS_1_MSG_NUM))
#define CAN_VESC_STATUS_1_IS_EXT_ID (true)
#define CAN_VESC_STATUS_1_MIN_BYTES (8)
typedef struct {
	int32_t erpm;
	int16_t current; // in 0.1A, so 100 = 10A
	int16_t duty_cycle; // in 0.001, so 1000 = 100%
} can_vesc_status_1_msg_t;


// - ID = `CAN_VESC_MSG_NUM_TO_EXT_ID(3 = CAN_VESC_SET_RPM_MSG_NUM)` (ext id) - **VESC set (E)RPM** (TX)
// 	- Byte 0-3: desired VESC ERPM (BE)
// 	- Byte 4-7: reserved
#define CAN_ID_VESC_SET_RPM (CAN_VESC_MSG_NUM_TO_EXT_ID(CAN_VESC_SET_RPM_MSG_NUM))
#define CAN_VESC_SET_RPM_IS_EXT_ID (true)
#define CAN_VESC_SET_RPM_MIN_BYTES (4)
typedef struct {
	int32_t erpm;
} can_vesc_set_rpm_msg_t;


bool can_is_able_to_parse(const CAN_RxHeaderTypeDef* rx_header, bool is_ext_id, uint32_t expected_id, uint8_t expected_min_dlc);

can_control_msg_t parse_can_control(const uint8_t* data);
can_heartbeat_msg_t parse_can_heartbeat(const uint8_t* data);
can_vesc_status_1_msg_t parse_can_vesc_status_1(const uint8_t* data);

void send_can_status(const can_status_msg_t* status, CAN_HandleTypeDef* hcan);
void send_can_vesc_set_rpm(const can_vesc_set_rpm_msg_t* msg, CAN_HandleTypeDef* hcan);

#endif // CAN_H
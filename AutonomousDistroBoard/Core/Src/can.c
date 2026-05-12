#include "can.h"
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "logic.h"
#include "util.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN_BUS) {
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
			return;
		}

		if (can_is_able_to_parse(&rx_header, CAN_CONTROL_IS_EXT_ID, CAN_ID_CONTROL, CAN_CONTROL_MIN_BYTES)) {
			can_control_msg_t cmd = parse_can_control(rx_data);
			logic_handle_control(&cmd);
		} else if (can_is_able_to_parse(&rx_header, CAN_HEARTBEAT_IS_EXT_ID, CAN_ID_HEARTBEAT, CAN_HEARTBEAT_MIN_BYTES)) {
			can_heartbeat_msg_t heartbeat = parse_can_heartbeat(rx_data);
			logic_handle_heartbeat(&heartbeat);
		} else if (can_is_able_to_parse(&rx_header, CAN_VESC_STATUS_1_IS_EXT_ID, CAN_ID_VESC_STATUS_1, CAN_VESC_STATUS_1_MIN_BYTES)) {
			can_vesc_status_1_msg_t vesc_status_1 = parse_can_vesc_status_1(rx_data);
			logic_handle_vesc_status_1(&vesc_status_1);
		}
    }
}

bool can_is_able_to_parse(
	const CAN_RxHeaderTypeDef* rx_header,
	bool is_ext_id,
	uint32_t expected_id,
	uint8_t expected_min_dlc
) {
	return (rx_header->IDE == CAN_ID_STD) &&
		   (rx_header->RTR == CAN_RTR_DATA) &&
		   (is_ext_id ? (rx_header->ExtId == expected_id) : (rx_header->StdId == expected_id)) &&
		   (rx_header->DLC >= expected_min_dlc);
}


can_control_msg_t parse_can_control(const uint8_t* data) {
	can_control_msg_t msg = {0};
	msg.throttle = data[0] | (data[1] << 8);
	msg.steering = data[2] | (data[3] << 8);
	return msg;
}

can_heartbeat_msg_t parse_can_heartbeat(const uint8_t* data) {
	can_heartbeat_msg_t msg = {0};
	msg.counter = data[0];
	return msg;
}

can_vesc_status_1_msg_t parse_can_vesc_status_1(const uint8_t* data) {
	can_vesc_status_1_msg_t msg = {0};
	// BIG ENDIAN
	msg.erpm =
		((int32_t)data[0] << 24) |
		((int32_t)data[1] << 16) |
		((int32_t)data[2] << 8)  |
		((int32_t)data[3]);
	msg.current =
		((int16_t)data[4] << 8) |
		((int16_t)data[5]);
	msg.duty_cycle =
		((int16_t)data[6] << 8) |
		((int16_t)data[7]);
	return msg;
}

void send_can_status(const can_status_msg_t* status, CAN_HandleTypeDef* hcan) {
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8] = {0};

	tx_header.StdId = CAN_ID_STATUS;
	tx_header.ExtId = 0;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.IDE = CAN_ID_STD;
	tx_header.DLC = 8;

	tx_data[0] = (status->mode & 0x0F) | ((uint8_t)status->rc_mode << 4);
	tx_data[1] = status->throttle_pwm & 0xFF;
	tx_data[2] = (status->throttle_pwm >> 8) & 0xFF;
	tx_data[3] = status->steering_pwm & 0xFF;
	tx_data[4] = (status->steering_pwm >> 8) & 0xFF;
	tx_data[5] = 0;
	tx_data[6] = 0;
	tx_data[7] = 0;

	uint32_t tx_mailbox;
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}

void send_can_vesc_set_rpm(const can_vesc_set_rpm_msg_t* msg, CAN_HandleTypeDef* hcan) {
	CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8] = {0};

	tx_header.StdId = 0;
	tx_header.ExtId = CAN_ID_VESC_SET_RPM;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.IDE = CAN_ID_EXT;
	tx_header.DLC = 8;

	// BIG ENDIAN
	tx_data[0] = (msg->erpm >> 24) & 0xFF;
	tx_data[1] = (msg->erpm >> 16) & 0xFF;
	tx_data[2] = (msg->erpm >> 8) & 0xFF;
	tx_data[3] = msg->erpm & 0xFF;
	tx_data[4] = 0;
	tx_data[5] = 0;
	tx_data[6] = 0;
	tx_data[7] = 0;

	uint32_t tx_mailbox;
	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &tx_mailbox);
}


uint16_t can_throttle_to_pwm(const can_control_msg_t* cmd) {
	return map_u16(cmd->throttle, 0, CAN_THROTTLE_MAX, THROTTLE_PWM_LOW, THROTTLE_PWM_HIGH);
}
uint16_t can_steering_to_pwm(const can_control_msg_t* cmd) {
	return map_u16(cmd->steering, 0, CAN_STEERING_MAX, STEERING_PWM_LOW, STEERING_PWM_HIGH);
}

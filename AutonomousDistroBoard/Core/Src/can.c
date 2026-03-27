#include "can.h"
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "logic.h"
#include "util.h"


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN_BUS) {
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
			return;
		}
		if ((rx_header.IDE == CAN_ID_STD) &&
			(rx_header.RTR == CAN_RTR_DATA) &&
			(rx_header.StdId == CAN_ID_CONTROL) &&
			(rx_header.DLC >= CAN_CONTROL_MIN_BYTES))
		{

			can_control_msg_t cmd = parse_can_control(rx_data);
			logic_handle_control(&cmd);
		}
    }
}


can_control_msg_t parse_can_control(const uint8_t* data) {
	can_control_msg_t msg;
	msg.throttle = data[0] | (data[1] << 8);
	msg.steering = data[2] | (data[3] << 8);
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


uint16_t can_throttle_to_pwm(const can_control_msg_t* cmd) {
	return map_u16(cmd->throttle, 0, CAN_THROTTLE_MAX, THROTTLE_PWM_LOW, THROTTLE_PWM_HIGH);
}
uint16_t can_steering_to_pwm(const can_control_msg_t* cmd) {
	return map_u16(cmd->steering, 0, CAN_STEERING_MAX, STEERING_PWM_LOW, STEERING_PWM_HIGH);
}
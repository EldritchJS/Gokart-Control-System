#include "av_com.h"

uint8_t data_frame[5];
uint8_t current_pos = 0;

void send_vehicle_status(UART_HandleTypeDef *huart, uint8_t message[], int size){
	HAL_UART_Transmit(huart, message, size, 14);// Sending in normal mode
}

void av_com_start_receiving(UART_HandleTypeDef *huart) {
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void av_com_stop_receiving(UART_HandleTypeDef *huart) {
	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
}

void av_com_irq_handler(UART_HandleTypeDef *huart) {
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		return;
	}

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == SET) {
		uint8_t data = (uint8_t) (huart->Instance->DR & (uint8_t) 0x00FF);
		data_frame[current_pos]  = data;
		current_pos ++;

		if (current_pos == 5){
			current_pos = 0;
		}

		return;
	}

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == SET) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		return;
	}
};

#include "uart_comm.h"

// Внутренние переменные
static UART_HandleTypeDef *huart = NULL;
static BoardType board_type = BOARD_A;
static uint8_t send_state = 0;
static uint8_t last_button = 1;
static uint8_t rx_buffer[10];
static volatile uint8_t rx_len = 0;

// Инициализация
void UART_COMM_Init(BoardType type, UART_HandleTypeDef *uart) {
	board_type = type;
	huart = uart;
	send_state = 0;
	last_button = 1;
	rx_len = 0;

	// Гасим светодиод
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	// Начинаем прием
	HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buffer, sizeof(rx_buffer));

	// Короткое подтверждение
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

// Основной цикл
void UART_COMM_Process(void) {
	uint8_t btn = HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin);

	// Обработка нажатия кнопки
	if (last_button == 1 && btn == 0) {
		HAL_Delay(20);

		if (HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin) == 0) {
			uint8_t tx_char;
			if (board_type == BOARD_A) {
				tx_char = (send_state == 0) ? 'A' : 'a';
			} else {
				tx_char = (send_state == 0) ? 'B' : 'b';
			}

			HAL_UART_Transmit(huart, &tx_char, 1, 100);
			send_state = !send_state;

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			HAL_Delay(80);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
	}

	last_button = btn;

	// Обработка принятых данных
	if (rx_len > 0) {
		for (int i = 0; i < rx_len; i++) {
			if (rx_buffer[i] == 'A' || rx_buffer[i] == 'B') {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			} else if (rx_buffer[i] == 'a' || rx_buffer[i] == 'b') {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			}
		}

		HAL_UARTEx_ReceiveToIdle_IT(huart, rx_buffer, sizeof(rx_buffer));
		rx_len = 0;
	}

	HAL_Delay(1);
}

// Обработчик прерывания
void UART_COMM_RxCallback(UART_HandleTypeDef *uart, uint16_t len) {
	if (huart != NULL && uart == huart) {
		rx_len = len;
	}
}

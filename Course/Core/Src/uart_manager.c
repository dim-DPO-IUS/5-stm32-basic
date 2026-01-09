#include "uart_manager.h"
#include "main.h"
#include "pid_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// ВНЕШНИЕ ПЕРЕМЕННЫЕ (из main.c)
// ============================================================================
extern UART_HandleTypeDef huart1;  // UART для приема команд

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ UART (определяем здесь)
// ============================================================================
uint8_t rx_data = 0;
char buffer[10] = { 0 };
uint8_t inx = 0;

// ============================================================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ (точно как в main.c)
// ============================================================================

// Обработчик прерывания UART (оставляем как есть)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (rx_data == '\n') {
			if (inx > 0) {
				buffer[inx] = '\0';
				int value = atoi(buffer);

				// ПРОВЕРКА ДИАПАЗОНА!
				if (value < 0)
					value = 0;
				if (value > 1023)
					value = 1023;

				// Преобразуем 0-1023 → 0-3300 милливольт
				setpoint = setpoint_to_millivolts((uint16_t) value);
				inx = 0;
			}
		} else if (rx_data >= '0' && rx_data <= '9') {
			if (inx < sizeof(buffer) - 1) {
				buffer[inx++] = rx_data;
			}
		}
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);  // Используем extern huart1
	}
}

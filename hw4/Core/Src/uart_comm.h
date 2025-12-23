#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"

// Типы плат
typedef enum {
	BOARD_A = 0,  // Отправляет A/a
	BOARD_B = 1   // Отправляет B/b
} BoardType;

// Инициализация
void UART_COMM_Init(BoardType type, UART_HandleTypeDef *huart);

// Основной цикл обработки
void UART_COMM_Process(void);

// Обработчик прерывания (вызывать из HAL_UARTEx_RxEventCallback)
void UART_COMM_RxCallback(UART_HandleTypeDef *huart, uint16_t len);

#endif

#ifndef UART_MANAGER_H
#define UART_MANAGER_H

#include "main.h"  //  для UART_HandleTypeDef
#include <stdint.h>

// ============================================================================
// ВНЕШНИЕ ПЕРЕМЕННЫЕ UART (объявляются в uart_manager.c)
// ============================================================================
extern uint8_t rx_data;
extern char buffer[10];
extern uint8_t inx;

// ============================================================================
// ФУНКЦИИ UART (точно как в main.c)
// ============================================================================

// Обработчик прерывания UART (оставляем как есть)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif // UART_MANAGER_H

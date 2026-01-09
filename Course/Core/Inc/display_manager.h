/*
 * display_manager.h
 *
 *  Created on: Jan 9, 2026
 *      Author: dim0k
 */
#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <stdint.h>

// ============================================================================
// ФУНКЦИИ ОТОБРАЖЕНИЯ PID ИНТЕРФЕЙСА (точно как в main.c)
// ============================================================================

// Функция для рисования PID значения в формате X.XXX увеличенным в 2 раза
void DrawPIDValue2x(uint16_t x, uint16_t y, int32_t value, uint16_t color);

// Основной интерфейс PID
void DisplayPIDInterface(void);

// Упрощенный интерфейс PID
void DisplayPIDInterfaceMono2x(void);

// Функции обновления дисплея
void UpdatePIDDisplay(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent);

void UpdatePIDDisplaySimple(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent);

void UpdatePIDDisplayMono2x(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent);

#endif // DISPLAY_MANAGER_H

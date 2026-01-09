/*
 *
 * */

#include "display_manager.h"
#include "tft_st7789.h"
#include "pid_controller.h"  //  KP, KI, KD
#include <stdio.h>
#include <string.h>
#include <stdlib.h>          //  abs/labs

// ============================================================================
// РЕАЛИЗАЦИЯ ФУНКЦИЙ
// ============================================================================

// Функция для рисования PID значения в формате X.XXX увеличенным в 2 раза
void DrawPIDValue2x(uint16_t x, uint16_t y, int32_t value, uint16_t color) {
	char buffer[16]; // Увеличили с 10 до 16 (" -1.999" + \0 = 8, лучше с запасом)

	// Форматируем всегда 6 символов: [знак][цифра].[3цифры]
	// Примеры: " 1.650", "-0.027"
	if (value < 0) {
		snprintf(buffer, sizeof(buffer), "-%01ld.%03ld", (-value) / 1000,
				(-value) % 1000);
	} else {
		snprintf(buffer, sizeof(buffer), " %01ld.%03ld", value / 1000,
				value % 1000);
	}

	// Рисуем увеличенным в 2 раза
	DrawMonoText2x(x, y, buffer, color, TFT_BLACK);
}
// ----------------------------------------------------------------------------

void DisplayPIDInterface(void) {
	TFT_FillScreen(TFT_BLACK);

	// 1. Заголовок
	TFT_DrawStringCentered(5, "PID CONTROLLER", FONT_SMALL, TFT_YELLOW,
	TFT_BLACK);

	// 2. Разделительная линия
	TFT_DrawFastHLine(10, 25, TFT_WIDTH - 20, TFT_GRAY);

	// 3. Метки параметров
	TFT_DrawString(15, 35, "SETPOINT:", FONT_SMALL, TFT_WHITE, TFT_BLACK);
	TFT_DrawString(15, 60, "FEEDBACK:", FONT_SMALL, TFT_WHITE, TFT_BLACK);
	TFT_DrawString(15, 85, "ERROR:", FONT_SMALL, TFT_WHITE, TFT_BLACK);

	// 4. Индикатор ошибки
	TFT_DrawString(15, 110, "ERROR BAR:", FONT_SMALL, TFT_WHITE, TFT_BLACK);
	TFT_DrawErrorBar(15, 125, TFT_WIDTH - 30, 12, 0, TFT_DARKGRAY,
	TFT_DARKGRAY);
	TFT_DrawFastVLine(15 + (TFT_WIDTH - 30) / 2, 125, 12, TFT_WHITE);

	// 5. Коэффициенты PID (теперь используем KP, KI, KD из pid_controller.h)
	TFT_DrawString(15, 145, "PID COEFFICIENTS:", FONT_SMALL, TFT_YELLOW,
	TFT_BLACK);
	TFT_DrawLabeledValue(15, 160, "Kp:", KP, 0, NULL, FONT_SMALL, FONT_SMALL,
	TFT_WHITE, TFT_GREEN);
	TFT_DrawLabeledValue(80, 160, "Ki:", KI, 0, NULL, FONT_SMALL, FONT_SMALL,
	TFT_WHITE, TFT_GREEN);
	TFT_DrawLabeledValue(145, 160, "Kd:", KD, 0, NULL, FONT_SMALL, FONT_SMALL,
	TFT_WHITE, TFT_GREEN);

	// 6. Выходной сигнал
	TFT_DrawString(15, 180, "OUTPUT POWER:", FONT_SMALL, TFT_YELLOW, TFT_BLACK);
	TFT_DrawHProgressBar(15, 195, TFT_WIDTH - 30, 15, 0, TFT_BLUE,
	TFT_DARKGRAY);

	// 7. Крупное значение и единицы измерения
	TFT_DrawString(TFT_WIDTH / 2 + 40, 216, "V", FONT_MEDIUM, TFT_WHITE,
	TFT_BLACK);
}

// ----------------------------------------------------------------------------

void UpdatePIDDisplay(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent) {
	// ФИКСИРОВАННЫЕ КООРДИНАТЫ И ШИРИНЫ
#define SETPOINT_X      85
#define FEEDBACK_X      85
#define ERROR_X         85
#define PERCENT_X       (TFT_WIDTH - 40)
#define LARGE_VALUE_X   (TFT_WIDTH / 2 - 25)

#define VALUE_WIDTH     70   // "-X.XXX" + запас
#define PERCENT_WIDTH   25   // "100%"
#define LARGE_WIDTH     60   // Крупные цифры

	// 1. Setpoint
	TFT_DrawFixedPointEx(SETPOINT_X, 35, setpoint, 3, FONT_MEDIUM,
	TFT_GREEN, TFT_BLACK, VALUE_WIDTH);

	// 2. Feedback
	TFT_DrawFixedPointEx(FEEDBACK_X, 60, feedback, 3, FONT_MEDIUM,
	TFT_CYAN, TFT_BLACK, VALUE_WIDTH);

	// 3. Error
	TFT_DrawFixedPointEx(ERROR_X, 85, error, 3, FONT_MEDIUM,
	TFT_RED, TFT_BLACK, VALUE_WIDTH);

	// 4. Индикатор ошибки (перерисовываем целиком)
	TFT_DrawErrorBar(15, 125, TFT_WIDTH - 30, 12, error_percent, TFT_DARKGRAY,
	TFT_RED);

	// 5. Прогресс-бар (перерисовываем целиком)
	TFT_DrawProgressBar(15, 195, TFT_WIDTH - 30, 15, output_percent, TFT_BLUE,
	TFT_DARKGRAY);

	// 6. Процент мощности
	if (output_percent < 1000) {
		uint16_t display_value = output_percent / 10;
		uint8_t decimals = 1;
		if (output_percent % 10 == 0) {
			display_value = output_percent / 10;
			decimals = 0;
		}
		TFT_DrawFixedPointEx(PERCENT_X, 180, display_value, decimals,
		FONT_SMALL, TFT_WHITE, TFT_BLACK, PERCENT_WIDTH);
		TFT_DrawChar(PERCENT_X + 15, 180, '%', FONT_SMALL, TFT_WHITE,
		TFT_BLACK);
	} else {
		TFT_DrawString(PERCENT_X, 180, "100%", FONT_SMALL, TFT_WHITE,
		TFT_BLACK);
	}

	// 7. Крупное значение (только если изменилось значительно)
	static int32_t last_large_feedback = -1;
	if (labs(feedback - last_large_feedback) > 5) { // Изменение > 0.005V, используем labs для int32_t
		TFT_DrawFixedPointEx(LARGE_VALUE_X, 220, feedback, 3, FONT_LARGE,
		TFT_WHITE, TFT_BLACK, LARGE_WIDTH);
		last_large_feedback = feedback;
	}
}

// ----------------------------------------------------------------------------

void UpdatePIDDisplaySimple(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent) {
	char buffer[20];

	// 1. Setpoint
	sprintf(buffer, "%ld", setpoint);
	TFT_DrawString(85, 35, buffer, FONT_MEDIUM, TFT_GREEN, TFT_BLACK);

	// 2. Feedback
	sprintf(buffer, "%ld", feedback);
	TFT_DrawString(85, 60, buffer, FONT_MEDIUM, TFT_CYAN, TFT_BLACK);

	// 3. Error
	sprintf(buffer, "%ld", error);
	TFT_DrawString(85, 85, buffer, FONT_MEDIUM, TFT_RED, TFT_BLACK);

	// 4. Output percent
	sprintf(buffer, "%d%%", output_percent / 10);
	TFT_DrawString(200, 180, buffer, FONT_SMALL, TFT_WHITE, TFT_BLACK);
}

// ----------------------------------------------------------------------------

void UpdatePIDDisplayMono2x(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent) {
	// 1. Основные значения
	DrawPIDValue2x(85, 35, setpoint, TFT_GREEN);
	DrawPIDValue2x(85, 60, feedback, TFT_CYAN);
	DrawPIDValue2x(85, 85, error, TFT_RED);

	// 2. Output percent
	char output_buffer[12];  // Увеличили с 10 до 12
	snprintf(output_buffer, sizeof(output_buffer), "%d%%", output_percent / 10);
	DrawMonoText2x(200, 180, output_buffer, TFT_WHITE, TFT_BLACK);

	// 3. Индикатор ошибки
	TFT_DrawErrorBar(15, 125, TFT_WIDTH - 30, 12, error_percent, TFT_DARKGRAY,
	TFT_RED);

	// 4. Прогресс-бар
	TFT_DrawHProgressBar(15, 195, TFT_WIDTH - 30, 15, output_percent, TFT_BLUE,
	TFT_DARKGRAY);

	// 5. Крупное значение
	static int32_t last_large_feedback = -1;
	if (labs(feedback - last_large_feedback) > 5) {
		char large_buffer[16];  // Увеличили с 10 до 16

		if (feedback < 0) {
			snprintf(large_buffer, sizeof(large_buffer), "-%01ld.%03ld",
					(-feedback) / 1000, (-feedback) % 1000);
		} else {
			snprintf(large_buffer, sizeof(large_buffer), " %01ld.%03ld",
					feedback / 1000, feedback % 1000);
		}
		// Рисуем увеличенным в 2 раза
		DrawMonoText2x(TFT_WIDTH / 2 - 40, 216, large_buffer, TFT_CYAN,
		TFT_BLACK);
		last_large_feedback = feedback;
	}
}

// ----------------------------------------------------------------------------

void DisplayPIDInterfaceMono2x(void) {
	TFT_FillScreen(TFT_BLACK);

	// 1. Заголовок
	TFT_DrawStringCentered(5, "PID CONTROLLER", FONT_SMALL, TFT_YELLOW,
	TFT_BLACK);

	// 2. Метки параметров увеличенные 2x
	DrawMonoText2x(15, 35, "SETPOINT:", TFT_WHITE, TFT_BLACK);
	DrawMonoText2x(15, 60, "FEEDBACK:", TFT_WHITE, TFT_BLACK);
	DrawMonoText2x(15, 85, "ERROR:", TFT_WHITE, TFT_BLACK);

	// 3. Остальные элементы как были...
	TFT_DrawFastHLine(10, 25, TFT_WIDTH - 20, TFT_GRAY);
	// ... остальной код DisplayPIDInterface можно скопировать при необходимости
}

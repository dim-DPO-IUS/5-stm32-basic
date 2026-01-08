/*
 * tft_st7789.c
 *
 *  Created on: Jan 7, 2026
 *      Author: dim0k
 */
#include "tft_st7789.h"
#include <string.h>
#include <stdio.h>  // Добавляем для snprintf

static SPI_HandleTypeDef *tft_spi = NULL;

// Внутренние макросы для управления пинами
#define TFT_RST_LOW()   HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET)
#define TFT_RST_HIGH()  HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET)
#define TFT_DC_CMD()    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET)
#define TFT_DC_DATA()   HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET)

// Внутренние функции SPI
static void TFT_WriteCommand(uint8_t cmd) {
	TFT_DC_CMD();
	HAL_SPI_Transmit(tft_spi, &cmd, 1, 10);
}

static void TFT_WriteData(uint8_t data) {
	TFT_DC_DATA();
	HAL_SPI_Transmit(tft_spi, &data, 1, 10);
}

static void TFT_WriteDataBurst(uint8_t *data, uint32_t size) {
	TFT_DC_DATA();
	HAL_SPI_Transmit(tft_spi, data, size, 1000);
}

// Инициализация (на основе Fixed_ST7789_Init)
void TFT_Init(SPI_HandleTypeDef *hspi) {
	tft_spi = hspi;

	// 1. Аппаратный сброс
	TFT_RST_LOW();
	HAL_Delay(10);
	TFT_RST_HIGH();
	HAL_Delay(120);

	// 2. Software Reset (если нужно)
	TFT_WriteCommand(0x01); // SWRESET
	HAL_Delay(150);

	// 3. Sleep Out
	TFT_WriteCommand(0x11); // SLPOUT
	HAL_Delay(255);

	// 4. Color Mode 16-bit
	TFT_WriteCommand(0x3A); // COLMOD
	TFT_WriteData(0x55);    // 16-bit RGB565

	// 5. Display Inversion On
	TFT_WriteCommand(0x21); // INVON
	HAL_Delay(10);

	// 6. Display On
	TFT_WriteCommand(0x29); // DISPON
	HAL_Delay(100);

	// 7. Очистка экрана
	TFT_FillScreen(TFT_BLACK);
}

// Установка окна (на основе Simple_Fill_Color)
void TFT_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	// CASET - установка колонок
	TFT_WriteCommand(0x2A);
	TFT_WriteData(x0 >> 8);
	TFT_WriteData(x0 & 0xFF);
	TFT_WriteData(x1 >> 8);
	TFT_WriteData(x1 & 0xFF);

	// RASET - установка строк
	TFT_WriteCommand(0x2B);
	TFT_WriteData(y0 >> 8);
	TFT_WriteData(y0 & 0xFF);
	TFT_WriteData(y1 >> 8);
	TFT_WriteData(y1 & 0xFF);

	// RAMWR - готовность к записи пикселей
	TFT_WriteCommand(0x2C);
}

// Заливка экрана (оптимизированная версия Simple_Fill_Color)
void TFT_FillScreen(uint16_t color) {
	uint8_t hi = color >> 8;
	uint8_t lo = color & 0xFF;

	// Устанавливаем окно на весь экран
	TFT_SetWindow(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

	// Отправляем данные
	TFT_DC_DATA();

	// Буфер для ускорения передачи
	uint8_t buffer[64]; // 32 пикселя
	for (int i = 0; i < 64; i += 2) {
		buffer[i] = hi;
		buffer[i + 1] = lo;
	}

	uint32_t total_pixels = TFT_WIDTH * TFT_HEIGHT;
	for (uint32_t i = 0; i < total_pixels; i += 32) {
		HAL_SPI_Transmit(tft_spi, buffer, 64, 100);
	}
}

// ============================================================================
// БАЗОВЫЕ ГРАФИЧЕСКИЕ ПРИМИТИВЫ
// ============================================================================

/**
 * @brief Рисование точки (пикселя)
 * @param x, y Координаты точки (0..TFT_WIDTH-1, 0..TFT_HEIGHT-1)
 * @param color Цвет в формате RGB565
 */
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
	// Проверка границ экрана
	if (x >= TFT_WIDTH || y >= TFT_HEIGHT) {
		return;
	}

	// Устанавливаем окно на один пиксель
	TFT_SetWindow(x, y, x, y);

	// Отправляем цвет пикселя
	TFT_DC_DATA();
	uint8_t color_data[] = { color >> 8, color & 0xFF };
	HAL_SPI_Transmit(tft_spi, color_data, sizeof(color_data), 10);
}

/**
 * @brief Быстрая горизонтальная линия
 * @param x, y Начальная точка линии
 * @param w Длина линии в пикселях
 * @param color Цвет линии
 */
void TFT_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) {
	// Проверка границ
	if (y >= TFT_HEIGHT)
		return;
	if (x >= TFT_WIDTH)
		return;

	// Ограничение длины линии границами экрана
	if (x + w > TFT_WIDTH) {
		w = TFT_WIDTH - x;
	}
	if (w == 0)
		return;

	// Устанавливаем окно для линии
	TFT_SetWindow(x, y, x + w - 1, y);

	// Отправляем данные
	TFT_DC_DATA();
	uint8_t hi = color >> 8;
	uint8_t lo = color & 0xFF;

	// Оптимизация: буфер для нескольких пикселей
#define LINE_BUFFER_SIZE 32
	uint8_t buffer[LINE_BUFFER_SIZE * 2];

	// Заполняем буфер цветом
	for (int i = 0; i < LINE_BUFFER_SIZE * 2; i += 2) {
		buffer[i] = hi;
		buffer[i + 1] = lo;
	}

	// Отправляем линию частями
	uint16_t pixels_drawn = 0;
	while (pixels_drawn < w) {
		uint16_t chunk = w - pixels_drawn;
		if (chunk > LINE_BUFFER_SIZE) {
			chunk = LINE_BUFFER_SIZE;
		}

		HAL_SPI_Transmit(tft_spi, buffer, chunk * 2, 100);
		pixels_drawn += chunk;
	}
}

/**
 * @brief Быстрая вертикальная линия
 * @param x, y Начальная точка линии
 * @param h Высота линии в пикселях
 * @param color Цвет линии
 */
void TFT_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) {
	// Проверка границ
	if (x >= TFT_WIDTH)
		return;
	if (y >= TFT_HEIGHT)
		return;

	// Ограничение высоты линии границами экрана
	if (y + h > TFT_HEIGHT) {
		h = TFT_HEIGHT - y;
	}
	if (h == 0)
		return;

	// Для вертикальной линии лучше рисовать по точкам,
	// так как установка окна на каждый пиксель отдельно неэффективна

	for (uint16_t i = 0; i < h; i++) {
		TFT_DrawPixel(x, y + i, color);
	}
}

/**
 * @brief Прямоугольник (контур)
 * @param x, y Координаты левого верхнего угла
 * @param w, h Ширина и высота
 * @param color Цвет контура
 */
void TFT_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color) {
	// Верхняя горизонтальная линия
	TFT_DrawFastHLine(x, y, w, color);

	// Нижняя горизонтальная линия
	if (h > 1) {
		TFT_DrawFastHLine(x, y + h - 1, w, color);
	}

	// Левая вертикальная линия
	if (h > 2) {
		TFT_DrawFastVLine(x, y + 1, h - 2, color);
	}

	// Правая вертикальная линия
	if (h > 2 && w > 1) {
		TFT_DrawFastVLine(x + w - 1, y + 1, h - 2, color);
	}
}

/**
 * @brief Залитый прямоугольник
 * @param x, y Координаты левого верхнего угла
 * @param w, h Ширина и высота
 * @param color Цвет заливки
 */
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color) {
	// Проверка границ
	if (x >= TFT_WIDTH || y >= TFT_HEIGHT)
		return;

	// Ограничение размера границами экрана
	if (x + w > TFT_WIDTH)
		w = TFT_WIDTH - x;
	if (y + h > TFT_HEIGHT)
		h = TFT_HEIGHT - y;
	if (w == 0 || h == 0)
		return;

	// Устанавливаем окно на прямоугольник
	TFT_SetWindow(x, y, x + w - 1, y + h - 1);

	// Отправляем данные
	TFT_DC_DATA();
	uint8_t hi = color >> 8;
	uint8_t lo = color & 0xFF;

	// Буфер для ускорения передачи
#define RECT_BUFFER_SIZE 64  // 32 пикселя
	uint8_t buffer[RECT_BUFFER_SIZE];

	// Заполняем буфер цветом
	for (int i = 0; i < RECT_BUFFER_SIZE; i += 2) {
		buffer[i] = hi;
		buffer[i + 1] = lo;
	}

	// Отправляем весь прямоугольник
	uint32_t total_pixels = w * h;
	uint32_t pixels_sent = 0;

	while (pixels_sent < total_pixels) {
		uint32_t chunk_pixels = total_pixels - pixels_sent;
		if (chunk_pixels * 2 > RECT_BUFFER_SIZE) {
			chunk_pixels = RECT_BUFFER_SIZE / 2;
		}

		HAL_SPI_Transmit(tft_spi, buffer, chunk_pixels * 2, 100);
		pixels_sent += chunk_pixels;
	}
}

// ============================================================================
// РАБОТА С ТЕКСТОМ
// ============================================================================

void TFT_DrawChar(uint16_t x, uint16_t y, char c, uint8_t font_id,
		uint16_t color, uint16_t bgcolor) {
	uint8_t *char_table;
	uint8_t width, height;
	uint8_t row, col;
	uint8_t byte;

	char_table = font_GetFontStruct(font_id, (uint8_t) c);
	if (char_table == NULL)
		return;

	width = font_GetCharWidth(char_table);
	height = font_GetCharHeight(char_table);
	char_table += 2;

	// Количество байт на строку
	uint8_t bytes_per_row = (width + 7) / 8;

	// Проходим по всем пикселям
	for (row = 0; row < height; row++) {
		for (col = 0; col < width; col++) {
			uint8_t byte_index = col / 8;
			uint8_t bit_index = 7 - (col % 8);
			byte = char_table[row * bytes_per_row + byte_index];

			if (byte & (1 << bit_index)) {
				// Бит установлен - рисуем пиксель текста
				TFT_DrawPixel(x + col, y + row, color);
			} else if (bgcolor != TFT_NO_BG) {
				// Бит не установлен и фон не прозрачный - рисуем фон
				TFT_DrawPixel(x + col, y + row, bgcolor);
			}
			// Если фон прозрачный (TFT_NO_BG) - ничего не делаем
		}
	}
}

void TFT_DrawString(uint16_t x, uint16_t y, const char *str, uint8_t font_id,
		uint16_t color, uint16_t bgcolor) {
	uint16_t x_pos = x;
	uint8_t *char_table;

	while (*str) {
		// Отрисовываем текущий символ С ФОНОМ В ОДНУ ОПЕРАЦИЮ
		TFT_DrawChar(x_pos, y, *str, font_id, color, bgcolor); // bgcolor передается!

		// Получаем ширину отрисованного символа для смещения
		char_table = font_GetFontStruct(font_id, (uint8_t) *str);
		if (char_table != NULL) {
			x_pos += font_GetCharWidth(char_table) + 1;
		} else {
			x_pos += 6;
		}

		str++;
	}
}

void TFT_DrawStringCentered(uint16_t y, const char *str, uint8_t font_id,
		uint16_t color, uint16_t bgcolor) {
	uint16_t total_width = 0;
	uint8_t *char_table;
	const char *ptr = str;
	uint16_t x_pos;

	// Вычисляем общую ширину строки
	while (*ptr) {
		char_table = font_GetFontStruct(font_id, (uint8_t) *ptr);
		if (char_table != NULL) {
			total_width += font_GetCharWidth(char_table) + 1;
		} else {
			total_width += 7;
		}
		ptr++;
	}

	// Убираем последний межсимвольный пробел
	if (total_width > 0) {
		total_width--;
	}

	// Вычисляем позицию X для центрирования
	if (total_width >= TFT_WIDTH) {
		x_pos = 0; // Строка слишком длинная, начинаем с края
	} else {
		x_pos = (TFT_WIDTH - total_width) / 2;
	}

	// Рисуем строку
	TFT_DrawString(x_pos, y, str, font_id, color, bgcolor);
}

// ============================================================================
// PID-СПЕЦИФИЧНЫЕ ФУНКЦИИ (целочисленные версии)
// ============================================================================

/**
 * @brief Отрисовка значения с меткой и единицами измерения (целочисленная версия)
 * @param x, y Координаты левого верхнего угла
 * @param label Текст метки (например "SP:")
 * @param value Целое значение (в тысячных долях, например 3300 = 3.300V)
 * @param decimals Количество знаков после запятой (0-3)
 * @param unit Единицы измерения (например "V", NULL если не нужны)
 * @param font_label_id ID шрифта для метки
 * @param font_value_id ID шрифта для значения
 * @param label_color Цвет метки
 * @param value_color Цвет значения
 */
void TFT_DrawLabeledValue(uint16_t x, uint16_t y, const char *label,
		int32_t value, uint8_t decimals, const char *unit,
		uint8_t font_label_id, uint8_t font_value_id, uint16_t label_color,
		uint16_t value_color) {
	char buffer[16];
	uint8_t *char_table;
	uint16_t label_width = 0;
	uint16_t value_x;
	const char *ptr = label;

	// 1. Рисуем метку (например, "SP:")
	TFT_DrawString(x, y, label, font_label_id, label_color, TFT_NO_BG);

	// 2. Вычисляем ширину метки для позиционирования значения
	while (*ptr) {
		char_table = font_GetFontStruct(font_label_id, (uint8_t) *ptr);
		if (char_table != NULL) {
			label_width += font_GetCharWidth(char_table) + 1;
		} else {
			label_width += 6;
		}
		ptr++;
	}

	// 3. Подготавливаем строку со значением (целочисленная обработка)
	if (value < 0) {
		// Отрицательное значение
		value = -value; // Делаем положительным для обработки
		if (decimals == 0) {
			snprintf(buffer, sizeof(buffer), "-%ld", (long) value);
		} else if (decimals == 1) {
			snprintf(buffer, sizeof(buffer), "-%ld.%01ld", (long) (value / 10),
					(long) (value % 10));
		} else if (decimals == 2) {
			snprintf(buffer, sizeof(buffer), "-%ld.%02ld", (long) (value / 100),
					(long) (value % 100));
		} else if (decimals == 3) {
			snprintf(buffer, sizeof(buffer), "-%ld.%03ld",
					(long) (value / 1000), (long) (value % 1000));
		}
	} else {
		// Положительное значение
		if (decimals == 0) {
			snprintf(buffer, sizeof(buffer), "%ld", (long) value);
		} else if (decimals == 1) {
			snprintf(buffer, sizeof(buffer), "%ld.%01ld", (long) (value / 10),
					(long) (value % 10));
		} else if (decimals == 2) {
			snprintf(buffer, sizeof(buffer), "%ld.%02ld", (long) (value / 100),
					(long) (value % 100));
		} else if (decimals == 3) {
			snprintf(buffer, sizeof(buffer), "%ld.%03ld", (long) (value / 1000),
					(long) (value % 1000));
		}
	}

	// 4. Добавляем единицы измерения, если указаны
	if (unit != NULL) {
		strcat(buffer, unit);
	}

	// 5. Рисуем значение справа от метки
	value_x = x + label_width + 4; // Небольшой отступ от метки
	TFT_DrawString(value_x, y, buffer, font_value_id, value_color, TFT_NO_BG);
}

/**
 * @brief Отрисовка целого числа с фиксированной точкой
 * @param x, y Координаты
 * @param value Целое значение (в тысячных долях, например 3141 = 3.141)
 * @param decimals Количество знаков после запятой (0-3)
 * @param font_id ID шрифта
 * @param color Цвет текста
 * @param bgcolor Цвет фона
 */
void TFT_DrawFixedPoint(uint16_t x, uint16_t y, int32_t value, uint8_t decimals,
		uint8_t font_id, uint16_t color, uint16_t bgcolor) {
	char buffer[16];
	char format[8];

	// Создаем строку формата
	snprintf(format, sizeof(format), "%%.%df", decimals);
	snprintf(buffer, sizeof(buffer), format, (double) value / 1000.0);

	// РИСУЕМ С ФОНОМ В ОДНУ ОПЕРАЦИЮ
	TFT_DrawString(x, y, buffer, font_id, color, bgcolor);
}

/**
 * @brief Отрисовка числа с фиксированной точкой и фиксированной шириной области
 * @param x, y Координаты левого верхнего угла
 * @param value Значение в тысячных долях
 * @param decimals Количество знаков после запятой
 * @param font_id ID шрифта
 * @param color Цвет текста
 * @param bg_color Цвет фона
 * @param area_width Фиксированная ширина области
 */
void TFT_DrawFixedPointEx(uint16_t x, uint16_t y, int32_t value,
		uint8_t decimals, uint8_t font_id, uint16_t color, uint16_t bg_color,
		uint16_t area_width) {
	char buffer[16];
	uint8_t *char_table;
	uint16_t text_width = 0;
	uint16_t x_offset = 0;
	uint16_t area_height = 8; // По умолчанию для мелких шрифтов

	// Формируем строку
	if (value < 0) {
		value = -value;
		if (decimals == 0) {
			snprintf(buffer, sizeof(buffer), "-%ld", (long) value);
		} else if (decimals == 1) {
			snprintf(buffer, sizeof(buffer), "-%ld.%01ld", (long) (value / 10),
					(long) (value % 10));
		} else if (decimals == 2) {
			snprintf(buffer, sizeof(buffer), "-%ld.%02ld", (long) (value / 100),
					(long) (value % 100));
		} else if (decimals == 3) {
			snprintf(buffer, sizeof(buffer), "-%ld.%03ld",
					(long) (value / 1000), (long) (value % 1000));
		}
	} else {
		if (decimals == 0) {
			snprintf(buffer, sizeof(buffer), "%ld", (long) value);
		} else if (decimals == 1) {
			snprintf(buffer, sizeof(buffer), "%ld.%01ld", (long) (value / 10),
					(long) (value % 10));
		} else if (decimals == 2) {
			snprintf(buffer, sizeof(buffer), "%ld.%02ld", (long) (value / 100),
					(long) (value % 100));
		} else if (decimals == 3) {
			snprintf(buffer, sizeof(buffer), "%ld.%03ld", (long) (value / 1000),
					(long) (value % 1000));
		}
	}

	// Определяем высоту области в зависимости от шрифта
	if (font_id == FONT_LARGE || font_id == FONTID_24F) {
		area_height = 24;
	} else if (font_id == FONT_MEDIUM || font_id == FONTID_16F) {
		area_height = 16;
	} else {
		area_height = 8; // FONT_SMALL
	}

	// Вычисляем ширину текста
	const char *ptr = buffer;
	while (*ptr) {
		char_table = font_GetFontStruct(font_id, (uint8_t) *ptr);
		if (char_table != NULL) {
			text_width += font_GetCharWidth(char_table) + 1;
		} else {
			text_width += 6;
		}
		ptr++;
	}
	if (text_width > 0)
		text_width--; // Убираем последний межсимвольный пробел

	// Вычисляем смещение для выравнивания по правому краю области
	if (text_width < area_width) {
		x_offset = area_width - text_width;
	}

	// 1. ЗАЛИВАЕМ ВСЮ ОБЛАСТЬ ФОНОМ (без мерцания между очисткой и рисованием)
	TFT_FillRect(x, y, area_width, area_height, bg_color);

	// 2. РИСУЕМ ТЕКСТ ВЫРОВНЕННЫМ ПО ПРАВОМУ КРАЮ ОБЛАСТИ
	TFT_DrawString(x + x_offset, y, buffer, font_id, color, TFT_NO_BG);
}

/**
 * @brief Горизонтальный индикатор ошибки (целочисленная версия)
 * @param x, y Координаты левого верхнего угла
 * @param width, height Размеры полосы
 * @param error_percent Ошибка в процентах (-100..+100) в десятых долях (например, -450 = -45.0%)
 * @param neutral_color Цвет нейтральной зоны (фон)
 * @param error_color Цвет индикатора ошибки
 */
void TFT_DrawErrorBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
		int16_t error_percent, uint16_t neutral_color, uint16_t error_color) {
	int16_t center_x;
	int16_t bar_width;
	int16_t bar_x;

	// Ограничиваем ошибку в диапазоне -1000..+1000 (-100.0%..+100.0%)
	if (error_percent < -1000)
		error_percent = -1000;
	if (error_percent > 1000)
		error_percent = 1000;

	// 1. Рисуем фон (нейтральную зону)
	TFT_FillRect(x, y, width, height, neutral_color);

	// 2. Вычисляем центр (нулевая точка)
	center_x = x + width / 2;

	// 3. Вычисляем ширину и положение индикатора ошибки
	// Используем целочисленное умножение для избежания float
	if (error_percent >= 0) {
		// Положительная ошибка - справа от центра
		bar_width = (width / 2) * error_percent / 1000;
		bar_x = center_x;
	} else {
		// Отрицательная ошибка - слева от центра
		bar_width = (width / 2) * (-error_percent) / 1000;
		bar_x = center_x - bar_width;
	}

	// 4. Рисуем индикатор ошибки (если есть что рисовать)
	if (bar_width > 0) {
		TFT_FillRect(bar_x, y, bar_width, height, error_color);
	}

	// 5. Рисуем вертикальную линию в центре для визуального разделения
	TFT_DrawFastVLine(center_x, y, height, TFT_WHITE);
}

/**
 * @brief Вертикальный прогресс-бар (целочисленная версия)
 * @param x, y Координаты левого верхнего угла
 * @param width, height Размеры прогресс-бара
 * @param fill_percent Заполнение в процентах (0..100) в десятых долях (например, 750 = 75.0%)
 * @param fill_color Цвет заполненной части
 * @param bg_color Цвет фона
 */
void TFT_DrawProgressBar(uint16_t x, uint16_t y, uint16_t width,
		uint16_t height, uint16_t fill_percent, uint16_t fill_color,
		uint16_t bg_color) {
	uint16_t fill_height;

	// Ограничиваем заполнение в диапазоне 0..1000 (0.0%..100.0%)
	if (fill_percent > 1000)
		fill_percent = 1000;

	// 1. Рисуем фон
	TFT_FillRect(x, y, width, height, bg_color);

	// 2. Вычисляем высоту заполненной части (целочисленное деление)
	fill_height = height * fill_percent / 1000;

	// 3. Рисуем заполненную часть (снизу вверх)
	if (fill_height > 0) {
		TFT_FillRect(x, y + height - fill_height, width, fill_height,
				fill_color);
	}

	// 4. Рисуем рамку для красоты
	TFT_DrawRect(x, y, width, height, TFT_WHITE);
}
//=========================================================================

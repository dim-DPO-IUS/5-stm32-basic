#ifndef TFT_ST7789_H
#define TFT_ST7789_H

#include "main.h"
#include "font.h"  // Основной заголовок шрифтов - он включает всё остальное

typedef struct {
	uint8_t width;      // Фиксированная ширина цифры (например, 10 пикселей)
	uint8_t height;     // Высота цифры (например, 16 пикселей)
	uint8_t spacing;    // Расстояние между цифрами (например, 2 пикселя)
} MonoFont;

// Простой моноширинный шрифт 10x16
extern const MonoFont font_mono_10x16;

// Новая функция для прототипа
void TFT_UpdateSingleNumber(uint16_t x, uint16_t y, int32_t new_value);

// ============================================================================
// КОНСТАНТЫ ДИСПЛЕЯ
// ============================================================================
#define TFT_WIDTH   240
#define TFT_HEIGHT  240

// ============================================================================
// ЦВЕТА (RGB565)
// ============================================================================
#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_YELLOW      0xFFE0
#define TFT_CYAN        0x07FF
#define TFT_MAGENTA     0xF81F
#define TFT_GRAY        0x8430
#define TFT_DARKGRAY    0x4208
#define TFT_LIGHTGRAY   0xC618
#define TFT_ORANGE      0xFC00
#define TFT_PURPLE      0x8010

// ============================================================================
// УДОБНЫЕ АЛИАСЫ ДЛЯ ШРИФТОВ
// ============================================================================
#define FONT_SMALL      FONTID_6X8M   // Основной шрифт 6x8 для меток
#define FONT_MEDIUM     FONTID_16F    // Средний пропорциональный 10x16
#define FONT_LARGE      FONTID_24F    // Крупные цифры (только 0-9)
#define FONT_XLARGE     FONTID_32F    // Очень крупные цифры (только 0-9)

// Для обратной совместимости
#define Font_7x10       FONTID_6X8M   // Ближайший аналог
#define Font_11x18      FONTID_16F    // Ближайший аналог
#define Font_16x26      FONTID_24F    // Ближайший аналог

// ============================================================================
// СПЕЦИАЛЬНЫЕ ЗНАЧЕНИЯ
// ============================================================================
#define TFT_NO_BG 0xFFFF  // Специальное значение для прозрачного фона

// ============================================================================
// БАЗОВЫЕ ФУНКЦИИ ДИСПЛЕЯ
// ============================================================================
void TFT_Init(SPI_HandleTypeDef *hspi);
void TFT_FillScreen(uint16_t color);

// Управление областью отрисовки
void TFT_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

// ============================================================================
// БАЗОВЫЕ ГРАФИЧЕСКИЕ ПРИМИТИВЫ
// ============================================================================
// устанавливает окно 1x1 пиксель и отправляет цвет
void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
// оптимизирована через буфер (быстрее чем рисовать по точкам)
void TFT_DrawFastHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
// пока через TFT_DrawPixel (можно оптимизировать позже)
void TFT_DrawFastVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void TFT_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color);
void TFT_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
		uint16_t color);

// ============================================================================
// РАБОТА С ТЕКСТОМ
// ============================================================================
/**
 * @brief Отрисовка одного символа
 * @param x, y Координаты левого верхнего угла символа
 * @param c Символ для отрисовки (ASCII код)
 * @param font_id ID шрифта из font.h (FONTID_6X8M, FONTID_16F, FONTID_24F, FONTID_32F)
 * @param color Цвет текста (RGB565)
 * @param bgcolor Цвет фона (RGB565). Если равен TFT_NO_BG, фон не рисуется (прозрачный)
 */
void TFT_DrawChar(uint16_t x, uint16_t y, char c, uint8_t font_id,
		uint16_t color, uint16_t bgcolor);

/**
 * @brief Отрисовка строки
 * @param x, y Координаты левого верхнего угла первого символа
 * @param str Строка для отрисовки (завершается нулем)
 * @param font_id ID шрифта из font.h
 * @param color Цвет текста (RGB565)
 * @param bgcolor Цвет фона (RGB565). Если равен TFT_NO_BG, фон не рисуется
 */
void TFT_DrawString(uint16_t x, uint16_t y, const char *str, uint8_t font_id,
		uint16_t color, uint16_t bgcolor);

/**
 * @brief Отрисовка центрированной строки
 * @param y Координата Y
 * @param str Строка для отрисовки
 * @param font_id ID шрифта
 * @param color Цвет текста
 * @param bgcolor Цвет фона
 */
void TFT_DrawStringCentered(uint16_t y, const char *str, uint8_t font_id,
		uint16_t color, uint16_t bgcolor);

void DrawMonoText2x(uint16_t x, uint16_t y, const char *text, uint16_t color,
		uint16_t bg_color);
// ============================================================================
// PID-СПЕЦИФИЧНЫЕ ФУНКЦИИ (ЦЕЛОЧИСЛЕННЫЕ ВЕРСИИ)
// ============================================================================

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
		uint8_t font_id, uint16_t color, uint16_t bgcolor);

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
		uint16_t area_width);

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
		uint16_t value_color);

/**
 * @brief Горизонтальный индикатор ошибки (целочисленная версия)
 * @param x, y Координаты левого верхнего угла
 * @param width, height Размеры полосы
 * @param error_percent Ошибка в процентах (-1000..+1000) где 1000 = 100.0%
 * @param neutral_color Цвет нейтральной зоны (фон)
 * @param error_color Цвет индикатора ошибки
 */
void TFT_DrawErrorBar(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
		int16_t error_percent, uint16_t neutral_color, uint16_t error_color);

/**
 * @brief Вертикальный прогресс-бар (целочисленная версия)
 * @param x, y Координаты левого верхнего угла
 * @param width, height Размеры прогресс-бара
 * @param fill_percent Заполнение в процентах (0..1000) где 1000 = 100.0%
 * @param fill_color Цвет заполненной части
 * @param bg_color Цвет фона
 */
void TFT_DrawProgressBar(uint16_t x, uint16_t y, uint16_t width,
		uint16_t height, uint16_t fill_percent, uint16_t fill_color,
		uint16_t bg_color);

/**
 * @brief Горизонтальный прогресс-бар (заполняется слева направо)
 * @param x, y Координаты левого верхнего угла
 * @param width, height Размеры прогресс-бара
 * @param fill_percent Заполнение в процентах (0..1000) где 1000 = 100.0%
 * @param fill_color Цвет заполненной части
 * @param bg_color Цвет фона
 */
void TFT_DrawHProgressBar(uint16_t x, uint16_t y, uint16_t width,
		uint16_t height, uint16_t fill_percent, uint16_t fill_color,
		uint16_t bg_color);
// ============================================================================
// ВНУТРЕННИЕ ФУНКЦИИ (не для использования вне библиотеки)
// ============================================================================
static void TFT_WriteCommand(uint8_t cmd);
static void TFT_WriteData(uint8_t data);
static void TFT_WriteDataBurst(uint8_t *data, uint32_t size);

#endif // TFT_ST7789_H

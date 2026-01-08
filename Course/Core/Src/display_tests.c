/*
 * display_tests.c
 *
 *  Created on: Jan 7, 2026
 *      Author: dim0k
 */

#include <stdio.h>
#include <string.h>
#include "display_tests.h"
// ============================================================================
//
// ============================================================================
// Объявляем функции из main.c как extern
extern void MX_SPI2_Init(void);
// ============================================================================
//
// ============================================================================
// Полная инициализация ST7789
void Full_ST7789_UART_Init(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	// 1. Аппаратный сброс
	HAL_UART_Transmit(huart, (uint8_t*) "1. Hardware Reset\r\n", 19, 100);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// 2. Software Reset
	HAL_UART_Transmit(huart, (uint8_t*) "2. Software Reset\r\n", 19, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x01; // SWRESET
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(150);

	// 3. Sleep Out
	HAL_UART_Transmit(huart, (uint8_t*) "3. Sleep Out\r\n", 14, 100);
	cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(255);

	// 4. Color Mode (16-bit RGB565)
	HAL_UART_Transmit(huart, (uint8_t*) "4. Color Mode 16-bit\r\n", 22, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55; // 16-bit RGB565
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 5. Memory Access Control (ориентация)
	HAL_UART_Transmit(huart, (uint8_t*) "5. Memory Access\r\n", 18, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x36; // MADCTL
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t madctl = 0x00; // RGB order, normal rotation
	HAL_SPI_Transmit(hspi, &madctl, 1, 100);
	HAL_Delay(10);

	// 6. Display Inversion On (часто требуется)
	HAL_UART_Transmit(huart, (uint8_t*) "6. Inversion On\r\n", 17, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x21; // INVON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 7. Normal Display Mode
	HAL_UART_Transmit(huart, (uint8_t*) "7. Normal Mode\r\n", 16, 100);
	cmd = 0x13; // NORON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 8. Display On
	HAL_UART_Transmit(huart, (uint8_t*) "8. Display On\r\n", 15, 100);
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);

	HAL_UART_Transmit(huart, (uint8_t*) "Init Complete\r\n", 15, 100);
}
// ============================================================================
//
// ============================================================================
void Full_ST7789_Init(SPI_HandleTypeDef *hspi) {
	// 1. Аппаратный сброс
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// 2. Software Reset
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x01; // SWRESET
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(150);

	// 3. Sleep Out
	cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(255);

	// 4. Color Mode (16-bit RGB565)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55; // 16-bit RGB565
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 5. Memory Access Control (ориентация)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x36; // MADCTL
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t madctl = 0x00; // RGB order, normal rotation
	HAL_SPI_Transmit(hspi, &madctl, 1, 100);

	// 6. Display Inversion On
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x21; // INVON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 7. Normal Display Mode
	cmd = 0x13; // NORON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 8. Display On
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);
}
// ============================================================================
//
// ============================================================================
void Minimal_ST7789_Init(SPI_HandleTypeDef *hspi) {
	// Только самое необходимое:

	// 1. Reset
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// 2. Sleep Out
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(120);

	// 3. Color Mode 16-bit
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55;
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 4. Display On
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);
}
// ============================================================================
//
// ============================================================================
void Fixed_ST7789_Init(SPI_HandleTypeDef *hspi) {
	// 1. Reset
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// 2. Sleep Out
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(120);

	// 3. Color Mode 16-bit
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55;
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 4. Display Inversion On (ДОБАВИТЬ!)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x21; // INVON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 5. Display On
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);
}
// ============================================================================
//
// ============================================================================
void Optimal_ST7789_Init(SPI_HandleTypeDef *hspi) {
	// 1. Аппаратный сброс
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// 2. Software Reset (гарантирует начальное состояние)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x01; // SWRESET
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(150);

	// 3. Sleep Out
	cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(255);

	// 4. Color Mode 16-bit
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55;
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 5. Memory Access Control (ОБЯЗАТЕЛЬНО)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x36; // MADCTL
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t madctl = 0x00; // Для ROTATION=2
	HAL_SPI_Transmit(hspi, &madctl, 1, 100);

	// 6. Display Inversion On (для большинства дисплеев)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x21; // INVON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 7. Normal Display Mode
	cmd = 0x13; // NORON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 8. Display On
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);
}
// ============================================================================
//
// ============================================================================
void Stable_ST7789_Init(SPI_HandleTypeDef *hspi) {
	// 0. Убедимся что питание стабильно
	HAL_Delay(50);

	// 1. Аппаратный сброс (удлиненный)
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);  // Увеличить задержку
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(150); // Увеличить до 150ms

	// 2. Software Reset (ОБЯЗАТЕЛЬНО перед другими командами)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x01; // SWRESET
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(150); // 150ms после SWRESET - КРИТИЧНО!

	// 3. Sleep Out
	cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(255); // 255ms после SLPOUT - КРИТИЧНО!

	// 4. Color Mode
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55;
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 5. MADCTL
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x36; // MADCTL
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t madctl = 0x00;
	HAL_SPI_Transmit(hspi, &madctl, 1, 100);

	// 6. INVON
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x21; // INVON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 7. NORON
	cmd = 0x13; // NORON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 8. DISPON
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(120); // Увеличить после DISPON

	// 9. Явная очистка экрана
	Simple_Fill_Color(hspi, BLACK);
}
// ============================================================================
//
// ============================================================================
void TestA_ST7789_Init(SPI_HandleTypeDef *hspi) {
	// 1. Аппаратный сброс (как в Fixed)
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// 2. Software Reset (ДОБАВЛЯЕМ - возможно проблема здесь)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x01; // SWRESET
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(150); // Обязательно 150ms после SWRESET

	// 3. Sleep Out
	cmd = 0x11; // SLPOUT
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(255);

	// 4. Color Mode 16-bit
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x3A; // COLMOD
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55;
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);
	HAL_Delay(10);

	// 5. Display Inversion On (есть в Fixed)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x21; // INVON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(10);

	// 6. Display On
	cmd = 0x29; // DISPON
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);
}
// ============================================================================
//
// ============================================================================
void Force_Clear_Display(SPI_HandleTypeDef *hspi) {
	// Сразу после питания, ДО инициализации
	// 1. Быстрый аппаратный сброс
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

	// 2. Минимальная команда для записи черного
	uint8_t cmd;

	// CASET
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2A;
	HAL_SPI_Transmit(hspi, &cmd, 1, 1);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t window[] = { 0x00, 0x00, 0x00, 0x01 }; // Только 2 колонки
	HAL_SPI_Transmit(hspi, window, 4, 1);

	// RAMWR + 2 черных пикселя
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2C;
	HAL_SPI_Transmit(hspi, &cmd, 1, 1);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);

	uint8_t black[] = { 0x00, 0x00 };
	HAL_SPI_Transmit(hspi, black, 2, 1); // 1 пиксель
}
// ============================================================================
//
// ============================================================================
// Упрощённая функция заливки экрана
void Simple_Fill_Color(SPI_HandleTypeDef *hspi, uint16_t color) {
	uint8_t hi = color >> 8;    // Старший байт цвета
	uint8_t lo = color & 0xFF;  // Младший байт цвета

	// 1. Установка области заливки (весь экран 240x240)
	// Команда CASET (0x2A) - установка колонок
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET); // DC = 0 (команда)
	uint8_t cmd = 0x2A;
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET); // DC = 1 (данные)
	uint8_t caset_data[] = { 0x00, 0x00, 0x00, 0xEF }; // 0-239 (240 колонок)
	HAL_SPI_Transmit(hspi, caset_data, 4, 100);

	// 2. Команда RASET (0x2B) - установка строк
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET); // DC = 0
	cmd = 0x2B;
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET); // DC = 1
	uint8_t raset_data[] = { 0x00, 0x00, 0x00, 0xEF }; // 0-239 (240 строк)
	HAL_SPI_Transmit(hspi, raset_data, 4, 100);

	// 3. Команда RAMWR (0x2C) - запись в память
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET); // DC = 0
	cmd = 0x2C;
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);

	// 4. Отправка цвета для всех пикселей
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET); // DC = 1
	for (uint32_t i = 0; i < (240 * 240); i++) {
		HAL_SPI_Transmit(hspi, &hi, 1, 100);
		HAL_SPI_Transmit(hspi, &lo, 1, 100);
	}
}
// ============================================================================
//
// ============================================================================
// Функция мониторинга SPI
void Monitor_SPI_Data(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char spi_msg[50];

	// Отправляем тестовые данные 0xAA
	uint8_t test_data = 0xAA;
	HAL_SPI_Transmit(hspi, &test_data, 1, 100);

	// Логируем
	sprintf(spi_msg, "SPI Sent: 0x%02X\r\n", test_data);
	HAL_UART_Transmit(huart, (uint8_t*) spi_msg, strlen(spi_msg), 100);
	HAL_Delay(10);
}
// ============================================================================
//
// ============================================================================
void Diagnostic_Fill_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi,
		volatile uint8_t *spi_error_flag) {
	char msg[100];

	// Тест 1: Залить 10x10 пикселей в центре
	sprintf(msg, "Test 1: 10x10 pixels at center\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Установим маленькое окно
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x2A; // CASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t caset_data[] = { 0x00, 0x77, 0x00, 0x80 }; // 119-128 (10 пикселей)
	HAL_SPI_Transmit(hspi, caset_data, 4, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2B; // RASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t raset_data[] = { 0x00, 0x77, 0x00, 0x80 }; // 119-128
	HAL_SPI_Transmit(hspi, raset_data, 4, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2C; // RAMWR
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	// Залить красным цветом
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t red_color[] = { 0xF8, 0x00 }; // RGB565 RED

	uint16_t pixel_count = 0;
	for (int i = 0; i < 100; i++) { // 10x10 = 100 пикселей
		HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, red_color, 2, 50);
		if (status != HAL_OK) {
			sprintf(msg, "SPI Error at pixel %d\r\n", i);
			HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
			*spi_error_flag = 1;
			return;
		}
		pixel_count++;
		// Мигаем светодиодом каждые 10 пикселей
		if (pixel_count % 10 == 0) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}

	sprintf(msg, "Test 1: Sent %d pixels\r\n", pixel_count);
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Тест 2: Нарисовать одну точку
	sprintf(msg, "Test 2: Single pixel at (120,120)\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Используем библиотечную функцию для одного пикселя
	ST7789_DrawPixel(120, 120, GREEN);

	sprintf(msg, "Test 2 complete\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	HAL_Delay(1000);
}
// ============================================================================
//
// ============================================================================
void Optimized_Fill_Color(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi,
		uint16_t color) {
	char msg[50];
	sprintf(msg, "Optimized fill started\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// 1. Установить окно на весь экран
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x2A; // CASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t caset_data[] = { 0x00, 0x00, 0x00, 0xEF }; // 0-239
	HAL_SPI_Transmit(hspi, caset_data, 4, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2B; // RASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t raset_data[] = { 0x00, 0x50, 0x01, 0x3F }; // 80-319
	HAL_SPI_Transmit(hspi, raset_data, 4, 10);

	// 2. Команда записи в RAM
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2C; // RAMWR
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	// 3. Отправка данных блоками по 512 байт
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);

	uint8_t color_hi = color >> 8;
	uint8_t color_lo = color & 0xFF;

#define BUFFER_SIZE 512
	uint8_t pixel_buffer[BUFFER_SIZE];

	// Заполняем буфер нашим цветом
	for (int i = 0; i < BUFFER_SIZE; i += 2) {
		pixel_buffer[i] = color_hi;
		pixel_buffer[i + 1] = color_lo;
	}

	uint32_t total_pixels = 240 * 240;
	uint32_t pixels_sent = 0;
	uint32_t iteration = 0;

	while (pixels_sent < total_pixels) {
		// Отправляем буфер
		HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, pixel_buffer,
		BUFFER_SIZE, 1000);

		if (status != HAL_OK) {
			sprintf(msg, "SPI Error at iteration %lu\r\n", iteration);
			HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
			return;
		}

		pixels_sent += (BUFFER_SIZE / 2);
		iteration++;

		// Мигаем светодиодом каждые 10 итераций
		if (iteration % 10 == 0) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}

	sprintf(msg, "Fill complete: %lu pixels sent in %lu iterations\r\n",
			pixels_sent, iteration);
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Test_Different_Configs(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];

	// Тест 1: БЕЗ смещения Y_SHIFT
	sprintf(msg, "\r\n=== TEST 1: No Y shift ===\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x2A; // CASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t caset_data[] = { 0x00, 0x00, 0x00, 0xEF }; // 0-239
	HAL_SPI_Transmit(hspi, caset_data, 4, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2B; // RASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t raset_data[] = { 0x00, 0x00, 0x00, 0xEF }; // 0-239
	HAL_SPI_Transmit(hspi, raset_data, 4, 10);

	// Залить красным
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2C; // RAMWR
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t red[] = { 0xF8, 0x00 }; // RGB565 RED
	for (int i = 0; i < 1000; i++) {
		HAL_SPI_Transmit(hspi, red, 2, 10);
	}

	sprintf(msg, "Test 1 done (Y=0-239). Check display!\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
	HAL_Delay(3000);
}
// ============================================================================
//
// ============================================================================
void Hardware_Verification_Test(UART_HandleTypeDef *huart,
		SPI_HandleTypeDef *hspi) {
	char msg[100];

	sprintf(msg, "\r\n=== HARDWARE VERIFICATION TEST ===\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	HAL_Delay(500);

	sprintf(msg, "Hardware test complete!\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Ultimate_Hardware_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];

	sprintf(msg, "\r\n=== ULTIMATE HARDWARE TEST ===\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Тест 1: Проверка DC пина
	sprintf(msg, "Test 1: DC pin toggle\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	for (int i = 0; i < 10; i++) {
		HAL_GPIO_TogglePin(TFT_DC_GPIO_Port, TFT_DC_Pin);
		HAL_Delay(100);
	}
	// Оставить DC=0 (команда)
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);

	HAL_Delay(500);

	sprintf(msg, "Hardware test complete!\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Physical_MOSI_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];

	sprintf(msg, "\r\n=== PHYSICAL MOSI TEST ===\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// 1. Настроим PC3 (MOSI) как обычный GPIO OUTPUT
	sprintf(msg, "Configuring PC3 as GPIO Output...\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Временно отключаем SPI
	HAL_SPI_DeInit(hspi);

	// Настраиваем PC3 как вывод
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// 2. Мигаем PC3 вручную
	sprintf(msg, "Manually toggling PC3 (MOSI)...\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	for (int i = 0; i < 20; i++) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

	// 3. Восстанавливаем SPI
	sprintf(msg, "Restoring SPI configuration...\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	MX_SPI2_Init();

	sprintf(msg, "MOSI test complete.\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Test_Alternative_MOSI_Pin(UART_HandleTypeDef *huart,
		SPI_HandleTypeDef *hspi) {
	char msg[100];

	sprintf(msg, "\r\n=== ALTERNATIVE MOSI PIN TEST ===\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Используем PA7 как альтернативный MOSI
	HAL_SPI_DeInit(hspi);

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// PA7 как SPI2_MOSI
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	MX_SPI2_Init();

	sprintf(msg, "Now using PA7 as MOSI. Testing display...\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Быстрый тест дисплея
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x2C; // RAMWR
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t red[] = { 0xF8, 0x00 };

	for (int i = 0; i < 100; i++) {
		HAL_SPI_Transmit(hspi, red, 2, 10);
		if (i % 10 == 0)
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

	sprintf(msg, "Alternative MOSI test complete.\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Quick_SPI_Diagnostic(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];

	// 1. Увеличиваем скорость SPI в runtime
	HAL_SPI_DeInit(hspi);
	hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 4.5 МГц
	HAL_SPI_Init(hspi);

	sprintf(msg, "SPI speed: 4.5 MHz\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// 2. Минимальная инициализация
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

	// SLPOUT
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x11;
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(120);

	// COLMOD - 16-bit
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x3A;
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t colmod = 0x55;
	HAL_SPI_Transmit(hspi, &colmod, 1, 100);

	// DISPON
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x29;
	HAL_SPI_Transmit(hspi, &cmd, 1, 100);
	HAL_Delay(100);

	// 3. Тест с разными вариантами данных
	sprintf(msg, "Testing different data formats...\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	HAL_Delay(500);

	sprintf(msg, "SPI diagnostic complete.\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Test_Full_Screen(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];

	sprintf(msg, "\r\n=== TEST FULL SCREEN ===\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Тест 1: Y=80-319 (со смещением 80)
	sprintf(msg, "Test 1: Y=80-319 (with 80 offset)\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Установить окно на весь экран со смещением
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	uint8_t cmd = 0x2A; // CASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t caset[] = { 0x00, 0x00, 0x00, 0xEF }; // X: 0-239
	HAL_SPI_Transmit(hspi, caset, 4, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2B; // RASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t raset_80[] = { 0x00, 0x50, 0x01, 0x3F }; // Y: 80-319
	HAL_SPI_Transmit(hspi, raset_80, 4, 10);

	// Залить красным
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2C; // RAMWR
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
//	uint8_t red[] = { 0xF8, 0x00 };
	uint8_t red[] = { 0x00, 0xF8 };    // SWAPPED!

	// Отправить 2000 пикселей красного (примерно 1/30 экрана)
	for (int i = 0; i < 2000; i++) {
		HAL_SPI_Transmit(hspi, red, 2, 10);
		if (i % 200 == 0)
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

	sprintf(msg, "Test 1 complete. Check if RED fills screen.\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
	HAL_Delay(3000);

	// Тест 2: Y=0-239 (без смещения)
	sprintf(msg, "Test 2: Y=0-239 (no offset)\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);

	// Установить окно Y=0-239
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2B; // RASET
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	uint8_t raset_0[] = { 0x00, 0x00, 0x00, 0xEF }; // Y: 0-239
	HAL_SPI_Transmit(hspi, raset_0, 4, 10);

	// Залить зеленым
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
	cmd = 0x2C; // RAMWR
	HAL_SPI_Transmit(hspi, &cmd, 1, 10);

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
//	uint8_t green[] = { 0x07, 0xE0 };
	uint8_t green[] = { 0xE0, 0x07 };  // SWAPPED!

	for (int i = 0; i < 2000; i++) {
		HAL_SPI_Transmit(hspi, green, 2, 10);
		if (i % 200 == 0)
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}

	sprintf(msg, "Test 2 complete. Check if GREEN fills screen.\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
	HAL_Delay(3000);

	sprintf(msg, "Full screen test complete.\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
// Остальные функции оставляем как заглушки
void Debug_DC_Signal(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];
	sprintf(msg, "Debug DC Signal - not implemented\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Test_SPI_Modes(UART_HandleTypeDef *huart) {
	char msg[100];
	sprintf(msg, "Test SPI Modes - not implemented\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void Test_With_CS_Control(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];
	sprintf(msg, "Test With CS Control - not implemented\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void CS_Simulation_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];
	sprintf(msg, "CS Simulation Test - not implemented\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void ST7789_Simple_Test(UART_HandleTypeDef *huart, SPI_HandleTypeDef *hspi) {
	char msg[100];
	sprintf(msg, "ST7789 Simple Test - not implemented\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
void ST7789_Arduino_Style_Test(UART_HandleTypeDef *huart,
		SPI_HandleTypeDef *hspi) {
	char msg[100];
	sprintf(msg, "ST7789 Arduino Style Test - not implemented\r\n");
	HAL_UART_Transmit(huart, (uint8_t*) msg, strlen(msg), 100);
}
// ============================================================================
//
// ============================================================================
// ============================================================================
//
// ============================================================================
// ============================================================================
//
// ============================================================================
// ============================================================================
//
// ============================================================================
// ============================================================================
//
// ============================================================================
// ============================================================================
//
// ============================================================================

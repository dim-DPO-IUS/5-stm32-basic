/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "display_tests.h"
#include "tft_st7789.h"
//#include "tft_pid.h"
// ============================================================================
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ============================================================================
volatile uint8_t adc_data_ready = 0;
volatile uint8_t new_setpoint_received = 0;
volatile uint8_t display_update_needed = 0;

// Целочисленный ПИД (фиксированная точка)
#define PID_SCALE 1000  // Масштабирующий коэффициент

// ПИД переменные
int32_t setpoint = 0;           // Уставка 0-4095
int32_t feedback = 0;           // Обратная связь 0-4095
int32_t output = 0;             // Выход 0-1000
int32_t error = 0;
int32_t integral = 0;
int32_t derivative = 0;
int32_t last_error = 0;

// Целочисленные коэффициенты (умноженные на PID_SCALE)
#define KP (1000)   // пропорциональный
#define KI (80)     // интегральный
#define KD (300)    // дифференциальный

// АЦП переменные
uint16_t adc_raw = 0;
uint16_t adc_mv = 0;

// UART переменные
uint8_t rx_data;
char buffer[10];
uint8_t inx = 0;
// ============================================================================
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ============================================================================
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
// ФУНКЦИИ ПРЕОБРАЗОВАНИЯ (фиксированная точка)
// ADC значение (0-4095) → милливольты (0-3300)
static inline int32_t adc_to_millivolts(uint16_t adc_val) {
	// adc_val * 3300 / 4095
	// Упрощаем: 3300/4095 ≈ 16125/20000 (для точности в целых числах)
	return ((int32_t) adc_val * 3300L) / 4095L;
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
// Уставка (0-1023) → милливольты (0-3300)
static inline int32_t setpoint_to_millivolts(uint16_t setpoint_1023) { // <-- uint16_t!
	// setpoint * 3300 / 1023
	return ((int32_t) setpoint_1023 * 3300L) / 1023L;
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
// Милливольты → значение для Plotter (0-1023 для обратной совместимости)
static inline uint16_t millivolts_to_plotter(int32_t mv) {
	// mv * 1023 / 3300
	return (uint16_t) ((mv * 1023L) / 3300L);
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
// Простой Целочисленный ПИД регулятор
int32_t pid_update_mv(int32_t sp_mv, int32_t fb_mv) {
	error = sp_mv - fb_mv;

	integral += error;
	if (integral > 5000 * PID_SCALE)
		integral = 5000 * PID_SCALE;
	if (integral < -5000 * PID_SCALE)
		integral = -5000 * PID_SCALE;

	derivative = error - last_error;
	last_error = error;

	// Вычисление с масштабированием (коэффициенты остаются те же!)
	output = (KP * error + KI * integral + KD * derivative) / PID_SCALE;

	// Ограничение выхода (ШИМ 0-1000)
	if (output > 1000)
		output = 1000;
	if (output < 0)
		output = 0;

	return output;
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// 1. Получаем значение АЦП
	adc_raw = HAL_ADC_GetValue(&hadc1);

	// 2. Конвертируем в милливольты
	int32_t feedback_mv = adc_to_millivolts(adc_raw);
	feedback = feedback_mv;  // Сохраняем для отладки (в милливольтах!)
	adc_mv = (uint16_t) feedback_mv;

	// 3. Уставка тоже должна быть в милливольтах
	// (setpoint уже в милливольтах после преобразования в UART обработчике)

	// 4. Вычисляем ПИД с милливольтами
	int32_t pid_output = pid_update_mv(setpoint, feedback_mv);

	// 5. Обновляем ШИМ
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t )pid_output);

	// 6. Флаг для отладки
	adc_data_ready = 1;
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
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
				setpoint = setpoint_to_millivolts((uint16_t) value); // <-- явное преобразование
				new_setpoint_received = 1;
				inx = 0;
			}
		} else if (rx_data >= '0' && rx_data <= '9') {
			if (inx < sizeof(buffer) - 1) {
				buffer[inx++] = rx_data;
			}
		}
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
// Функция для рисования PID значения в формате X.XXX увеличенным в 2 раза
void DrawPIDValue2x(uint16_t x, uint16_t y, int32_t value, uint16_t color) {
	char buffer[10];

	// Форматируем всегда 6 символов: [знак][цифра].[3цифры]
	// Примеры: " 1.650", "-0.027"
	if (value < 0) {
		sprintf(buffer, "-%01d.%03d", (-value) / 1000, (-value) % 1000);
	} else {
		sprintf(buffer, " %01d.%03d", value / 1000, value % 1000);
	}

	// Рисуем увеличенным в 2 раза
	DrawMonoText2x(x, y, buffer, color, TFT_BLACK);
}

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
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

	// 5. Коэффициенты PID
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
	TFT_BLACK); // Было 235
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
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
	if (abs(feedback - last_large_feedback) > 5) { // Изменение > 0.005V
		TFT_DrawFixedPointEx(LARGE_VALUE_X, 220, feedback, 3, FONT_LARGE,
		TFT_WHITE, TFT_BLACK, LARGE_WIDTH);
		last_large_feedback = feedback;
	}
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
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

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
void UpdatePIDDisplayMono2x(int32_t setpoint, int32_t feedback, int32_t error,
		int16_t error_percent, uint16_t output_percent) {
	// 1. Основные значения
	DrawPIDValue2x(85, 35, setpoint, TFT_GREEN);
	DrawPIDValue2x(85, 60, feedback, TFT_CYAN);
	DrawPIDValue2x(85, 85, error, TFT_RED);

	// 2. Output percent
	char output_buffer[10];
	sprintf(output_buffer, "%d%%", output_percent / 10);
	DrawMonoText2x(200, 180, output_buffer, TFT_WHITE, TFT_BLACK); //

	// 3. Индикатор ошибки
	TFT_DrawErrorBar(15, 125, TFT_WIDTH - 30, 12, error_percent, TFT_DARKGRAY,
	TFT_RED);

	// 4. Прогресс-бар
	TFT_DrawHProgressBar(15, 195, TFT_WIDTH - 30, 15, output_percent, TFT_BLUE,
	TFT_DARKGRAY);

	// 5. Крупное значение
	static int32_t last_large_feedback = -1;
	if (abs(feedback - last_large_feedback) > 5) {
		char large_buffer[10];
		if (feedback < 0) {
			sprintf(large_buffer, "-%01d.%03d", (-feedback) / 1000,
					(-feedback) % 1000);
		} else {
			sprintf(large_buffer, " %01d.%03d", feedback / 1000,
					feedback % 1000);
		}
		// Рисуем увеличенным в 2 раза
		DrawMonoText2x(TFT_WIDTH / 2 - 40, 216, large_buffer, TFT_CYAN,
		TFT_BLACK); // Было 220
		last_large_feedback = feedback;
	}
}

//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
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
	// ... остальной код DisplayPIDInterface
}
//------------------------------------------------------------------------
//
//------------------------------------------------------------------------
// Обработчик прерывания таймера 4 (100 Гц)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM4) {
		// Запускаем АЦП преобразование
		HAL_ADC_Start_IT(&hadc1);
	}

	else if (htim->Instance == TIM1) {
//		// Обновление дисплея
//		// Вычисляем error_percent
//		int16_t error_percent = 0;
//		if (setpoint != 0) {
//			error_percent = (error * 1000) / (setpoint / 2);
//			if (error_percent > 1000)
//				error_percent = 1000;
//			if (error_percent < -1000)
//				error_percent = -1000;
//		}
//
//		// Используем output напрямую (уже 0-1000)
//		UpdatePIDDisplay(setpoint, feedback, error, error_percent, output);

//		static int counter = 0;
//		counter++;
//		char temp[20];
//		sprintf(temp, "%d", counter);
//		TFT_DrawString(50, 50, temp, FONT_MEDIUM, TFT_WHITE, TFT_BLACK);
	}
}
// ============================================================================
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_SPI2_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */
	// ========================================================================
	// Приветствие в USB
	HAL_UART_Transmit(&huart2, (uint8_t*) "Ready\r\n", 7, 100);
	// Включаем прием по прерыванию
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	// Запускаем  PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	// Запуск Таймера 4 с прерываниями - Запускаем АЦП преобразование
	HAL_TIM_Base_Start_IT(&htim4);
	//************************************************************************
	// Запуск Таймера 10 с прерываниями - Обновление дисплея
	HAL_TIM_Base_Start_IT(&htim10);
//	// Явно разрешить прерывание по обновлению
//	__HAL_TIM_ENABLE_IT(&htim10, TIM_IT_UPDATE);
//
//	// Проверим регистр прерывания
//	if (TIM10->DIER & TIM_DIER_UIE) {
//		HAL_UART_Transmit(&huart2, (uint8_t*) "UIE_ON\r\n", 8, 100);
//	}

	//************************************************************************
	// Инициализация ПИД
	setpoint = 1650;  // 1650 милливольт = 1.65 Вольт
	output = 500;     // Начальный выход 50%

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------------------------------------------
	TFT_Init(&hspi2);
	DisplayPIDInterface(); // Или DisplayPIDInterfaceMono2x
//	TFT_FillScreen(TFT_BLACK); // Черный фон
//	TFT_DrawString(10, 10, "TEST", FONT_SMALL, TFT_WHITE, TFT_BLACK);
//	TFT_FillScreen(TFT_BLACK);

//	TFT_FillScreen(TFT_BLACK);

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	// Тест 1: Статичный текст (без обновления)
//	TFT_DrawString(50, 50, "STATIC", FONT_MEDIUM, TFT_WHITE, TFT_BLACK);
//	HAL_Delay(3000);
//
//	// Тест 2: Динамический текст в цикле (без прерываний)
//	for (int i = 0; i < 1000; i++) {
//
////		TFT_FillRect(50, 100, 100, 30, TFT_BLACK);
//
//		char temp[20];
//		sprintf(temp, "VAL:%d", i);
//		TFT_DrawString(50, 100, temp, FONT_LARGE, TFT_WHITE, TFT_BLACK);
//		HAL_Delay(100); // 10 Гц
//	}
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// ========================================================================
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	static uint32_t last_display = 0;

	while (1) {
		// ====================================================================
		if (HAL_GetTick() - last_display >= 100) { // 10 Гц
			last_display = HAL_GetTick();

			// Вычисляем error_percent
			int16_t error_percent = 0;
			if (setpoint != 0) {
				error_percent = (error * 1000) / (setpoint / 2);
				if (error_percent > 1000)
					error_percent = 1000;
				if (error_percent < -1000)
					error_percent = -1000;
			}

			// Обновляем дисплей новой функцией
			UpdatePIDDisplayMono2x(setpoint, feedback, error, error_percent,
					output);
		}
// ----------------------------------------------------------------------------
//		if (HAL_GetTick() - last_update >= 100) { // 10 Гц (каждые 100 мс)
//			last_update = HAL_GetTick();
//
//			counter++;
//			if (counter > 9999)
//				counter = 0;
//
//			// 1. Рисуем заголовок обычным шрифтом
//			TFT_DrawString(50, 10, "MONO 2X TEST", FONT_SMALL, TFT_YELLOW,
//			TFT_BLACK);
//
//			// 2. Форматируем счетчик как PID значение: X.XXX
//			char buffer[10];
//			int32_t fake_value = counter; // 0-9999
//
//			if (fake_value < 0) {
//				sprintf(buffer, "-%01d.%03d", (-fake_value) / 1000,
//						(-fake_value) % 1000);
//			} else {
//				sprintf(buffer, " %01d.%03d", fake_value / 1000,
//						fake_value % 1000);
//			}
//
//			// 3. Рисуем увеличенным в 2 раза
//			DrawMonoText2x(50, 50, buffer, TFT_GREEN, TFT_BLACK);
//
//			// 4. Для сравнения - обычный размер
//			TFT_DrawString(50, 80, "Normal:", FONT_SMALL, TFT_WHITE, TFT_BLACK);
//			TFT_DrawString(100, 80, buffer, FONT_SMALL, TFT_WHITE, TFT_BLACK);
//
//			// 5. Выводим в UART для отладки
//			char msg[50];
//			sprintf(msg, "Counter: %d -> %s\r\n", counter, buffer);
//			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
//		}
//
// ----------------------------------------------------------------------------
//		static uint32_t last_display = 0;
//		if (HAL_GetTick() - last_display >= 100) { // 10 Гц
//			last_display = HAL_GetTick();
//
//			// Вычислите error_percent здесь
//			int16_t error_percent = 0;
//			if (setpoint != 0) {
//				error_percent = (error * 1000) / (setpoint / 2);
//				if (error_percent > 1000)
//					error_percent = 1000;
//				if (error_percent < -1000)
//					error_percent = -1000;
//			}
//
//			// Вызовите упрощенную функцию
//			UpdatePIDDisplaySimple(setpoint, feedback, error, error_percent,
//					output);
//		}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		static uint32_t last_display = 0;
//		if (HAL_GetTick() - last_display >= 100) { // 10 Гц
//			last_display = HAL_GetTick();
//
//			static int i = 0;
//			i++;
//			char temp[20];
//			sprintf(temp, "VAL:%d", i);
//			TFT_DrawString(50, 100, temp, FONT_LARGE, TFT_WHITE, TFT_BLACK);
//			if (i > 1000)
//				i = 0;
//		}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//		if (display_update_needed) {
//			display_update_needed = 0;
//
//			// ТЕСТ ПРЯМОУГОЛЬНИКА
//			static uint8_t toggle = 0;
//			if (toggle == 0) {
//				TFT_FillRect(50, 50, 100, 100, TFT_RED);
////				TFT_FillScreen(TFT_RED);
//				toggle = 1;
//			} else {
//				TFT_FillRect(50, 50, 100, 100, TFT_BLUE);
////				TFT_FillScreen(TFT_BLUE);
//				toggle = 0;
//			}
//		}
// ----------------------------------------------------------------------------

		// Отправка данных для Plotter (каждые 50 мс)
		static uint32_t last_plot_time = 0;
		if (HAL_GetTick() - last_plot_time >= 50) {
			char plot_data[50];
			uint16_t sp_plot = millivolts_to_plotter(setpoint);
			uint16_t fb_plot = millivolts_to_plotter(feedback);
			uint16_t out_plot = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1)
					* 1023 / 1000;

			sprintf(plot_data, "%u,%u,%u\r\n", sp_plot, fb_plot, out_plot);
			HAL_UART_Transmit(&huart2, (uint8_t*) plot_data, strlen(plot_data),
					100);
			last_plot_time = HAL_GetTick();
		}

//		 Индикация работы светодиодом (каждые 500 мс)
		static uint32_t last_led_time = 0;
		if (HAL_GetTick() - last_led_time >= 500) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			last_led_time = HAL_GetTick();
		}
		// ====================================================================
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_1LINE;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 719 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 7200 - 1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 250 - 1;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, TFT_RESET_Pin | TFT_DC_Pin | LD2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TFT_RESET_Pin TFT_DC_Pin LD2_Pin */
	GPIO_InitStruct.Pin = TFT_RESET_Pin | TFT_DC_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
//	adc_data_ready = 1;
//}
//
//// Обработчик прерывания UART
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance == USART1) {
//		// Отправляем "PONG" на любой символ
//		HAL_UART_Transmit(&huart1, (uint8_t*) "PONG\r\n", 6, 100);
//
//		// Снова включаем прием
//		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//	}
//}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

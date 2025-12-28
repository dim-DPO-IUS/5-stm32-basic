#include "csv_logger.h"
#include <stdio.h>
#include <string.h>

// ДОБАВИТЬ ЭТИ ЗАГОЛОВКИ:
#include "main.h"           // для HAL_GetTick()
#include "fatfs.h"         // для FIL, FRESULT, FA_*, FR_*
#include "ff.h"            // для функций FATFS (f_open, f_write и т.д.)

// Определяем CSV_FILENAME если не определён
#ifndef CSV_FILENAME
#define CSV_FILENAME "data.csv"
#endif

// Глобальные переменные
char csv_buffer[CSV_BUFFER_SIZE];
bool csv_file_initialized = false;
uint32_t data_counter = 0;
uint32_t buffer_index = 0;
FIL csv_file;

void csv_logger_init(void) {
	if (!csv_file_initialized) {
		FRESULT fr = f_open(&csv_file, CSV_FILENAME,
		FA_WRITE | FA_CREATE_ALWAYS);
		if (fr == FR_OK) {
			// ИЗМЕНЯЕМ ЗАГОЛОВОК на новый формат
			f_puts("timestamp; adc_mv; delta\n", &csv_file);
			f_sync(&csv_file);
			csv_file_initialized = true;
		}
		buffer_index = 0;
		data_counter = 0;
	}
}

// НОВАЯ ФУНКЦИЯ с нужным форматом
//void csv_logger_write_timestamp(uint32_t timestamp, uint16_t pwm_mv,
//		uint32_t delta) {
//	if (buffer_index >= CSV_BUFFER_SIZE - 30) {
//		csv_logger_flush();
//	}
//
//	// НОВЫЙ ФОРМАТ: timestamp; pwm_mv; delta
//	int len = snprintf(csv_buffer + buffer_index,
//	CSV_BUFFER_SIZE - buffer_index, "%lu;%d;%lu\n", timestamp, pwm_mv, delta);
//
//	if (len > 0 && (buffer_index + len) < CSV_BUFFER_SIZE) {
//		buffer_index += len;
//		data_counter++;
//	} else {
//		csv_logger_flush();
//	}
//}

// Старая функция (оставляем для совместимости)
//void csv_logger_write(uint16_t pwm_mv, uint16_t adc_mv) {
//	uint32_t current_time = HAL_GetTick();
//	int len = snprintf(csv_buffer + buffer_index,
//	CSV_BUFFER_SIZE - buffer_index, "%lu,%d,%d\n", current_time, pwm_mv,
//			adc_mv);
//	if (len > 0 && (buffer_index + len) < CSV_BUFFER_SIZE) {
//		buffer_index += len;
//		data_counter++;
//	} else {
//		csv_logger_flush();
//	}
//}

// НОВАЯ ФУНКЦИЯ с нужным форматом - ТОЛЬКО ОДНО ОПРЕДЕЛЕНИЕ
void csv_logger_write_timestamp(uint32_t timestamp, uint16_t adc_mv,
		uint32_t delta_32) {
	if (buffer_index >= CSV_BUFFER_SIZE - 30) {
		csv_logger_flush();
	}

	// Конвертируем delta из uint32_t в uint16_t с проверкой
	uint16_t delta_16;
	if (delta_32 > 65535) {
		delta_16 = 65535;  // Или другое значение для обозначения переполнения
	} else {
		delta_16 = (uint16_t) delta_32;
	}

	// ФОРМАТ: timestamp; adc_mv; delta
	int len = snprintf(csv_buffer + buffer_index,
	CSV_BUFFER_SIZE - buffer_index, "%lu;%d;%d\n",  // %d для uint16_t delta
			timestamp, adc_mv, delta_16);

	if (len > 0 && (buffer_index + len) < CSV_BUFFER_SIZE) {
		buffer_index += len;
		data_counter++;
	} else {
		csv_logger_flush();
	}
}

void csv_logger_flush(void) {
	if (buffer_index > 0 && csv_file_initialized) {
		UINT bytes_written;
		FRESULT fr = f_write(&csv_file, csv_buffer, buffer_index,
				&bytes_written);
		if (fr == FR_OK) {
			f_sync(&csv_file);
			buffer_index = 0;
		}
	}
}

uint32_t csv_logger_get_count(void) {
	return data_counter;
}
